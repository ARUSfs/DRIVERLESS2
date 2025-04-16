#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stdlib.h>
#include <time.h> 
#include <thread>
#include <mutex>  
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>

namespace GroundRemove 
{
	std::mutex regionmutex;
	Eigen::MatrixXf normal_;
	float th_dist_d_ = 0;
	int num_iter_ = 5;
	int num_lpr_ = 20;
	double th_seeds_ = 1.0;
	double th_dist_ = 0.15;

		
	bool point_cmp(pcl::PointXYZI a, pcl::PointXYZI b) {
		return a.z < b.z;
	}

	void extract_initial_seeds_(
			const pcl::PointCloud<pcl::PointXYZI>& p_sorted,
			pcl::PointCloud<pcl::PointXYZI>& g_seeds_pc) {
		// LPR is the mean of low point representative
		double sum = 0;
		int cnt = 0;
		// Calculate the mean height value.
		for (int i = 0; i < p_sorted.points.size() && cnt < num_lpr_; ++i) {
			sum += p_sorted.points[i].z;
			cnt++;
		}
		double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0
		g_seeds_pc.clear();
		// iterate pointcloud, filter those height is less than lpr.height+th_seeds_
		for (int i = 0; i < p_sorted.points.size(); ++i) {
			if (p_sorted.points[i].z < lpr_height + th_seeds_) {
				g_seeds_pc.points.push_back(p_sorted.points[i]);
			}
		}
		// return seeds points
	}

	void estimate_plane_(
			const pcl::PointCloud<pcl::PointXYZI>& g_ground_pc) {
		// Create covarian matrix in single pass.
		Eigen::Matrix3f cov;
		Eigen::Vector4f pc_mean;

		pcl::computeMeanAndCovarianceMatrix(g_ground_pc, cov, pc_mean);

		// Singular Value Decomposition: SVD
		Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
		// use the least singular vector as normal
		normal_ = Eigen::MatrixXf(3,1);
		normal_.fill(0.0);
		normal_ = (svd.matrixU().col(2));

		float d_ = -(normal_(0, 0) * pc_mean(0) + normal_(1, 0) * pc_mean(1)
				+ normal_(2, 0) * pc_mean(2));

		// set distance threhold to `th_dist - d`
		th_dist_d_ = th_dist_ - d_;
	}

	void RemoveGround_Thread(pcl::PointCloud<pcl::PointXYZI>& cloudIn,
			pcl::PointCloud<pcl::PointXYZI>& cloudgc,
			pcl::PointCloud<pcl::PointXYZI>& cloudngc,
			pcl::PointCloud<pcl::PointXYZI>& g_ground_pc1,
			pcl::PointCloud<pcl::PointXYZI>& g_not_ground_pc1) {

		std::lock_guard < std::mutex > lock(regionmutex);
		pcl::PointCloud<pcl::PointXYZI>::Ptr g_seeds_pc(
				new pcl::PointCloud<pcl::PointXYZI>());

		std::sort(cloudIn.points.begin(), cloudIn.points.end(), point_cmp);

		extract_initial_seeds_(cloudIn, *g_seeds_pc);

		cloudgc = *g_seeds_pc;

		for (int i = 0; i < num_iter_; ++i) {

			estimate_plane_(cloudgc);

			cloudgc.clear();
			cloudngc.clear();

			float xd = normal_(0, 0);
			float yd = normal_(1, 0);
			float zd = normal_(2, 0);
			for (auto p : cloudIn.points) {
				float distance = p.x * xd + p.y * yd + p.z * zd;

				float range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
				float delta = 0.006 * range;
				if (distance < th_dist_d_ - delta) {
					cloudgc.points.push_back(p); // Ground
				} else {
					cloudngc.points.push_back(p); // Cones
				}

			}

		}

		for (int k = 0; k < cloudgc.points.size(); ++k) {

			g_ground_pc1.points.push_back(cloudgc.points[k]);
		}

		for (int k = 0; k < cloudngc.points.size(); ++k) {

			g_not_ground_pc1.points.push_back(cloudngc.points[k]);
		}

	}

	void RemoveGround(pcl::PointCloud<pcl::PointXYZI>& cloudIn,
					pcl::PointCloud<pcl::PointXYZI>& g_ground_pc,
					pcl::PointCloud<pcl::PointXYZI>& g_not_ground_pc) {
						
		pcl::PointCloud<pcl::PointXYZI>::Ptr g_ground_pc1(
				new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr g_not_ground_pc1(
				new pcl::PointCloud<pcl::PointXYZI>());

		const int num_sectors = 7; 
		const int num_rings = 15;    

		const float max_range = 70.0;
		const float sector_angle = M_PI / float(num_sectors);
		const float ring_step = max_range / float(num_rings);

		std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> grid(num_rings, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>(num_sectors));

		// Create the grid
		for (int r = 0; r < num_rings; ++r) {
			for (int s = 0; s < num_sectors; ++s) {
				grid[r][s] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
			}
		}
		std::vector<int> sector_count(num_sectors, 0);
		// Assign points to the grid
		for (auto& p : cloudIn.points) {
			float d = std::sqrt(p.x * p.x + p.y * p.y);
			if (d > max_range || d < 1.0) {
				g_not_ground_pc.push_back(p);
				continue;
			}

			float angle = std::atan2(p.y, p.x);
			if (angle < 0) angle += M_PI;

			int r_idx = std::min(int(d / ring_step), num_rings - 1);
			int s_idx = std::min(int(angle / sector_angle), num_sectors - 1);

			// Visual encoding
            //p.intensity = 20.0f * s_idx / float(num_sectors);
			grid[r_idx][s_idx]->push_back(p);
		}

		// Filter each grid
		for (int s = 0; s < num_sectors; ++s) {
			std::vector<std::thread> thread_vec(num_rings);

			for (int r = 0; r < num_rings; ++r) {
				thread_vec[r] = std::thread(&RemoveGround_Thread,
				std::ref(*grid[r][s]), std::ref(*g_ground_pc1),
				std::ref(*g_not_ground_pc1), std::ref(g_ground_pc),
				std::ref(g_not_ground_pc));
				}

			for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it) {
				it->join();
			}
	
		}
		g_ground_pc.width = g_ground_pc.points.size();
		g_ground_pc.height = 1;
		g_ground_pc.is_dense = true;

		g_not_ground_pc.width = g_not_ground_pc.points.size();
		g_not_ground_pc.height = 1;
		g_not_ground_pc.is_dense = true;
	}
}