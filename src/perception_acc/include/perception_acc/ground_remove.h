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
	int num_seg_ = 8;

		
	bool point_cmp(PointXYZIRingTime a, PointXYZIRingTime b) {
		return a.z < b.z;
	}

	void extract_initial_seeds_(
			const pcl::PointCloud<PointXYZIRingTime>& p_sorted,
			pcl::PointCloud<PointXYZIRingTime>& g_seeds_pc) {
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
			const pcl::PointCloud<PointXYZIRingTime>& g_ground_pc) {
		// Create covarian matrix in single pass.
		// TODO: compare the efficiency.
		Eigen::Matrix3f cov;
		Eigen::Vector4f pc_mean;

		pcl::computeMeanAndCovarianceMatrix(g_ground_pc, cov, pc_mean);

		//computeMeanAndCovarianceMatrix(g_ground_pc, conv, mean);
		// Singular Value Decomposition: SVD
		Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
		// use the least singular vector as normal
		normal_ = Eigen::MatrixXf(3,1);
		normal_.fill(0.0);
		normal_ = (svd.matrixU().col(2));
		// mean ground seeds value
		//Eigen::Vector3f seeds_mean = pc_mean.head<3>();

		// according to normal.T*[x,y,z] = -d
		//float d_ = -(normal_.transpose()*seeds_mean)(0,0);

		float d_ = -(normal_(0, 0) * pc_mean(0) + normal_(1, 0) * pc_mean(1)
				+ normal_(2, 0) * pc_mean(2));

		// set distance threhold to `th_dist - d`
		th_dist_d_ = th_dist_ - d_;

		// return the equation parameters
	}

	void RemoveGround_Thread(pcl::PointCloud<PointXYZIRingTime>& cloudIn,
			pcl::PointCloud<PointXYZIRingTime>& cloudgc,
			pcl::PointCloud<PointXYZIRingTime>& cloudngc,
			pcl::PointCloud<PointXYZIRingTime>& g_ground_pc1,
			pcl::PointCloud<PointXYZIRingTime>& g_not_ground_pc1) {

		std::lock_guard < std::mutex > lock(regionmutex);
		pcl::PointCloud<PointXYZIRingTime>::Ptr g_seeds_pc(
				new pcl::PointCloud<PointXYZIRingTime>());

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
				if (distance < th_dist_d_) {
					//g_all_pc->points[r].label = 1u;// means ground
					cloudgc.points.push_back(p);
				} else {
					//g_all_pc->points[r].label = 0u;// means not ground and non clusterred
					cloudngc.points.push_back(p);
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

	void RemoveGround(pcl::PointCloud<PointXYZIRingTime>& cloudIn,
			pcl::PointCloud<PointXYZIRingTime>& g_ground_pc,
			pcl::PointCloud<PointXYZIRingTime>& g_not_ground_pc) {
		pcl::PointCloud<PointXYZIRingTime>::Ptr g_seeds_region1(
				new pcl::PointCloud<PointXYZIRingTime>());
		pcl::PointCloud<PointXYZIRingTime>::Ptr g_seeds_region2(
				new pcl::PointCloud<PointXYZIRingTime>());
		pcl::PointCloud<PointXYZIRingTime>::Ptr g_seeds_region3(
				new pcl::PointCloud<PointXYZIRingTime>());
		pcl::PointCloud<PointXYZIRingTime>::Ptr g_ground_pc1(
				new pcl::PointCloud<PointXYZIRingTime>());
		pcl::PointCloud<PointXYZIRingTime>::Ptr g_not_ground_pc1(
				new pcl::PointCloud<PointXYZIRingTime>());

		std::vector<pcl::PointCloud<PointXYZIRingTime>::Ptr> pcregion(num_seg_);
		for (int i = 0; i < num_seg_; ++i) {
    		pcregion[i] = pcl::PointCloud<PointXYZIRingTime>::Ptr(new pcl::PointCloud<PointXYZIRingTime>);
		}

		float xmin = -35, xmax = 35, ymin = -30, ymax = 30, zmin = -1.0, zmax = 0.5;
		float regionsize = (ymax - ymin) / static_cast<float>(num_seg_);
		for (int i = 0; i < cloudIn.points.size(); ++i) {
			if (cloudIn.points[i].z < 0.4) {
				float y = cloudIn.points[i].y;
				for (int r = 0; r < num_seg_; ++r) {
					float upper = ymax - r * regionsize;
					float lower = ymax - (r + 1) * regionsize;
					if (y < upper && y > lower) {
						pcregion[r]->points.push_back(cloudIn.points[i]);
						break;
					}
				}
			} else {
				g_not_ground_pc.points.push_back(cloudIn.points[i]);
			}
		}

		cloudIn.clear();

		std::vector<std::thread> thread_vec(num_seg_);

		for (int ri = 0; ri < num_seg_; ++ri) {

			thread_vec[ri] = std::thread(&RemoveGround_Thread,
					std::ref(*pcregion[ri]), std::ref(*g_ground_pc1),
					std::ref(*g_not_ground_pc1), std::ref(g_ground_pc),
					std::ref(g_not_ground_pc));

		}

		for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it) {
			it->join();
		}
	}
}