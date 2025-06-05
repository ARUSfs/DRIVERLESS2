/**
 * @file cropping.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Contains auxiliar functions used in the algorithm to crop the point cloud received from the lidar.
 */


#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

namespace Cropping
{
    /**
    * @brief Create crop filter function using the class CropBox from pcl.
    * @tparam PointT Point type (e.g., pcl::PointXYZ, pcl::PointXYZI)
    */
    template<typename PointT>
    void box_filter(typename pcl::PointCloud<PointT>::Ptr& cloud, typename pcl::PointCloud<PointT>::Ptr& output_cloud, 
        double min_x, double min_y, double min_z, double max_x, double max_y, double max_z)
    {
        pcl::PassThrough<PointT> pass_x;
        pass_x.setInputCloud(cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(min_x, max_x);
        pass_x.filter(*output_cloud);

        pcl::PassThrough<PointT> pass_y;
        pass_y.setInputCloud(output_cloud);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(min_y, max_y);
        pass_y.filter(*output_cloud);

        pcl::PassThrough<PointT> pass_z;
        pass_z.setInputCloud(output_cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(min_z, max_z);
        pass_z.filter(*output_cloud);
    }

    /**
    * @brief Axis filter using PassThrough for a general point type.
    * @tparam PointT Point type (e.g., pcl::PointXYZ, pcl::PointXYZI)
    */
    template<typename PointT>
    void axis_filter(typename pcl::PointCloud<PointT>::Ptr& cloud, typename pcl::PointCloud<PointT>::Ptr& output_cloud, 
        const std::string& axis, double min_v, double max_v)
    {
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName(axis);
        pass.setFilterLimits(min_v, max_v);
        pass.filter(*output_cloud);
    }
}
