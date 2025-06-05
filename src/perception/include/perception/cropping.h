/**
 * @file cropping.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Contains auxiliar functions used in the algorithm to crop the point cloud received from the lidar.
 */


#include <pcl/filters/crop_box.h>


namespace Cropping
{
    /**
    * @brief Create crop filter function using the class CropBox from pcl.
    */
    void crop_filter_cropbox(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud, double min_x, double min_y, double min_z,
                             double max_x, double max_y, double max_z)
    {
        // Set the crop box limits
        pcl::CropBox<pcl::PointXYZI> crop_box_filter;
        crop_box_filter.setInputCloud(cloud);
        crop_box_filter.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
        crop_box_filter.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));

        // Store the cropped cloud
        crop_box_filter.filter(*output_cloud);
    }
}
