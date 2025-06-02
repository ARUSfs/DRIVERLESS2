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
    void crop_filter_cropbox(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double& Mx, double& My, double& Mz)
    {
        // Configure the cropping filter
        pcl::CropBox<pcl::PointXYZI> crop_box_filter;
        crop_box_filter.setInputCloud(cloud);
        crop_box_filter.setMin(Eigen::Vector4f(0, -My, 0.05, 1.0));
        crop_box_filter.setMax(Eigen::Vector4f(Mx, My, Mz, 1.0));
    
        // Store the cropped cloud
        crop_box_filter.filter(*cloud);
    }   
}
