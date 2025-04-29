#include <pcl/point_types.h>
#include <iostream>


std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<PointXYZIRingTime>& cloud) {
    std::vector<Eigen::Vector3f> vec;
    vec.reserve(cloud.size());
    for (const auto& pt : cloud.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      vec.emplace_back(pt.x, pt.y, pt.z);
    }
    return vec;
}


Eigen::Matrix4f tf2TransformToEigen(const tf2::Transform &tf)
{
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();

    // Rotación
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            mat(i, j) = tf.getBasis()[i][j];

    // Traslación
    mat(0, 3) = tf.getOrigin().x();
    mat(1, 3) = tf.getOrigin().y();
    mat(2, 3) = tf.getOrigin().z();

    return mat;
}