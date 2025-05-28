#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm> 
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

  
float estimateZOffsetMedian(const pcl::PointCloud<PointXYZIRingTime>& src_xy,
                            const pcl::PointCloud<PointXYZIRingTime>& tgt,
                            float xy_radius   = 0.30f,   
                            int   max_samples = 100) 
{
  pcl::KdTreeFLANN<PointXYZIRingTime> kdtree;
  kdtree.setInputCloud(tgt.makeShared());

  std::vector<float> dz; dz.reserve(max_samples);
  std::vector<int>   idx(1);
  std::vector<float> dist2(1);

  const int step = std::max<int>(1, src_xy.size() / max_samples);

  for (size_t i = 0; i < src_xy.size(); i += step)
  {
    const auto& p = src_xy[i];
    if (kdtree.nearestKSearch(p, 1, idx, dist2) > 0)
    {
      const auto& q = tgt[idx[0]];
      float dx = p.x - q.x, dy = p.y - q.y;
      if (dx*dx + dy*dy < xy_radius*xy_radius)
        dz.push_back(q.z - p.z);             
    }
  }
  if (dz.empty()) return 0.f;              

  std::nth_element(dz.begin(), dz.begin()+dz.size()/2, dz.end());
  return dz[dz.size()/2];                   
}