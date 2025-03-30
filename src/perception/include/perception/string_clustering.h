namespace GroundFiltering2
{
    void RemoveGroundByRings(
        const pcl::PointCloud<PointXYZIRingTime>::ConstPtr& cloud_in,
        pcl::PointCloud<PointXYZIRingTime>::Ptr& ground_cloud,
        pcl::PointCloud<PointXYZIRingTime>::Ptr& non_ground_cloud,
        float max_slope_deg = 10.0f,
        float max_height_diff = 0.2f)
    {
        std::unordered_map<uint16_t, std::vector<PointXYZIRingTime>> ring_map;

        for (const auto& point : cloud_in->points)
        {
            ring_map[point.ring].push_back(point);
        }

        float max_slope = std::tan(max_slope_deg * M_PI / 180.0f);

        ground_cloud->clear();
        non_ground_cloud->clear();

        for (auto& ring_pair : ring_map)
        {
            auto& ring_points = ring_pair.second;

            if (ring_points.empty()) continue;

            std::sort(ring_points.begin(), ring_points.end(), [](const PointXYZIRingTime& a, const PointXYZIRingTime& b) {
                return (a.x * a.x + a.y * a.y) < (b.x * b.x + b.y * b.y);
            });

            ground_cloud->points.push_back(ring_points[0]);

            for (size_t i = 1; i < ring_points.size(); ++i)
            {
                const auto& prev = ring_points[i - 1];
                const auto& curr = ring_points[i];

                float dx = curr.x - prev.x;
                float dy = curr.y - prev.y;
                float dz = curr.z - prev.z;

                float distance = std::sqrt(dx * dx + dy * dy);
                float slope = distance > 1e-3f ? std::abs(dz / distance) : 0.0f;

                if (slope < max_slope && std::abs(dz) < max_height_diff)
                {
                    ground_cloud->points.push_back(curr);
                }
                else
                {
                    non_ground_cloud->points.push_back(curr);
                }
            }
        }

        if (!ground_cloud->points.empty()) {
            ground_cloud->width = ground_cloud->points.size();
            ground_cloud->height = 1;
            ground_cloud->is_dense = true;
        }

        if (!non_ground_cloud->points.empty()) {
            non_ground_cloud->width = non_ground_cloud->points.size();
            non_ground_cloud->height = 1;
            non_ground_cloud->is_dense = true;
        }
    }
}