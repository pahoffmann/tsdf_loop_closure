#pragma once

/**
 * @author Malte Hillmann (mhillmann)
 * @author Marc Eisoldt (meisoldt)
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <loop_closure/util/point.h>
#include <loop_closure/map/local_map.h>
#include <unordered_map>
#include <loop_closure/map/attribute_data_model.h>
#include <loop_closure/util/point.h>

namespace std
{
    template <>
    struct hash<Eigen::Vector3i>
    {
        std::size_t operator()(Eigen::Vector3i const &p) const noexcept
        {
            long long v = ((long long)p.x() << 32) ^ ((long long)p.y() << 16) ^ (long long)p.z();
            return std::hash<long long>()(v);
        }
    };
}

// using Map = std::unordered_map<Point, std::pair<float, float>>;
using FastMap = std::unordered_map<Eigen::Vector3i, TSDFEntry>;

Eigen::Vector3i to_point(const Vector3f &vec);

/**
 * @brief Generate new TSDF data and weights based on the new point data and update the given map
 *
 * @param scan_points Points from which the TSDF values should be calculated
 * @param scanner_pos current position of the laser scanner
 * @param buffer map which should be updated with the new data
 * @param tsdf_method Method which should be used for the tsdf calculation
 * @param tau Truncation distance for the TSDF values (in map resolution)
 * @param max_weight Maximum weight for every TSDF cell in the map
 */
void update_tsdf(const std::vector<Eigen::Vector3i> &scan_points, const Eigen::Vector3i &scanner_pos, const Eigen::Vector3i &up,
                 LocalMap &buffer, int tau, int max_weight, int map_resolution);
