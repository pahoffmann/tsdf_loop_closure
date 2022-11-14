#pragma once

/**
 * @author Patrick Hoffmann
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <unordered_map>

#include <loop_closure/util/point.h>
#include <loop_closure/map/local_map.h>
#include <loop_closure/map/attribute_data_model.h>
#include <loop_closure/path/path.h>
#include <loop_closure/params/loop_closure_params.h>
#include <loop_closure/util/update_tsdf.h>
#include <loop_closure/ray_tracer/ray_tracer.h>
#include <loop_closure/util/csv_wrapper.h>

#include <boost/filesystem.hpp>
#include <chrono>
#include <thread>

namespace Map_Updater
{
    const std::string print_prefix = "[Map_Updater]: ";

    /**
     * @brief only update the map for poses, which have been moved at least by transl_delta or rotated at least by rotation_delta
     *
     * @param old_path
     * @param new_path
     * @param transl_delta
     * @param rotation_delta
     * @param clouds
     * @param global_map_ptr
     * @param local_map_ptr
     * @param params
     */
    void partial_map_update(Path *old_path, Path *new_path, float transl_delta, float rotation_delta,
                            std::vector<pcl::PointCloud<PointType>::Ptr> &clouds,
                            GlobalMap *global_map_ptr, LocalMap *local_map_ptr,
                            LoopClosureParams &params, RayTracer *tracer);

    void partial_map_update_reverse(Path *old_path, Path *new_path, float transl_delta, float rotation_delta,
                                    std::vector<pcl::PointCloud<PointType>::Ptr> &clouds,
                                    GlobalMap *global_map_ptr, LocalMap *local_map_ptr,
                                    LoopClosureParams &params);

    void full_map_update(Path *new_path, std::vector<pcl::PointCloud<PointType>::Ptr> &clouds,
                         GlobalMap *global_map_ptr, LocalMap *local_map_ptr,
                         LoopClosureParams &params, std::string suffix,
                         CSVWrapper::CSVRow &header_row,
                         CSVWrapper::CSVRow &shift_time,
                         CSVWrapper::CSVRow &update_time);
}