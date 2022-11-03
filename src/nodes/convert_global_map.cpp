/**
 * @file path_listener.cpp
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma region INCLUDES

// ros related includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <geometry_msgs/PoseStamped.h>

#include <bondcpp/bond.h>

// c++ standard related includes
#include <sstream>
#include <iostream>

// external bibs
#include <highfive/H5File.hpp>

// own includes
#include <loop_closure/params/loop_closure_params.h>
#include <loop_closure/map/global_map.h>
#include <loop_closure/map/local_map.h>
#include <loop_closure/ray_tracer/ray_tracer.h>
#include <loop_closure/path/path.h>
#include <loop_closure/visualization/ros_viewhelper.h>
#include <loop_closure/util/point.h>
#include <loop_closure/util/eigen_vs_ros.h>
#include <loop_closure/util/update_tsdf.h>
#include <loop_closure/gtsam/gtsam_wrapper.h>
#include <loop_closure/util/lc_evaluator.h>
#include <loop_closure/map/map_updater.h>
#include <loop_closure/coordinate_systems/coord_sys_transform.h>
#include <loop_closure/util/csv_wrapper.h>
#include <loop_closure/util/evaluation.h>

// transform between ros and eigen
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

// pcl includes
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#pragma endregion

using namespace message_filters;

#pragma region GLOBAL_VARIABLES

// parametrization
LoopClosureParams params;

// own stuff
RayTracer *tracer;
std::shared_ptr<LocalMap> local_map_ptr;
std::shared_ptr<GlobalMap> global_map_ptr;
std::shared_ptr<CSVWrapper> csv_wrapper_ptr;

Path *path;

// ROS //
ros::Publisher tsdf_publisher;

// translation error against ground truth
CSVWrapper::CSVObject *translation_error;


#pragma endregion

/**
 * @brief method used to initiate all the objects associated with this node
 *
 */
void init_obj()
{
    // create maps
    global_map_ptr.reset(new GlobalMap(params.map.filename.string()));
    local_map_ptr.reset(new LocalMap(params.map.size.x(), params.map.size.y(), params.map.size.y(), global_map_ptr));
    csv_wrapper_ptr.reset(new CSVWrapper(params.loop_closure.csv_save_path, ','));

    // init ray tracer
    tracer = new RayTracer(params, local_map_ptr, global_map_ptr);

    // init path
    path = new Path();
    path->attach_raytracer(tracer);

    // evaluation
    translation_error = csv_wrapper_ptr->create_object("translation_error");
}



/**
 * @brief Main method
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "convert_global_map");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // specify ros loop rate
    ros::Rate loop_rate(1.0f);

    // load params from nodehandle
    params = LoopClosureParams(nh);
    init_obj();

    std::cout << "Dir: " << params.map.dir.string() << std::endl;
    params.map.filename = boost::filesystem::path("/home/patrick/maps/converted/test_converted.h5");
    std::cout << "New file name: " << params.map.filename.string() << std::endl;

    GlobalMap tmp = GlobalMap(params.map);
    auto old_data = global_map_ptr->read_old_format();
    auto path = global_map_ptr->get_path();

    tmp.write_path(path);


    for(auto chunk : old_data)
    {

    }

    std::cout << "Done converting.. " << std::endl;

    ros::spin();

    return EXIT_SUCCESS;
}
