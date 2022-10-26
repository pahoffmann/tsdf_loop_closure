/**
 * @file loop_closure_node.cpp
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief Loop Closure Node used to detect and apply loop closure to a tsdf based slam
 * @version 0.1
 * @date 2022-03-14
 *
 * @copyright Copyright (c) 2022
 *
 */

// ros related includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

// c++ standard related includes
#include <sstream>
#include <iostream>

// external bibs
#include <highfive/H5File.hpp>

#include <loop_closure/params/loop_closure_params.h>

#include <loop_closure/map/global_map.h>
#include <loop_closure/map/local_map.h>
#include <loop_closure/util/colors.h>
#include <loop_closure/path/path.h>
#include <loop_closure/util/point.h>
#include <loop_closure/ray_tracer/ray_tracer.h>
#include <loop_closure/serialization/read_path_json.h>
#include <loop_closure/data_association/association_manager.h>
#include <loop_closure/visualization/ros_viewhelper.h>
#include <loop_closure/path/path_exploration.h>
#include <loop_closure/test/math_test.h>

// Configuration stuff //

LoopClosureParams params;

// ROS STUFF //
visualization_msgs::Marker ray_markers;
visualization_msgs::Marker bb_marker;
visualization_msgs::Marker tsdf_map;             // marker for the tsdf map (local)
visualization_msgs::Marker tsdf_map_full_before; // marker for the full tsdf map (global) (before map update)
visualization_msgs::Marker tsdf_map_full_after;  // marker for the full tsdf map (global) (after map update)

std::vector<visualization_msgs::Marker> loop_visualizations;
visualization_msgs::Marker path_marker;
visualization_msgs::Marker updated_path_marker;
visualization_msgs::Marker pose_marker;
visualization_msgs::Marker chunk_marker;
visualization_msgs::Marker tsdf_read_marker;
visualization_msgs::Marker bresenham_marker;

// used to store sinuglar updates between the path and the path after all the updates
std::vector<visualization_msgs::Marker> path_marker_updates;

// ros publishers declaration
ros::Publisher tsdf_before_publisher;
ros::Publisher tsdf_after_publisher;
ros::Publisher tsdf_read_publisher;
ros::Publisher pose_publisher;
ros::Publisher path_publisher;
ros::Publisher updated_path_publisher;
ros::Publisher ray_publisher;
ros::Publisher bb_publisher;
ros::Publisher chunk_publisher;
ros::Publisher bresenham_int_publisher;
ros::Publisher loop_pub;

/// Map Stuff ///
std::shared_ptr<GlobalMap> global_map_ptr_;
std::shared_ptr<LocalMap> local_map_ptr_;

/// Ray Tracer ///
RayTracer *ray_tracer;

/// Path ///
Path *path;
Path updated_path;

/**
 * @brief initializes the local and global map
 *
 */
void initMaps(bool cleanup)
{
    global_map_ptr_ = std::make_shared<GlobalMap>(params.map.filename.string()); // create a global map, use it's attributes
    auto attribute_data = global_map_ptr_->get_attribute_data();

    local_map_ptr_ = std::make_shared<LocalMap>(attribute_data.get_map_size_x(),
                                                attribute_data.get_map_size_y(),
                                                attribute_data.get_map_size_z(),
                                                global_map_ptr_, true);

    std::cout << "Finished init the maps" << std::endl;
}

/**
 * @brief initializes the path and tries to find path positions, if none was specified
 */
void initialize_path()
{
    // init path and read from specified source (mostly hdf5)
    path = new Path();
    path->attach_raytracer(ray_tracer);

    // retrieve path from various locations
    // if the global map does not have a path, we do path exploration, just in case.
    if (global_map_ptr_->has_path())
    {
        auto poses = global_map_ptr_->get_path();
        for (auto pose : poses)
        {
            path->add_pose(pose);
        }
    }
    else
    {
        std::cerr << "[LoopClosureNode] The globalmap has no path poses" << std::endl;
        exit(EXIT_FAILURE);
    }
}

/**
 * @brief Populate the ros publishers
 *
 */
void populate_publishers(ros::NodeHandle &n)
{
    tsdf_before_publisher = n.advertise<visualization_msgs::Marker>("tsdf_before", 1, true);
    tsdf_after_publisher = n.advertise<visualization_msgs::Marker>("tsdf_after", 1, true);
    tsdf_read_publisher = n.advertise<visualization_msgs::Marker>("tsdf_read", 1, true);
    path_publisher = n.advertise<visualization_msgs::Marker>("path", 1, true);
    updated_path_publisher = n.advertise<visualization_msgs::Marker>("path_updated", 1, true);
    ray_publisher = n.advertise<visualization_msgs::Marker>("rays", 100);
    bb_publisher = n.advertise<visualization_msgs::Marker>("bounding_box", 1, true);
    chunk_publisher = n.advertise<visualization_msgs::Marker>("chunk_poses", 1, true);
    bresenham_int_publisher = n.advertise<visualization_msgs::Marker>("bresenham_intersections", 1, true);
    loop_pub = n.advertise<visualization_msgs::Marker>("loop", 1);
}

void populate_markers()
{
    // create a marker for the updated map
    updated_path_marker = ROSViewhelper::initPathMarker(&updated_path);

    auto gm_data = global_map_ptr_->get_full_data();
    tsdf_map_full_after = ROSViewhelper::marker_from_gm_read(gm_data);
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
    ros::init(argc, argv, "test_partial_update");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // read params
    params = LoopClosureParams(nh);

    // init local and global maps
    initMaps(false);

    // define stuff for raytracer
    ray_tracer = new RayTracer(params, local_map_ptr_, global_map_ptr_);

    initialize_path();
    path_marker = ROSViewhelper::initPathMarker(path);

    // generate publishers
    populate_publishers(n);

    // generate marker for before map
    auto gm_data = global_map_ptr_->get_full_data();
    tsdf_map_full_before = ROSViewhelper::marker_from_gm_read(gm_data);

    // translate the whole path
    auto translated_path = path->translate_ret(Vector3f(10.0f, 0.0f, 0.0f), 0, path->get_length() - 1);

    for (int i = path->get_length() -1; i >= 0; i--)
    {
        std::cout << "Cleaning from pose " << i << "!" << std::endl;
        auto pose = path->at(i);
        ray_tracer->local_removal(pose);

        local_map_ptr_->write_back();
    }

    // now populate the markers
    populate_markers();

#ifdef DEBUG
    std::cout
        << "Before update size: " << tsdf_map_full_before.points.size() << std::endl;
    std::cout << "After update size: " << tsdf_map_full_after.points.size() << std::endl;
#endif

    // auto single_marker = ROSViewhelper::initPoseAssociationVisualization(global_map_ptr_, path->at(0), 0);

    // specify ros loop rate
    ros::Rate loop_rate(10);

    // some stuff doesnt need to be published every iteration...
    path_publisher.publish(path_marker);
    updated_path_publisher.publish(updated_path_marker);
    tsdf_before_publisher.publish(tsdf_map_full_before);
    tsdf_after_publisher.publish(tsdf_map_full_after);

    ros::spinOnce();

    // ros loop
    while (ros::ok())
    {
        // more ros related stuff
        ros::spinOnce();

        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}