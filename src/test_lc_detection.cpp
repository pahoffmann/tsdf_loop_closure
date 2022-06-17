/**
 * @file path_listener.cpp
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief
 * @version 0.1
 * @date 2022-05-29
 *
 * @copyright Copyright (c) 2022
 *
 */

// ros related includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

// c++ standard related includes
#include <sstream>
#include <iostream>

// external bibs
#include <highfive/H5File.hpp>

// relative includes
#include "map/global_map.h"
#include "map/local_map.h"
#include "ray_tracer/ray_tracer.h"
#include "path/path.h"
#include "options/options_reader.h"

RayTracer *tracer;
std::shared_ptr<LocalMap> local_map_ptr;
std::shared_ptr<GlobalMap> global_map_ptr;
Path *path;
int side_length_xy;
int side_length_z;
lc_options_reader *options;


// ros
ros::Subscriber pose_sub;

void pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    ROS_INFO("Retrieved a path!");

    auto pose = msg->pose.pose;

    std::cout << "Got odometry message. Pose:" << pose.position.x << " | " << pose.position.y << " | " << pose.position.z << std::endl;
}

/**
 * @brief initializes the local and global map
 * @todo don't harcode this
 *
 */
void initMaps()
{
    global_map_ptr = std::make_shared<GlobalMap>(options->get_map_file_name(), 0.0, 0.0);
    // todo: this is currently hardcoded. is there a way to retrieve the local map size from the hdf5?
    // FIXME: there is currently no way to read metadata from the map, we should introduce this in the map implementation
    local_map_ptr = std::make_shared<LocalMap>(201, 201, 95, global_map_ptr, true); // still hardcoded af

    auto &size = local_map_ptr.get()->get_size();
    side_length_xy = size.x() * MAP_RESOLUTION / 1000.0f;
    side_length_z = size.z() * MAP_RESOLUTION / 1000.0f;

    std::cout << "Finished init the maps" << std::endl;
}

void initOptions(int argc, char **argv)
{
    // read options from cmdline
    options = new lc_options_reader();
    int status = options->read_options(argc, argv);

    if (status == 1)
    {
        std::cout << "[CLI] Terminate node, because there were some errors while reading from cmd-line" << std::endl;

        exit(EXIT_FAILURE);
    }
    else if (status == 2)
    {
        std::cout << "[CLI] Terminate node, because help was requested" << std::endl;

        return exit(EXIT_FAILURE);
    }
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
    initOptions(argc, argv);
    initMaps();

    ros::init(argc, argv, "test_lc_detection_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // specify ros loop rate
    ros::Rate loop_rate(10);

    pose_sub = n.subscribe("/pose", 1, pose_callback);

    // ros loop
    while (ros::ok())
    {
        // more ros related stuff
        ros::spinOnce();

        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
