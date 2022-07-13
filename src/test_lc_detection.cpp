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
#include "visualization/ros_viewhelper.h"
#include "util/point.h"
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

RayTracer *tracer;
std::shared_ptr<LocalMap> local_map_ptr;
std::shared_ptr<GlobalMap> global_map_ptr;
Path *path;
int side_length_xy;
int side_length_z;
lc_options_reader *options;

// distance between poses (min) in path
int PATH_DIST = 0.2;

// ros
ros::Subscriber pose_sub;
ros::Publisher path_pub;
ros::Publisher loop_pub;

nav_msgs::Path ros_path;

/**
 * @brief method, which converts a eigen vector to a ros point stamped. if no stamp is passed, it defaults to the current time
 *
 * @param vec
 * @param stamp
 * @return geometry_msgs::PointStamped
 */
geometry_msgs::PointStamped eigen_vec_to_pose_stamped(Eigen::Vector3f vec, ros::Time stamp = ros::Time::now())
{
    geometry_msgs::PointStamped point;

    point.header.stamp = stamp;
    point.header.frame_id = "map";
    point.point.x = vec.x();
    point.point.y = vec.y();
    point.point.z = vec.z();

    return point;
}

geometry_msgs::PoseStamped lc_pose_to_pose_stamped(Pose lc_pose, ros::Time stamp = ros::Time::now())
{
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = stamp;
    pose.header.frame_id = "map";
    pose.pose.position.x = lc_pose.pos.x();
    pose.pose.position.y = lc_pose.pos.y();
    pose.pose.position.z = lc_pose.pos.z();

    pose.pose.orientation.x = lc_pose.quat.x();
    pose.pose.orientation.y = lc_pose.quat.y();
    pose.pose.orientation.z = lc_pose.quat.z();
    pose.pose.orientation.w = lc_pose.quat.w();

    return pose;
}

/**
 * @brief method, which reacts to new odometry messages
 *
 * @param msg
 */
void pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    auto pose = msg->pose.pose;

    // convert pose to own pose struct
    Eigen::Vector3f vec(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaternionf quat;
    quat.x() = pose.orientation.x;
    quat.y() = pose.orientation.y;
    quat.z() = pose.orientation.z;
    quat.w() = pose.orientation.w;

    // when the path length is 0, there is no need to proceed here
    if(path->get_length() == 0) {
        return;
    }

    auto last_pose = path->at(path->get_length() - 1);

    // guard clause
    if (!((vec - last_pose->pos).norm() > PATH_DIST))
    {
        return;
    }

    // if the distance to the last saved pose is big enough, we add a pose to the path

    // create pose
    Pose lc_pose;
    lc_pose.quat = quat;
    lc_pose.pos = vec;

    // add to path
    path->add_pose(lc_pose);

    // after every new pose, we search for loops
    // beginning at first pose
    // with max distance of one meter
    // with at least 10 meters traveled
    auto res = path->find_loop_greedy(0, 1, 10);

    // if no loop found, ignore this iteration
    if (res.first == -1 && res.second == -1)
    {
        return;
    }

    // else: loop found, display it.
    auto marker = ROSViewhelper::init_loop_detected_marker(path->at(res.first)->pos, path->at(res.second)->pos);

    std::cout << "Loop found" << std::endl;

    ros_path.poses.push_back(lc_pose_to_pose_stamped(lc_pose, msg->header.stamp));

    path_pub.publish(ros_path);
    loop_pub.publish(marker);
}

/**
 * @brief initializes the local and global map
 * @todo don't harcode this
 *
 */
void init_maps()
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
 * @brief method used to initiate all the objects associated with this test case
 *
 */
void init_obj()
{
    // initialize the map based ob the passed arguments first
    init_maps();

    // init ray tracer
    tracer = new RayTracer(options, local_map_ptr, global_map_ptr);

    // init path
    path = new Path(tracer);

    ros_path.header.frame_id = "map";
    ros_path.header.stamp = ros::Time::now();
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

    ros::init(argc, argv, "test_lc_detection_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    init_obj();

    // specify ros loop rate
    ros::Rate loop_rate(10);

    pose_sub = n.subscribe("/base_footprint_pose_ground_truth", 1, pose_callback);

    path_pub = n.advertise<nav_msgs::Path>("/test_path", 1);
    loop_pub = n.advertise<visualization_msgs::Marker>("/loop", 1);

    // ros loop
    while (ros::ok())
    {
        // more ros related stuff
        ros::spinOnce();

        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
