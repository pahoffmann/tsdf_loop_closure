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
#include <loop_closure/map/global_map.h>
#include <loop_closure/map/local_map.h>
#include <loop_closure/ray_tracer/ray_tracer.h>
#include <loop_closure/path/path.h>
#include <loop_closure/options/options_reader.h>
#include <loop_closure/visualization/ros_viewhelper.h>
#include <loop_closure/util/point.h>
#include <loop_closure/util/eigen_vs_ros.h>
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
float PATH_DIST = 0.2f;
float MAX_DIST_LC = 1.0f;
float MIN_TRAVELED_LC = 10.0f;

float cur_length = 0.0f;
Eigen::Vector3f last_vec = Vector3f::Zero();
float last_path_idx = 0;

// ros
ros::Subscriber pose_sub;
ros::Publisher path_pub;
ros::Publisher path_marker_pub;
ros::Publisher loop_pub;

nav_msgs::Path ros_path;

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

    // create pose
    Pose lc_pose;
    lc_pose.quat = quat;
    lc_pose.pos = vec;

    // when the path length is 0, there is no need to proceed here
    if (path->get_length() == 0)
    {
        // add first pose to path
        path->add_pose(lc_pose);
        ros_path.poses.push_back(type_transform::lc_pose_to_pose_stamped(lc_pose, msg->header.stamp));

        // save the current pose vec as last vec to ensure, that the path length calc works fine
        last_vec = vec;

        return;
    }

    // save the current length of the "path", the length of the actual path is just an approximation, as it only uses the x poses
    cur_length += (vec - last_vec).norm();
    last_vec = vec;

    std::cout << "Current length: " << cur_length << std::endl;

    auto last_pose = path->at(path->get_length() - 1);

    // guard clause, checking if the distance between the current pose of the robot and the last position is big enough in order to add it to the path.
    if (!((vec - last_pose->pos).norm() > PATH_DIST))
    {
        return;
    }

    // if the distance to the last saved pose is big enough, we add a pose to the path

    // add to path
    path->add_pose(lc_pose);
    ros_path.poses.push_back(type_transform::lc_pose_to_pose_stamped(lc_pose, msg->header.stamp));

    path_pub.publish(ros_path);

    // after every new pose, we search for loops
    // beginning at first pose
    // with max distance of one meter
    // with at least 10 meters traveled
    auto res = path->find_loop_greedy(last_path_idx, MAX_DIST_LC, MIN_TRAVELED_LC);

    // if no loop found, ignore this iteration
    if (res.first == -1 && res.second == -1)
    {
        return;
    }

    last_path_idx = res.second;
    cur_length = 0;

    // else: loop found, display it.
    auto marker = ROSViewhelper::init_loop_detected_marker(path->at(res.first)->pos, path->at(res.second)->pos);

    std::cout << "Loop found" << std::endl;

    loop_pub.publish(marker);
}

/**
 * @brief initializes the local and global map
 * @todo don't harcode this
 *
 */
void init_maps()
{
    global_map_ptr = std::make_shared<GlobalMap>(options->get_map_file_name());

    // extract attribute data from global map
    auto attribute_data = global_map_ptr->get_attribute_data();

    // init local map using attribute data
    local_map_ptr = std::make_shared<LocalMap>(attribute_data.get_map_size_x(),
                                               attribute_data.get_map_size_y(),
                                               attribute_data.get_map_size_z(),
                                               global_map_ptr, true); // still hardcoded af

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

    // pose_sub = n.subscribe("/base_footprint_pose_ground_truth", 1, pose_callback);

    path_pub = n.advertise<nav_msgs::Path>("/test_path", 1);
    path_marker_pub = n.advertise<visualization_msgs::Marker>("/path", 1);
    loop_pub = n.advertise<visualization_msgs::Marker>("/loop", 1);

    // extract poses from global map and add to path
    auto poses = global_map_ptr->get_path();

    for (auto pose : poses)
    {
        path->add_pose(pose);
    }

    // crate path marker 
    auto path_marker = ROSViewhelper::initPathMarker(path);

    int start_idx = 0;
    int end_idx = path->get_length() - 1;
    bool is_ok = true;
    std::vector<visualization_msgs::Marker> loop_visualizations;

    while (is_ok)
    {
        // find path, including visibility check
        auto res = path->find_loop_greedy(start_idx, 3.0f, 10.0f, true);

        // found
        if (res.first != -1 && res.second != -1)
        {
            // update start index for possible second closure
            start_idx = res.second;

            std::cout << "Found a closed loop! Between index " << res.first << " and index " << res.second << std::endl;

            loop_visualizations.push_back(ROSViewhelper::init_loop_detected_marker(path->at(res.first)->pos, path->at(res.second)->pos));
        }
        else
        {
            is_ok = false;
        }
    }

    std::cout << "Found " << loop_visualizations.size() << " loop(s)" << std::endl;

    // ros loop
    while (ros::ok())
    {
        // publish path and loop detects
        path_marker_pub.publish(path_marker);

        for (auto marker : loop_visualizations)
        {
            loop_pub.publish(marker);
        }

        // more ros related stuff
        ros::spinOnce();

        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
