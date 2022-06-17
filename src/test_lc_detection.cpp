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


void pose_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    ROS_INFO("Retrieved a path!");

    auto pose = msg->pose.pose;
    
    std::cout << "Got odometry message. Pose:" << pose.position.x << " | " << pose.position.y << " | " << pose.position.z << std::endl;
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
  ros::init(argc, argv, "test_lc_detection_node");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

// specify ros loop rate
  ros::Rate loop_rate(10);

  ros::Subscriber path_sub = n.subscribe("/pose", 1, pose_callback);

  // ros loop
  while (ros::ok())
  {
    // more ros related stuff
    ros::spinOnce();

    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

