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

// c++ standard related includes
#include <sstream>
#include <iostream>

// external bibs
#include <highfive/H5File.hpp>

// relative includes
#include <loop_closure/map/global_map.h>
#include <loop_closure/map/local_map.h>


void path_callback(const nav_msgs::Path::ConstPtr &msg) {
    ROS_INFO("Retrieved a path!");

    for(auto pose : msg->poses) {
        // do something as writing it to a map file, or similarly, saving it to a json.
        // hdf5 might be more useful though. easier to merge later on
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
  ros::init(argc, argv, "loop_closure_node");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

// specify ros loop rate
  ros::Rate loop_rate(10);

  ros::Subscriber path_sub = n.subscribe("/pose", 1, path_callback);

  // ros loop
  while (ros::ok())
  {
    // more ros related stuff
    ros::spinOnce();

    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

