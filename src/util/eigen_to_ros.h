#pragma once

#include "point.h"
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

/**
 * @brief converts eigen pos to ros point, maybe outsource to header
 * 
 * @param pos 
 * @return geometry_msgs::Point 
 */
geometry_msgs::Point eigen_point_to_ros_point(Eigen::Vector3f pos)
{
  geometry_msgs::Point point;
  point.x = pos.x();
  point.y = pos.y();
  point.z = pos.z();

  return point;
}