/**
 * @file eigen_vs_ros.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief File containing conversions between ros and eigen
 * @version 0.1
 * @date 2022-07-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include "point.h"
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace type_transform
{
  /**
   * @brief converts eigen pos to ros point
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

  /**
   * @brief method, which converts a eigen vector to a ros point stamped. if no stamp is passed, it defaults to the current time
   *
   * @param vec
   * @param stamp
   * @return geometry_msgs::PointStamped
   */
  geometry_msgs::PointStamped eigen_vec_to_pose_stamped(Eigen::Vector3f vec, ros::Time stamp = ros::Time::now(), std::string frame_id = "map")
  {
    geometry_msgs::PointStamped point;

    point.header.stamp = stamp;
    point.header.frame_id = frame_id;
    point.point.x = vec.x();
    point.point.y = vec.y();
    point.point.z = vec.z();

    return point;
  }

  /**
   * @brief transforms a loop closure pose to a ros pose stamped for visalization
   *
   * @param lc_pose
   * @param stamp
   * @return geometry_msgs::PoseStamped
   */
  geometry_msgs::PoseStamped lc_pose_to_pose_stamped(Pose lc_pose, ros::Time stamp = ros::Time::now(), std::string frame_id = "map")
  {
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = stamp;
    pose.header.frame_id = frame_id;
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
   * @brief converts a ros odometry message to a eigen isometry message
   *
   * @param msg
   * @return Eigen::Isometry3d
   */
  Eigen::Isometry3d ros_odometry_to_eigen_iso(nav_msgs::Odometry &msg)
  {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.rotate(Eigen::Quaterniond(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z));
    iso.translate(Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));

    return iso;
  }
}
