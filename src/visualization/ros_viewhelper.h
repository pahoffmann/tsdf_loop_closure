#pragma once

/**
 * @file ros_viewhelper.h
 * @author Patrick Hoffmann pahoffmann@uni-osnabrueck.de)
 * @brief Used to create ROS visualization Markers, which can later be displayed for evaluation
 * @version 0.1
 * @date 2022-04-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "../util/point.h"
#include "../util/constants.h"
#include "../util/colors.h"

namespace ROSViewhelper
{
    /**
     * @brief visualizes a bounding box for a given pose and sidelengths
     *
     * @param side_length_xy
     * @param side_length_z
     * @param box_pose
     * @return visualization_msgs::Marker
     */
    visualization_msgs::Marker getBoundingBoxMarker(float side_length_xy, float side_length_z, Pose *box_pose)
    {
        ROS_INFO("[BOUNDING-BOX]: side_length: %f, %f, %f", side_length_xy, side_length_xy, side_length_z);
        ROS_INFO("[BOUNDING-BOX]: starting_pose: %f, %f, %f", box_pose->pos.x(), box_pose->pos.y(), box_pose->pos.z());
        visualization_msgs::Marker cube;
        cube.header.frame_id = "map";
        cube.header.stamp = ros::Time();
        cube.ns = "bounding_box";
        cube.id = 0;
        cube.type = visualization_msgs::Marker::CUBE;
        cube.action = visualization_msgs::Marker::ADD;
        cube.pose.position.x = box_pose->pos.x();
        cube.pose.position.y = box_pose->pos.y();
        cube.pose.position.z = box_pose->pos.z();
        cube.pose.orientation.x = 0.0;
        cube.pose.orientation.y = 0.0;
        cube.pose.orientation.z = 0.0;
        cube.pose.orientation.w = 1.0;
        cube.scale.x = side_length_xy;
        cube.scale.y = side_length_xy;
        cube.scale.z = side_length_z;
        cube.color.a = 0.2; // Don't forget to set the alpha!
        cube.color.r = 1.0;
        cube.color.g = 1.0;
        cube.color.b = 0.0;
        cube.lifetime.fromSec(30);

        return cube;
    }

    /**
     * @brief Method, which generates the pose marker for the current ray trace position
     *
     * @return visualization_msgs::Marker (Sphere)
     */
    visualization_msgs::Marker initPoseMarker(Pose *pose)
    {
        // raytrace pose
        visualization_msgs::Marker raytrace_starting_pose_marker;
        raytrace_starting_pose_marker.header.frame_id = "map";
        raytrace_starting_pose_marker.header.stamp = ros::Time();
        raytrace_starting_pose_marker.ns = "ray_pose";
        raytrace_starting_pose_marker.id = 0;
        raytrace_starting_pose_marker.type = visualization_msgs::Marker::SPHERE;
        raytrace_starting_pose_marker.action = visualization_msgs::Marker::ADD;
        raytrace_starting_pose_marker.pose.position.x = pose->pos.x();
        raytrace_starting_pose_marker.pose.position.y = pose->pos.y();
        raytrace_starting_pose_marker.pose.position.z = pose->pos.z();
        raytrace_starting_pose_marker.pose.orientation.x = 0.0;
        raytrace_starting_pose_marker.pose.orientation.y = 0.0;
        raytrace_starting_pose_marker.pose.orientation.z = 0.0;
        raytrace_starting_pose_marker.pose.orientation.w = 1.0;
        raytrace_starting_pose_marker.scale.x = 0.4;
        raytrace_starting_pose_marker.scale.y = 0.4;
        raytrace_starting_pose_marker.scale.z = 0.4;
        raytrace_starting_pose_marker.color.a = 0.5; // Don't forget to set the alpha!
        raytrace_starting_pose_marker.color.r = 1.0;
        raytrace_starting_pose_marker.color.g = 0.0;
        raytrace_starting_pose_marker.color.b = 0.0;

        return raytrace_starting_pose_marker;
    }

    /**
     * @brief Reads a tsdf global map and displays it in
     *
     * @todo use a tsdf-map datatype to read it and check intersection (separate class), marker should only be used as visualization,
     * only visualize chunks, which are in the bounds of the "local map", which can be seen from the raytracing position.
     *
     * @return visualization_msgs::Marker
     */
    visualization_msgs::Marker initTSDFmarker(std::shared_ptr<LocalMap> local_map_ptr_, Pose *pose)
    {
        // create marker.
        visualization_msgs::Marker tsdf_markers;
        tsdf_markers.header.frame_id = "map";
        tsdf_markers.header.stamp = ros::Time();
        tsdf_markers.ns = "tsdf";
        tsdf_markers.id = 0;
        tsdf_markers.type = visualization_msgs::Marker::POINTS;
        tsdf_markers.action = visualization_msgs::Marker::ADD;
        tsdf_markers.scale.x = tsdf_markers.scale.y = MAP_RESOLUTION * 1.0 * 0.001; // why this? @julian -> 0.6 as parameter?
        std::vector<geometry_msgs::Point> points;                                   // 3 x 3 markers

        // display the current local map...

        // iterate through the whole localmap [3d]
        auto local_map = local_map_ptr_.get();
        auto &size = local_map->get_size();
        int num_intersects = 0;

        // define colors for the tsdf and intersections
        // intersection color might need to vary

        auto intersectColor = Colors::color_from_name(Colors::ColorNames::fuchsia);
        auto redTSDFColor = Colors::color_from_name(Colors::ColorNames::maroon);
        auto greenTSDFColor = Colors::color_from_name(Colors::ColorNames::green);

        ROS_INFO("Color: %f, %f, %f", intersectColor.r, intersectColor.g, intersectColor.b);

        Vector3i tmp_pos = (pose->pos * 1000.0f / MAP_RESOLUTION).cast<int>();

        // get values, ignore offset
        for (int x = tmp_pos.x() + (-1 * (size.x() - 1) / 2); x < tmp_pos.x() + ((size.x() - 1) / 2); x++)
        {
            for (int y = tmp_pos.y() + (-1 * (size.y() - 1) / 2); y < tmp_pos.y() + ((size.y() - 1) / 2); y++)
            {
                for (int z = tmp_pos.z() + (-1 * (size.z() - 1) / 2); z < tmp_pos.z() + ((size.z() - 1) / 2); z++)
                {
                    auto tsdf = local_map->value(x, y, z);
                    auto value = tsdf.value();
                    auto weight = tsdf.weight();
                    auto intersect = tsdf.getIntersect();
                    if (intersect)
                    {
                        num_intersects++;
                    }

                    // just the cells, which actually have a weight and a value other than (and including) the default one
                    if (weight > 0 && value < 600)
                    {
                        geometry_msgs::Point point;
                        point.x = x * 0.001 * MAP_RESOLUTION;
                        point.y = y * 0.001 * MAP_RESOLUTION;
                        point.z = z * 0.001 * MAP_RESOLUTION;

                        std_msgs::ColorRGBA color;

                        if (intersect)
                        {
                            color = intersectColor;
                        }
                        else if (value < 0)
                        {
                            color = redTSDFColor;
                        }
                        else
                        {
                            color = greenTSDFColor;
                        }

                        tsdf_markers.points.push_back(point);
                        tsdf_markers.colors.push_back(color);
                    }
                }
            }
        }

        ROS_INFO("NUM_INTERSECTS: %d", num_intersects);

        return tsdf_markers;
    }
}
