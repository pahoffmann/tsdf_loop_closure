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
#include <unordered_map>
#include <map>
#include <boost/unordered_map.hpp>
#include "../util/point.h"
#include "../util/constants.h"
#include "../util/colors.h"
#include "../util/eigen_to_ros.h"

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

    visualization_msgs::Marker initPathMarker(Path *path)
    {
        // raytrace pose
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = ros::Time();
        path_marker.ns = "path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::Marker::CUBE_LIST;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.scale.x = 0.2;
        path_marker.scale.y = 0.2;
        path_marker.scale.z = 0.2;
        path_marker.color.a = 0.5; // Don't forget to set the alpha!
        path_marker.color.r = 64 / 255.0f;
        path_marker.color.g = 224 / 255.0f;
        path_marker.color.b = 208 / 255.0f;

        for (Pose pose : path->getPoses())
        {
            path_marker.points.push_back(eigen_point_to_ros_point(pose.pos));
        }

        return path_marker;
    }

    /**
     * @brief Reads a tsdf global map and displays it in
     *
     * @todo use a tsdf-map datatype to read it and check intersection (separate class), marker should only be used as visualization,
     * only visualize chunks, which are in the bounds of the "local map", which can be seen from the raytracing position.
     *
     * @return visualization_msgs::Marker
     */
    visualization_msgs::Marker initTSDFmarkerPose(std::shared_ptr<LocalMap> local_map_ptr_, Pose *pose)
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
        int not_intersected = 0;
        int num_free = 0;

        // define colors for the tsdf and intersections
        // intersection color might need to vary

        auto intersectColor = Colors::color_from_name(Colors::ColorNames::fuchsia);
        auto intersectNegColor = Colors::color_from_name(Colors::ColorNames::yellow);
        auto intersectZeroColor = Colors::color_from_name(Colors::ColorNames::aqua);
        auto redTSDFColor = Colors::color_from_name(Colors::ColorNames::maroon);
        auto greenTSDFColor = Colors::color_from_name(Colors::ColorNames::green);

        ROS_INFO("Color: %f, %f, %f", intersectColor.r, intersectColor.g, intersectColor.b);

        // global cell index
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

                    // if there is an intersect...
                    if (intersect != TSDFEntry::IntersectStatus::NO_INT)
                    {
                        num_intersects++;
                    }

                    // just the cells, which actually have a weight and a value other than (and including) the default one
                    if (weight > 0 && value < 600)
                    {
                        if (intersect != TSDFEntry::IntersectStatus::NO_INT)
                        {
                            not_intersected++;
                        }
                        else
                        {
                            num_free++;
                        }

                        geometry_msgs::Point point;
                        point.x = x * 0.001 * MAP_RESOLUTION;
                        point.y = y * 0.001 * MAP_RESOLUTION;
                        point.z = z * 0.001 * MAP_RESOLUTION;

                        std_msgs::ColorRGBA color;

                        if (intersect == TSDFEntry::IntersectStatus::INT)
                        {
                            color = intersectColor;
                        }
                        else if (intersect == TSDFEntry::IntersectStatus::INT_NEG)
                        {
                            color = intersectNegColor;
                        }
                        else if (intersect == TSDFEntry::IntersectStatus::INT_ZERO)
                        {
                            color = intersectZeroColor;
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
        ROS_INFO("NUM_NOT_INTERSECTED: %d", not_intersected);
        ROS_INFO("NUM_Free: %d", num_free);

        return tsdf_markers;
    }

    /**
     * @brief Reads the full range of the map, which can be possibly seen from the path positions, and stores it into a marker array
     *        Prints information about:
     *          1.) Number of intersected tsdf cells (unique)
     *          2.) Number of cells, which have been missed by the ray tracer
     *
     *  @todo make this work
     *
     * @return visualization_msgs::Marker
     */
    visualization_msgs::Marker initTSDFmarkerPath(std::shared_ptr<LocalMap> local_map_ptr_, Path *path)
    {
        // create marker.
        visualization_msgs::Marker tsdf_markers;
        tsdf_markers.header.frame_id = "map";
        tsdf_markers.header.stamp = ros::Time();
        tsdf_markers.ns = "tsdf";
        tsdf_markers.id = 0;
        tsdf_markers.lifetime.fromSec(100);
        tsdf_markers.type = visualization_msgs::Marker::POINTS;
        tsdf_markers.action = visualization_msgs::Marker::ADD;
        tsdf_markers.scale.x = tsdf_markers.scale.y = MAP_RESOLUTION * 1.0 * 0.001; // 1.0 is the relative size of the marker
        std::vector<geometry_msgs::Point> points;                                   // 3 x 3 markers

        // hashmap to avoid adding duplicate points
        // the string (hash) being the point in the following way:
        // (x)-(y)-(z)
        // std::map<std::string, TSDFEntry::IntersectStatus> map;
        boost::unordered_map<std::string, TSDFEntry::IntersectStatus> map;
        // std::unordered_map<std::string, TSDFEntry::IntersectStatus> map;

        // display the current local map...

        // iterate through the whole localmap [3d]
        auto local_map = local_map_ptr_.get();
        auto &size = local_map->get_size();
        int num_intersects = 0;
        int not_intersected = 0;
        int num_interesting = 0;

        // check for duplicates when creating the map marker.
        int num_duplicates = 0;

        // define colors for the tsdf and intersections
        // intersection color might need to vary
        auto intersectColor = Colors::color_from_name(Colors::ColorNames::fuchsia);
        auto intersectNegColor = Colors::color_from_name(Colors::ColorNames::yellow);
        auto intersectZeroColor = Colors::color_from_name(Colors::ColorNames::aqua);
        auto redTSDFColor = Colors::color_from_name(Colors::ColorNames::maroon);
        auto greenTSDFColor = Colors::color_from_name(Colors::ColorNames::green);

        ros::Duration hashmap_duration;

        for (int i = 0; i < path->get_length(); i++)
        {
            // global cell index
            Vector3i tmp_pos = (path->at(i)->pos * 1000.0f / MAP_RESOLUTION).cast<int>();

            std::cout << "Gettings the tsdf marker for the whole path. Current pos: " << tmp_pos << std::endl;

            // shift local map to new path pos
            local_map->shift(tmp_pos);

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

                        // just the cells, which actually have a weight and a value other than (and including) the default one
                        if (weight > 0 && value < 600)
                        {
                            // calculate the hash for the current pos
                            auto start_time = ros::Time::now();

                            std::string point_hash = "(" + std::to_string(x) + ")-(" + std::to_string(y) + ")-(" + std::to_string(z) + ")";
                            bool is_duplicate = !(map.find(point_hash) == map.end());

                            if (is_duplicate)
                            {
                                // if it is a duplicate we can skip everything else
                                num_duplicates++;
                                continue;
                            }
                            else
                            {
                                // insert into hashmap and proceed
                                map[point_hash] = intersect;
                            }

                            // more time measuring
                            auto end_time = ros::Time::now();

                            // calc duration
                            hashmap_duration += end_time - start_time;

                            num_interesting++;

                            // every cell, which has a considerable value and weight AND is considered not intersected,
                            // is a miss
                            if (intersect == TSDFEntry::IntersectStatus::NO_INT)
                            {
                                not_intersected++;
                            }
                            else
                            {
                                num_intersects++;
                            }

                            geometry_msgs::Point point;
                            point.x = x * 0.001 * MAP_RESOLUTION;
                            point.y = y * 0.001 * MAP_RESOLUTION;
                            point.z = z * 0.001 * MAP_RESOLUTION;

                            std_msgs::ColorRGBA color;

                            if (intersect == TSDFEntry::IntersectStatus::INT)
                            {
                                color = intersectColor;
                            }
                            else if (intersect == TSDFEntry::IntersectStatus::INT_NEG)
                            {
                                color = intersectNegColor;
                            }
                            else if (intersect == TSDFEntry::IntersectStatus::INT_ZERO)
                            {
                                color = intersectZeroColor;
                            }
                            else if (value < 0)
                            {
                                color = redTSDFColor;
                            }
                            else
                            {
                                color = greenTSDFColor;
                            }

                            map[point_hash] = intersect;
                            tsdf_markers.points.push_back(point);
                            tsdf_markers.colors.push_back(color);
                        }
                    }
                }
            }
        }

        ROS_INFO("[Visualization] Time Measurement hashmap access: %.2f ms", hashmap_duration.toNSec() / 1000000.0f); // display time in ms, with two decimal points

        ROS_INFO("Num-Hit: %d", num_intersects);
        ROS_INFO("Num-missed: %d", not_intersected);
        ROS_INFO("Number of duplicates: %d", num_duplicates);

        ROS_INFO("Hit Percentage: %.2f", (num_intersects * 1.0f / num_interesting) * 100.0f);
        return tsdf_markers;
    }
}
