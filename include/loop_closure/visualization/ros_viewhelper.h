#pragma once

/**
 * @file ros_viewhelper.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
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
#include <loop_closure/util/point.h>
#include <loop_closure/util/constants.h>
#include <loop_closure/util/colors.h>
#include <loop_closure/util/eigen_vs_ros.h>
#include <omp.h>

namespace ROSViewhelper
{

    static int TSDF_LIFETIME = 1000;

    /**
     * @brief visualizes a bounding box for a given pose and sidelengths
     *
     * @param side_length_xy
     * @param side_length_z
     * @param box_pose
     * @return visualization_msgs::Marker
     */
    static visualization_msgs::Marker getBoundingBoxMarker(Eigen::Vector3f box, Pose *box_pose)
    {
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
        cube.scale.x = box.x();
        cube.scale.y = box.y();
        cube.scale.z = box.z();
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
    static visualization_msgs::Marker initPoseMarker(Pose *pose)
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

    static visualization_msgs::Marker initPathMarker(Path *path)
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
            path_marker.points.push_back(type_transform::eigen_point_to_ros_point(pose.pos));
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
    static visualization_msgs::Marker initTSDFmarkerPose(std::shared_ptr<LocalMap> local_map_ptr_, Pose *pose)
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
        Vector3i tmp_pos = real_to_map(pose->pos);

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
     * @return visualization_msgs::Marker
     */
    static visualization_msgs::Marker initTSDFmarkerPath(std::shared_ptr<LocalMap> local_map_ptr_, Path *path, bool consider_none_associates = true)
    {
        // create marker.
        visualization_msgs::Marker tsdf_markers;
        tsdf_markers.header.frame_id = "map";
        tsdf_markers.header.stamp = ros::Time();
        tsdf_markers.ns = "tsdf";
        tsdf_markers.id = 0;
        tsdf_markers.lifetime.fromSec(TSDF_LIFETIME);
        tsdf_markers.type = visualization_msgs::Marker::POINTS;
        tsdf_markers.action = visualization_msgs::Marker::ADD;
        tsdf_markers.scale.x = tsdf_markers.scale.y = MAP_RESOLUTION * 1.0 * 0.001; // 1.0 is the relative size of the marker
        std::vector<geometry_msgs::Point> points;                                   // 3 x 3 markers

        // hashmap to avoid adding duplicate points
        // the hash of the map is a combination of the int vector components, using boosts hash_combine function

        // The following maps were also tested and considered too slow
        // std::map<std::string, TSDFEntry::IntersectStatus> map;
        // boost::unordered_map<std::string, TSDFEntry::IntersectStatus> map;
        // std::unordered_map<std::string, TSDFEntry::IntersectStatus> map;
        boost::unordered_map<size_t, TSDFEntry::IntersectStatus> map;

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
            Vector3i tmp_pos = real_to_map(path->at(i)->pos);

            // shift local map to new path pos
            local_map->shift(tmp_pos);

            // get values, ignore offset
//#pragma omp parallel for
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

                            size_t seed = hash_from_vec(Vector3i(x, y, z));
                            bool is_duplicate = !(map.find(seed) == map.end());

                            if (is_duplicate)
                            {
                                // if it is a duplicate we can skip everything else
                                num_duplicates++;
                                continue;
                            }
                            else
                            {
                                // insert into hashmap and proceed
                                map[seed] = intersect;
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
                            else if (!consider_none_associates)
                            {
                                // when not considering associates, we skip adding them to the marker
                                continue;
                            }
                            else
                            {
                                if (value < 0)
                                {
                                    color = redTSDFColor;
                                }
                                else
                                {
                                    color = greenTSDFColor;
                                }
                            }

                            // map[seed] = intersect;
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

    /**
     * @brief Used to debug code and check, if the association information read from the hdf5 overlaps with the information obtained
     *        using bresenham
     *
     *
     * @return visualization_msgs::Marker
     */
    static visualization_msgs::Marker init_TSDF_marker_from_hashmap(boost::unordered_map<size_t, std::tuple<Vector3i, Vector3i, TSDFEntry, int>> level_one_data)
    {
        // create marker.
        visualization_msgs::Marker tsdf_markers;
        tsdf_markers.header.frame_id = "map";
        tsdf_markers.header.stamp = ros::Time();
        tsdf_markers.ns = "tsdf";
        tsdf_markers.id = 0;
        tsdf_markers.lifetime.fromSec(TSDF_LIFETIME);
        tsdf_markers.type = visualization_msgs::Marker::POINTS;
        tsdf_markers.action = visualization_msgs::Marker::ADD;
        tsdf_markers.scale.x = tsdf_markers.scale.y = MAP_RESOLUTION * 1.0 * 0.001; // 1.0 is the relative size of the marker

        // check for duplicates when creating the map marker.
        int num_duplicates = 0;

        // define colors for the tsdf and intersections
        // intersection color might need to vary
        auto intersectColor = Colors::color_from_name(Colors::ColorNames::fuchsia);
        auto intersectNegColor = Colors::color_from_name(Colors::ColorNames::yellow);
        auto intersectZeroColor = Colors::color_from_name(Colors::ColorNames::aqua);
        auto redTSDFColor = Colors::color_from_name(Colors::ColorNames::maroon);
        auto greenTSDFColor = Colors::color_from_name(Colors::ColorNames::green);

        for (auto pair : level_one_data)
        {
            auto point_map = std::get<0>(pair.second);
            auto point_real = map_to_real(point_map);
            auto tsdf = std::get<2>(pair.second);

            geometry_msgs::Point point;
            point.x = point_real.x();
            point.y = point_real.y();
            point.z = point_real.z();

            std_msgs::ColorRGBA color;

            // assign colors
            if (tsdf.value() < 0)
            {
                color = redTSDFColor;
            }
            else
            {
                color = greenTSDFColor;
            }

            // map[seed] = intersect;
            tsdf_markers.points.push_back(point);
            tsdf_markers.colors.push_back(color);
        }

        return tsdf_markers;
    }

    /**
     * @brief function may be used to display the intersection data for one pose and one pose only, returning a marker
     *        which may display tsdf values inside a local map sized container around the pose
     *
     * @param local_map_ptr_
     * @param pose
     * @return visualization_msgs::Marker
     */
    static visualization_msgs::Marker initSinglePoseMarker(std::shared_ptr<LocalMap> local_map_ptr_, Pose *pose)
    {
        // create marker.
        visualization_msgs::Marker tsdf_markers;
        tsdf_markers.header.frame_id = "map";
        tsdf_markers.header.stamp = ros::Time();
        tsdf_markers.ns = "tsdf";
        tsdf_markers.id = 0;
        tsdf_markers.lifetime.fromSec(TSDF_LIFETIME);
        tsdf_markers.type = visualization_msgs::Marker::POINTS;
        tsdf_markers.action = visualization_msgs::Marker::ADD;
        tsdf_markers.scale.x = tsdf_markers.scale.y = MAP_RESOLUTION * 1.0 * 0.001; // 1.0 is the relative size of the marker
        std::vector<geometry_msgs::Point> points;                                   // 3 x 3 markers

        // define colors for the tsdf and intersections
        // intersection color might need to vary
        auto intersectColor = Colors::color_from_name(Colors::ColorNames::fuchsia);
        auto intersectNegColor = Colors::color_from_name(Colors::ColorNames::yellow);
        auto intersectZeroColor = Colors::color_from_name(Colors::ColorNames::aqua);
        auto redTSDFColor = Colors::color_from_name(Colors::ColorNames::maroon);
        auto greenTSDFColor = Colors::color_from_name(Colors::ColorNames::green);

        local_map_ptr_->shift(real_to_map(pose->pos));

        // get values, ignore offset
        for (int x = pose->pos.x() + (-1 * (local_map_ptr_->get_size().x() - 1) / 2); x < pose->pos.x() + ((local_map_ptr_->get_size().x() - 1) / 2); x++)
        {
            for (int y = pose->pos.y() + (-1 * (local_map_ptr_->get_size().y() - 1) / 2); y < pose->pos.y() + ((local_map_ptr_->get_size().y() - 1) / 2); y++)
            {
                for (int z = pose->pos.z() + (-1 * (local_map_ptr_->get_size().z() - 1) / 2); z < pose->pos.z() + ((local_map_ptr_->get_size().z() - 1) / 2); z++)
                {
                    auto tsdf = local_map_ptr_->value(x, y, z);
                    auto value = tsdf.value();
                    auto weight = tsdf.weight();
                    auto intersect = tsdf.getIntersect();

                    // just the cells, which actually have a weight and a value other than (and including) the default one
                    if (weight > 0 && value < 600)
                    {
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

        return tsdf_markers;
    }

    static visualization_msgs::Marker initPoseAssociationVisualization(std::shared_ptr<GlobalMap> global_map_ptr, Pose *pose, int pose_number = 0)
    {
        // create marker.
        visualization_msgs::Marker tsdf_markers;
        tsdf_markers.header.frame_id = "map";
        tsdf_markers.header.stamp = ros::Time();
        tsdf_markers.ns = "tsdf_one";
        tsdf_markers.id = 0;
        tsdf_markers.lifetime.fromSec(TSDF_LIFETIME);
        tsdf_markers.type = visualization_msgs::Marker::POINTS;
        tsdf_markers.action = visualization_msgs::Marker::ADD;
        tsdf_markers.scale.x = tsdf_markers.scale.y = MAP_RESOLUTION * 1.0 * 0.001; // 1.0 is the relative size of the marker

        // point and color array for the marker
        std::vector<geometry_msgs::Point> points; // 3 x 3 markers
        std::vector<std_msgs::ColorRGBA> colors;

        // define colors for the tsdf and intersections
        // intersection color might need to vary
        auto intersectColor = Colors::color_from_name(Colors::ColorNames::fuchsia);
        auto intersectNegColor = Colors::color_from_name(Colors::ColorNames::yellow);

        // obtain the associations of the pose from the map
        auto associations = global_map_ptr->read_association_data(pose_number);

        // add them to the marker and colorize
        for (int i = 0; i < associations.size(); i += 5)
        {
            Vector3i point(associations[i], associations[i + 1], associations[i + 2]);
            tsdf_markers.points.push_back(type_transform::eigen_point_to_ros_point(map_to_real(point)));

            // colorize
            if (associations[i + 3] < 0)
            {
                tsdf_markers.colors.push_back(intersectNegColor);
            }
            else
            {
                tsdf_markers.colors.push_back(intersectColor);
            }
        }

        return tsdf_markers;
    }

    static visualization_msgs::Marker initPathExtractionVisualizion(std::shared_ptr<GlobalMap> global_map_ptr_, std::shared_ptr<LocalMap> local_map_ptr_)
    {
        auto chunks = global_map_ptr_->all_chunk_poses();
        std::cout << "Number of chunks: " << chunks.size() << std::endl;

        auto chunks_1 = global_map_ptr_->all_chunk_poses(local_map_ptr_->get_size());
        std::cout << "Number filtered chunks: " << chunks_1.size() << std::endl;

        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = ros::Time();
        path_marker.ns = "chunks";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::Marker::CUBE_LIST;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.scale.x = (CHUNK_SIZE * (MAP_RESOLUTION / 1000.0f));
        path_marker.scale.y = (CHUNK_SIZE * (MAP_RESOLUTION / 1000.0f));
        path_marker.scale.z = (CHUNK_SIZE * (MAP_RESOLUTION / 1000.0f));
        path_marker.color.a = 0.5; // Don't forget to set the alpha!
        path_marker.color.r = 255 / 255.0f;
        path_marker.color.g = 15 / 255.0f;
        path_marker.color.b = 15 / 255.0f;

        std_msgs::ColorRGBA color_non_filtered = Colors::color_from_name(Colors::ColorNames::aqua);
        color_non_filtered.a = 0.5;

        for (auto chunk : chunks)
        {
            // for display: every chunk starts at the corner (e.g. 0,0,0 as chunk pose for chunk 0,0,0 -> 64,64,64)
            // but: ros uses the center of a cube for the pose, thus half of the cube size needs to be added.

            Vector3f result = chunk.cast<float>() * (CHUNK_SIZE * (MAP_RESOLUTION / 1000.0f)) + Vector3f(1, 1, 1) * ((CHUNK_SIZE / 2) * (MAP_RESOLUTION / 1000.0f));
            path_marker.points.push_back(type_transform::eigen_point_to_ros_point(result));
            path_marker.colors.push_back(color_non_filtered);
        }

        for (auto chunk : chunks_1)
        {
            Vector3f result = chunk.cast<float>() * (CHUNK_SIZE * (MAP_RESOLUTION / 1000.0f)) + Vector3f(1, 1, 1) * ((CHUNK_SIZE / 2) * (MAP_RESOLUTION / 1000.0f));
            path_marker.points.push_back(type_transform::eigen_point_to_ros_point(result));
            path_marker.colors.push_back(Colors::color_from_name(Colors::ColorNames::red));
        }

        return path_marker;
    }

    /**
     * @brief Constructs a ros marker which displays a line between the two positions passed to the function
     *
     * @param first
     * @param second
     * @return visualization_msgs::Marker
     */
    static visualization_msgs::Marker init_loop_detected_marker(Eigen::Vector3f first, Eigen::Vector3f second)
    {

        // todo: check, wether the line marker needs two points or one point and scaljgnk
        visualization_msgs::Marker line_marker;
        line_marker.ns = "lc_detect";
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.position.x = 0;
        line_marker.pose.position.y = 0;
        line_marker.pose.position.z = 0;
        line_marker.pose.orientation.x = 0.0;
        line_marker.pose.orientation.y = 0.0;
        line_marker.pose.orientation.z = 0.0;
        line_marker.pose.orientation.w = 1.0;
        line_marker.scale.x = 0.3;
        line_marker.scale.y = 0.3;
        line_marker.scale.z = 0.3;
        line_marker.color.r = 255 / 255.0f;
        line_marker.color.g = 0 / 255.0f;
        line_marker.color.b = 0 / 255.0f;
        line_marker.color.a = 0.6;
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time();
        line_marker.id = 0;
        line_marker.points.push_back(type_transform::eigen_point_to_ros_point(first));
        line_marker.points.push_back(type_transform::eigen_point_to_ros_point(second));

        return line_marker;
    }

    static visualization_msgs::Marker marker_from_map_points(std::vector<Eigen::Vector3i> points)
    {
        // raytrace pose
        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = "map";
        points_marker.header.stamp = ros::Time();
        points_marker.ns = "wall";
        points_marker.id = 0;
        points_marker.type = visualization_msgs::Marker::CUBE_LIST;
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.scale.x = points_marker.scale.y = points_marker.scale.z = MAP_RESOLUTION * 1.0 * 0.001; // 1.0 is the relative size of the marker
        points_marker.color.a = 0.5;                                                                          // Don't forget to set the alpha!
        points_marker.color.r = 136 / 255.0f;
        points_marker.color.g = 0 / 255.0f;
        points_marker.color.b = 255 / 255.0f;

        for (auto point : points)
        {
            auto real_point = map_to_real(point);

            points_marker.points.push_back(type_transform::eigen_point_to_ros_point(real_point));
        }

        return points_marker;
    }

    /**
     * @brief gets a data array of a map vector and an attached tsdf entry and creates a ros marker from it
     * 
     * @param data 
     * @return visualization_msgs::Marker 
     */
    static visualization_msgs::Marker marker_from_gm_read(std::vector<std::pair<Eigen::Vector3i, TSDFEntry>> data)
    {
        // raytrace pose
        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = "map";
        points_marker.header.stamp = ros::Time();
        points_marker.ns = "gm_read";
        points_marker.id = 0;
        points_marker.lifetime.fromSec(TSDF_LIFETIME);
        points_marker.type = visualization_msgs::Marker::POINTS;
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.scale.x = points_marker.scale.y = points_marker.scale.z = MAP_RESOLUTION * 1.0 * 0.001; // 1.0 is the relative size of the marker

        for (auto data_pt : data)
        {
            auto real_point = map_to_real(data_pt.first);
            auto tsdf = data_pt.second;


            points_marker.points.push_back(type_transform::eigen_point_to_ros_point(real_point));

            if(tsdf.value() < 0)
            {
                points_marker.colors.push_back(Colors::color_from_name(Colors::ColorNames::maroon));
            }
            else
            {
                points_marker.colors.push_back(Colors::color_from_name(Colors::ColorNames::green));
            }

        }

        return points_marker;
    }
}
