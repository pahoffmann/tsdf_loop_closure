#pragma once

/**
 * @file math_test.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief
 * @version 0.1
 * @date 2022-08-24
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
#include <loop_closure/path/path.h>

namespace Testing
{
    /**
     * @brief Method testing the math behind the transformation of a distinct cell (here: a distinct point), given a new and an old path
     *        This method will
     *
     * @param cell_before
     * @param before
     * @param after
     * @return std::pair<visualization_msgs::Marker, visualization_msgs::Marker>
     */
    static std::pair<visualization_msgs::Marker, visualization_msgs::Marker> test_cell_transformation(Vector3i cell_before, Path *before, Path *after, int start_idx = -1, int end_idx = -1)
    {

        return std::make_pair(visualization_msgs::Marker(), visualization_msgs::Marker());
    }
}