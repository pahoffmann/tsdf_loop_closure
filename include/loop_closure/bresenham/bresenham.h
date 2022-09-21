#pragma once

/**
 * @file bresenham.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief 
 * @version 0.1
 * @date 2022-07-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <loop_closure/util/point.h>
#include <loop_closure/util/colors.h>
#include <loop_closure/map/local_map.h>
#include <loop_closure/data_association/association.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <loop_closure/LoopClosureConfig.h>
#include <visualization_msgs/Marker.h>

class Bresenham
{
private:
    /* data */
public:
    Bresenham(/* args */);
    ~Bresenham();
};

Bresenham::Bresenham(/* args */)
{
}

Bresenham::~Bresenham()
{
}
