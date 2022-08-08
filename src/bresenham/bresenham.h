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

#include "../util/point.h"
#include "../util/colors.h"
#include "../map/local_map.h"
#include "../data_association/association.h"
#include "../options/options_reader.h"
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
