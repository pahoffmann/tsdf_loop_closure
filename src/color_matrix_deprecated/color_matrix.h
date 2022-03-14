/**
 * @file color_matrix.h
 * @author Patrick Hoffmann (you@domain.com)
 * @brief Currently just an idea, use a color matrix for display purposes used to display data associations, possibly dumb idea
 * @version 0.1
 * @date 2022-03-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <std_msgs/ColorRGBA.h>
#include <vector>
#include "../util/point.h"

class ColorMatrix
{
public:

    ColorMatrix();

private:
    std::vector<std_msgs::ColorRGBA> data;
};