#pragma once

/**
 * @file pcl_utility.h
 * @author Patrick Hoffmann (pahoffmann@uos.de)
 * @brief 
 * @version 0.1
 * @date 2022-09-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace PCLUtil {
    
    /**
     * @brief filters outliers in a pointcloud. if a point does not have another point with a predefined max dist, it is removed from the cloud
     * 
     * @param max_dist 
     */
    static void filter_pointcloud_outliers(float max_dist = 0.2f)
    {

    }
}