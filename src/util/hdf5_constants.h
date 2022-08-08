/**
 * @file hdf5_constants.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief 
 * @version 0.1
 * @date 2022-07-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once


namespace hdf5_constants {

    constexpr auto POSE_DATASET_NAME = "pose";
    constexpr auto ASSOCIATION_DATASET_NAME = "associations";

    constexpr auto MAP_GROUP_NAME = "/map";
    constexpr auto POSES_GROUP_NAME = "/poses";
    constexpr auto INTERSECTIONS_GROUP_NAME = "/intersections";
    
    // number of values in a pose attribute
    constexpr int POSE_DATASET_SIZE = 7;
    constexpr int NUM_GM_ATTRIBUTES = 7;
}