#pragma once


/**
 * @file path.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief This file stores the path and should or should not also consider a "loop close start and end position"
 * @version 0.1
 * @date 2022-03-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <vector>
#include "../util/point.h"
#include <string>
#include "../util/read_json.h"

class Path
{
private:
    std::vector<Pose> poses;

    // index of the start pose of the current closed loop
    size_t lc_start_idx = -1;

    // index of the end pose of the current closed loop
    size_t lc_end_idx = -1;

public:
    Path();
    void addPoseFromEuler(float x, float y, float z, float roll, float pitch, float yaw);
    void fromJSON(std::string filename);
    inline std::vector<Pose>& getPoses() {
        return poses;
    }
};
