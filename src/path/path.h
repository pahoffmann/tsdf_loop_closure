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
#include "../serialization/read_path_json.h"

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

    /**
     * @brief Reads a path from a json
     * 
     * @param filename 
     */
    void fromJSON(std::string filename);

    /**
     * @brief Get the Poses object
     * 
     * @return std::vector<Pose>& 
     */
    inline std::vector<Pose>& getPoses() {
        return poses;
    }

    /**
     * @brief The real magic: looks for a closed loop
     * 
     * @param start_idx 
     * @return int 
     */
    int find_next_loop(int start_idx);

    /**
     * @brief returns a pose of the path for a given idx
     * 
     * @param idx 
     * @return Pose* or NULL, if index is out of bounds
     * @throw std::out_of_range  if the passed index is in bounds of the path
     */
    inline Pose* at(int idx) {
        if(idx < poses.size() && idx >= 0)
        {
            return &poses[idx];
        }
        else {
            throw std::out_of_range("[Path] There is no Pose at the requested Path-Index.");
        }
    }
};
