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
#include "../ray_tracer/ray_tracer.h"

class Path
{
private:
    std::vector<Pose> poses;

    // index of the start pose of the current closed loop
    size_t lc_start_idx = -1;

    // index of the end pose of the current closed loop
    size_t lc_end_idx = -1;

    // ray tracer used for loop closure detection (visibility)
    RayTracer *ray_tracer;

public:
    Path(RayTracer *tracer);

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
     * @brief function which adds a pose to the path
     * 
     * @param pose 
     */
    inline void add_pose(Pose pose) {
        poses.push_back(pose);
    }

    /**
     * @brief Get the length of the path
     * 
     * @return int 
     */
    inline int get_length() {
        return poses.size();
    }

    /**
     * @brief The real magic: looks for a closed loop, greedy: uses distance only (and a visibility criteria)
     *        CAUTION: CURRENTLY, THIS PICKS THE FIRST FOUND LOOP, NO MATTER IF THERE ARE ONES MORE SUITABLE, THIS SHOULD BE ADDRESSED.
     * 
     * @param start_idx 
     * @param max_dist max distance between poses to have a valid closed loop
     * @param min_traveled min distance traveled from start_idx to the pose belonging to a possible closed loop
     * 
     * @return std::pair<int, int> start and end index of the closed loop, will return (-1, -1) if no loop found
     */
    std::pair<int, int> find_loop_greedy(int start_idx, float max_dist, float min_traveled, bool check_visibility = false);

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

    /**
     * @brief method, which blurs parts of the path, used for testing purposes
     * 
     * @param start_idx 
     * @param end_idx 
     */
    void blur(int start_idx, int end_idx);
};
