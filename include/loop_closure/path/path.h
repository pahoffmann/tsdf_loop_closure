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
#include <loop_closure/util/point.h>
#include <string>
#include <loop_closure/serialization/read_path_json.h>
#include <loop_closure/ray_tracer/ray_tracer.h>
#include <loop_closure/util/random.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class Path
{
private:
    // vector holding the paths poses
    std::vector<Pose> poses;

    // ray tracer used for loop closure detection (visibility)
    RayTracer *ray_tracer;

public:
    /**
     * @brief Copy constructor of the path
     *
     * @param other
     */
    Path(Path &other);

    /**
     * @brief default constructor
     *
     */
    Path() = default;

    /**
     * @brief attaches a ray tracer to the path, needs to be called before any visibility checks take place
     *
     * @param tracer
     */
    void attach_raytracer(RayTracer *tracer)
    {
        ray_tracer = tracer;
    }

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
    inline std::vector<Pose> &getPoses()
    {
        return poses;
    }

    /**
     * @brief function which adds a pose to the path
     *
     * @param pose
     */
    inline void add_pose(Pose pose)
    {
        poses.push_back(pose);
    }

    /**
     * @brief Get the length of the path
     *
     * @return int
     */
    inline int get_length() const
    {
        return poses.size();
    }

    /**
     * @brief The real magic: looks for a closed loop, greedy: uses distance only (and a visibility criteria)
     *        CAUTION: CURRENTLY, THIS PICKS THE FIRST FOUND LOOP, NO MATTER IF THERE ARE ONES MORE SUITABLE, THIS SHOULD BE ADDRESSED.
     *        SOLUTION: find loop candidates and compare them
     *        CAUTION: there may be more than one loop
     *
     * @param start_idx
     * @param max_dist max distance between poses to have a valid closed loop
     * @param min_traveled min distance traveled from start_idx to the pose belonging to a possible closed loop
     *
     * @return std::pair<int, int> start and end index of the closed loop, will return (-1, -1) if no loop found
     */
    std::pair<int, int> find_loop_greedy(int start_idx, float max_dist, float min_traveled, bool check_visibility = false);

    /**
     * @brief will use the method proposed by liosam to find a lc:
     *        https://github.com/TixiaoShan/LIO-SAM/blob/6665aa0a4fcb5a9bb3af7d3923ae4a035b489d47/src/mapOptmization.cpp#L610
     *
     * @param start_idx
     * @param max_dist
     * @param min_traveled
     * @param check_visibility
     * @return std::pair<int, int>
     */
    std::pair<int, int> find_loop_kd_min_dist(int start_idx, float max_dist, float min_traveled, bool check_visibility);

    /**
     * @brief will use the method proposed by liosam to find a lc:
     *        https://github.com/TixiaoShan/LIO-SAM/blob/6665aa0a4fcb5a9bb3af7d3923ae4a035b489d47/src/mapOptmization.cpp#L610
     * @details will check if there exists a loop closure for the given index with an arbitrary start pose
     *
     * @param start_idx
     * @param max_dist
     * @param min_traveled
     * @param check_visibility
     * @return std::pair<int, int>
     */
    std::pair<int, int> find_loop_kd_min_dist_backwards(int idx, float max_dist, float min_traveled, bool check_visibility);

    /**
     * @brief returns a pose of the path for a given idx
     *
     * @param idx
     * @return Pose* or NULL, if index is out of bounds
     * @throw std::out_of_range  if the passed index is in bounds of the path
     */
    inline Pose *at(int idx)
    {
        if (idx < poses.size() && idx >= 0)
        {
            return &poses[idx];
        }
        else
        {
            std::stringstream ss;
            ss << "[Path] There is no Pose at the requested Path-Index: " << idx << std::endl;
            throw std::out_of_range(ss.str());
        }
    }

    /**
     * @brief method, which blurs parts of the path, used for testing purposes, blurs the current path
     *
     * @param start_idx
     * @param end_idx
     * @param radius
     */
    void blur(int start_idx, int end_idx, double radius);

    /**
     * @brief method, which blurs parts of the path, used for testing purposes, will return a new path
     *
     * @param start_idx
     * @param end_idx
     * @param radius
     */
    Path blur_ret(int start_idx, int end_idx, double radius);

    /**
     * @brief rotate the path around a given vertex, if identity vertex is passed, the path is rotated against its centroid
     *
     * @param roll_deg
     * @param pitch_deg
     * @param yaw_deg
     * @param rotation_pose path may be also rotated around a pose.
     * @return Path
     */
    Path rotate_ret(float roll_deg, float pitch_deg, float yaw_deg, Pose *rotation_pose = NULL);

    /**
     * @brief returns a translated copy of the current path
     *
     * @param tanslation_vec
     * @return Path
     */
    Path translate_ret(Vector3f tanslation_vec, int start_idx, int end_idx);

    /**
     * @brief Get the centroid of the path positions
     *
     * @return Vector3f
     */
    Vector3f get_centroid();

    /**
     * @brief Get the distances between two path poses when following the path poses
     *
     * @param idx1
     * @param idx2
     * @return float
     */
    float get_distance_between_path_poses(int idx1, int idx2);
};
