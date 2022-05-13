#pragma once

/**
 * @file association_manager.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief Class used to manage (e.g. load and unload associations)
 * @version 0.1
 * @date 2022-03-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <map>
#include <iostream>
#include <cassert>
#include <ctime>
#include <chrono>
#include <regex>
#include <filesystem>
#include <highfive/H5File.hpp>

#include "association.h"
#include "../path/path.h"
#include "../ray_tracer/ray_tracer.h"

class AssociationManager
{
private:
    std::vector<Association> associations;
    Path *path;
    std::time_t time;
    std::string base_path;
    RayTracer *ray_tracer;
    std::shared_ptr<LocalMap> local_map_ptr;

    /**
     * @brief Create a serialization folder at the requested path using filesystem utils
     *
     * @param path
     */
    void create_serialization_folder(std::string path);

public:
    /**
     * @brief Construct a new Association Manager object from a path. For every pose in the path, an association is created which is supposed
     *        to connect the tsdf data to a pose and make is serializable. creeeeeeepy.
     *
     * @param path
     * @param file_path
     */
    AssociationManager(Path *path, std::string file_path, RayTracer *tracer, std::shared_ptr<LocalMap> local_map_ptr_);

    /**
     * @brief Destroy the Association Manager object
     * @todo TODO: implement this, this should remove all association data to stop polluting the hard drive
     */
    ~AssociationManager();

    /**
     * @brief Creates greedy associations using a ray-tracer, the current local map and the path
     *        Beginning from the last pose, we ray-trace and every tsdf cell which is touched by the ray, gets unrewokeable associated with it
     *        This greedy association is used to create a MVP and might not be the optimal way to tackle this problem, as cells get associated with
     *        poses, even if the artificial ray trace just crosses it at the border of the local map, or with a weird angle.
     *        Though, it is definetly the most easy way to tackle this problem.
     */
    void greedy_associations();

    /**
     * @brief This is just an idea at the moment, given the following: a ray is interupted, once it crosses a plane between the closest X poses
     *        This plane needs to be calculated for each pose "pair", with the average of the poses as location vector.
     *        This makes sure, that every tsdf cell gets only matched to the closest pose, inluding (hopefully) an acceleration in speed.
     *        Might be necessary to span a net of planes to limit each poses ranging, which somehow spans multiple boxes in which every position operates
     *        This seems pretty similiar to a 3D Voronoi Approach, i didnt even see that until now lol. nice.
     *        see: https://stackoverflow.com/questions/9227285/calculating-a-voronoi-diagram-for-planes-in-3d
     */
    void plane_limited_associations();
};
