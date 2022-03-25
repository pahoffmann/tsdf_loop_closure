#include "association_manager.h"

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

AssociationManager::AssociationManager(Path *path, std::string file_path, RayTracer *tracer, std::shared_ptr<LocalMap> local_map_ptr_) : base_path(file_path)
{
    auto poses = path->getPoses();
    ray_tracer = tracer;
    local_map_ptr = local_map_ptr_;

    // create timestamp
    time = std::time(nullptr);
    std::string time_string(std::asctime(std::localtime(&time)));

    std::cout << "[AssociationManager] The current time is: " << time_string << std::endl;

    time_string = std::regex_replace(time_string, std::regex(" "), "_");
    time_string = std::regex_replace(time_string, std::regex(":"), "_");

    std::cout << "[AssociationManager] Current time with replacements: " << time_string << std::endl;

    // check if the file path contains a trailing slash, if not: add it
    if (file_path.at(file_path.length() - 1) != '/')
    {
        file_path += "/";
    }

    // create serialization folder
    create_serialization_folder(file_path);

    for (int i = 0; i < poses.size(); i++)
    {
        // create new association and add it to the array
        Association association(poses[i], i, file_path, Association::SerializationStrategy::JSON);
        associations.push_back(association);
    }
}

AssociationManager::~AssociationManager()
{
}

void AssociationManager::create_serialization_folder(std::string path)
{
    // create a folder inside the given filepath, for the current timestamp
    std::cout << "[AssociationManager] Creating new association directiory: " << path << std::endl;

    std::filesystem::create_directories(path);
}

void AssociationManager::greedy_associations()
{

    for (int i = associations.size() - 1; i >= 0; i--)
    {
        // configure and start the ray tracer for every iteration, which will fill each association
        // ray_tracer->update_map_pointer(local_map_ptr);

        // update ray tracer data for the next trace
        ray_tracer->update_pose(associations[i].getPose());
        ray_tracer->update_association(&associations[i]);
        ray_tracer->start(); // start tracing, given the current association.

        associations[i].serialize(); // serialize data
    }
}

void AssociationManager::plane_limited_associations()
{
}
