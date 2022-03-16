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

void AssociationManager::greedy_associations()
{

    for (int i = associations.size() - 1; i >= 0; i--)
    {
        // configure and start the ray tracer for every iteration, which will fill
        ray_tracer->update_map_pointer(local_map_ptr);
        ray_tracer->update_association(&associations[i]);
        ray_tracer->start(); // start tracing, given the current association.


    }
}

void AssociationManager::plane_limited_associations()
{

}
