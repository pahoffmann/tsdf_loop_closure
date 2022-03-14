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

AssociationManager::AssociationManager(Path *path, std::string file_path) : base_path(file_path)
{
    auto poses = path->getPoses();

    for(int i = 0; i < poses.size(); i++) {
        // create new association and add it to the array
        Association association(poses[i], i, file_path, Association::SerializationStrategy::JSON);
        associations.push_back(association);
    }
}

AssociationManager::~AssociationManager()
{
    
}