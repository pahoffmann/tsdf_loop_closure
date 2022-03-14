#include "association_manager.h"

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