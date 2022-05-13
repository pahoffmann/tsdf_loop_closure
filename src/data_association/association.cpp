#include "association.h"

/**
 * @file association.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief Associates a pose with the corresponding data, needs some form of caching
 *        For caching we need to check multiple different strategies concerning speed
 *        Candidates: HDF5, JSON, SQL-Database (might make sense for models other than greedy)
 * @version 0.1
 * @date 2022-03-14
 *
 * @copyright Copyright (c) 2022
 *
 */

Association::Association(Pose start_pose, int num_pose, std::string base_path, SerializationStrategy ser_strat) : pose(start_pose)
{
    strat = ser_strat;

    std::string type = "";

    switch (ser_strat)
    {
    case SerializationStrategy::HDF5:
        type = ".h5";
        break;

    default:
        type = ".json";
        break;
    }

    // check if the base path contains a trailing slash, if not: add it
    if (base_path.at(base_path.length() - 1) != '/')
    {
        base_path += "/";
    }
    
    file_path = base_path + std::to_string(num_pose) + "_association" + type;
}

void Association::serialize()
{
    switch (strat)
    {
    case SerializationStrategy::SQL:
        serialize_HDF5();
        return;

    case SerializationStrategy::HDF5:
        serialize_SQL();
        return;

    default: // JSON
        serialize_JSON();
        return;
    }

    // now delete data from array ( free data )
    associations.clear();
}

void Association::deserialze()
{
    switch (strat)
    {
    case SerializationStrategy::SQL:
        deserialize_HDF5();
        return;

    case SerializationStrategy::HDF5:
        deserialize_SQL();
        return;

    default: // JSON
        deserialize_JSON();
        return;
    }
}

Association::~Association()
{
    //TODO: eval, whether this destructor might be used to actually remove the file on the harddrive, should be called from association manager
}

/**
 * Below: different serialization / deserialization methods
 */

void Association::serialize_JSON()
{
    Json::Value root;

    for(int i = 0; i < numAssociations(); i++)
    {
        auto& first = associations[i].first;
        auto& second = associations[i].second;

        std::string index = std::to_string(i); 
        root[index] = Json::arrayValue;

        Json::Value association_val;
        association_val["x"] = first.x();
        association_val["y"] = first.y();
        association_val["z"] = first.z();

        association_val["value"] = second.value();
        association_val["weight"] = second.weight();
        association_val["raw"] = second.raw();

        root[index].append(association_val);
    }
    
    // write json data to output file
    std::ofstream file;

    file.open(file_path);
    file << root;
}

void Association::deserialize_JSON()
{
}

void Association::serialize_HDF5()
{
}

void Association::deserialize_HDF5()
{
}

void Association::serialize_SQL()
{
}

void Association::deserialize_SQL()
{
}
