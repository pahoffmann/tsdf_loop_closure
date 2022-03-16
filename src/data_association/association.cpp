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
    case SerializationStrategy::SQL:
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

    std::cout << "Created new Association, using file path: " << file_path << std::endl;
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
}

/**
 * Below: different serialization / deserialization methods
 */

void Association::serialize_JSON()
{
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
