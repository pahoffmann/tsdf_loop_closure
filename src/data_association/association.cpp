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

Association::Association(Pose start_pose, int num_pose, std::shared_ptr<GlobalMap> global_map_ptr, std::string base_path, SerializationStrategy ser_strat) : pose(start_pose)
{
    strat = ser_strat;
    pose_number = num_pose;

    std::string type = "";

    switch (ser_strat)
    {
    case SerializationStrategy::HDF5:
        // do nothing, as here the global map hdf5 is used
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

    if (ser_strat == SerializationStrategy::JSON)
    {
        file_path = base_path + std::to_string(num_pose) + "_association" + type;
    }
    else 
    {
        // currently nothing, as for hdf file, the association data is written to the global map hdf5
    }
}

void Association::serialize()
{
    switch (strat)
    {
    case SerializationStrategy::SQL:
        serialize_SQL();
        return;

    case SerializationStrategy::HDF5:
        serialize_HDF5();
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
        deserialize_SQL();
        return;

    case SerializationStrategy::HDF5:
        deserialize_HDF5();
        return;

    default: // JSON
        deserialize_JSON();
        return;
    }
}

Association::~Association()
{
    // TODO: eval, whether this destructor might be used to actually remove the file on the harddrive, should be called from association manager
}

/**
 * Below: different serialization / deserialization methods
 */

void Association::serialize_JSON()
{
    Json::Value root;

    int i = 0;
    for (auto it : associations)
    {
        auto first = vec_from_tag(it.first);
        auto second = it.second;

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

        i++;
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
    std::cout << "[Association - serialize_HDF5] Start serialization" << std::endl;
    std::vector<int> data;

    for (auto it : associations)
    {
        Vector3i ass = vec_from_tag(it.first);
        auto tsdf = it.second;
        data.push_back(ass.x());
        data.push_back(ass.y());
        data.push_back(ass.z());
        data.push_back(tsdf.value());
        data.push_back(tsdf.weight());
    }

    // write the data to the hdf5
    global_map_ptr->write_association_data(data, pose_number);
    associations.clear();
}

void Association::deserialize_HDF5()
{
    auto data = global_map_ptr->read_association_data(pose_number);

    // transform data to map
    for(int i = 0; i < data.size(); i += 5) 
    {
        associations[tag_from_vec(Vector3i(data[i], data[i + 1], data[i + 2]))] = TSDFEntry(data[i + 4], data[i + 5]);
    }
}

void Association::serialize_SQL()
{
}

void Association::deserialize_SQL()
{
}
