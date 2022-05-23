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

    if (ser_strat == SerializationStrategy::HDF5)
    {
        file_path = base_path + "hdf5_association" + type;
    }
    else
    {
        file_path = base_path + std::to_string(num_pose) + "_association" + type;
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
    // TODO: to make this usable, we should try to save the x,y,z value in one datatype. how?

    HighFive::File file_(file_path, HighFive::File::OpenOrCreate);

    if (!file_.exist("/associations"))
    {
        file_.createGroup("/associations");
    }

    HighFive::Group g = file_.getGroup("/associations");

    std::vector<int> data;

    for (auto it : associations)
    {
        Vector3i ass = vec_from_tag(it.first);
        data.push_back(ass.x());
        data.push_back(ass.y());
        data.push_back(ass.z());
        // data.push_back(ass.second.value());
        // data.push_back(ass.second.weight());
    }

    Vector3i pos_discr = real_to_map(pose.pos);
    g.createDataSet(tag_from_vec(pos_discr), data);

    file_.flush();
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
