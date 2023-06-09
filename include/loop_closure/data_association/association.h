#pragma once

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

#include <vector>
#include <fstream>

// imports for serialization
#include <jsoncpp/json/json.h>
#include <highfive/H5File.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include <loop_closure/util/point.h>
#include <loop_closure/util/tsdf.h>
#include <loop_closure/map/local_map.h>

class Association
{

public:
    /**
     * @brief There will be multiple strategys for serializing the associations, which might be needed by different association strategies
     *
     * @details currently the only viable strategy is to use hdf5, sql is not implemented
     *
     */
    enum SerializationStrategy
    {
        SQL,
        HDF5,
        JSON
    };

    Association(Pose start_pose, int num_pose, std::shared_ptr<GlobalMap> global_map_ptr_, std::string base_path, SerializationStrategy ser_strat = SerializationStrategy::HDF5);
    ~Association();
    // maybe use indexing here insead of actual global pose
    inline void addAssociation(Eigen::Vector3i cell, TSDFEntry entry)
    {
        // auto tag = tag_from_vec(pose);
        size_t seed = hash_from_vec(cell);

        if (associations.find(seed) == associations.end())
        {
            associations[seed] = std::make_pair(cell, entry);
            num_accesses_to_hm_w++;
        }
        else
        {
            num_accesses_to_hm_r++;
        }
    }

    // serialize data if no longer needed
    void serialize();

    // deserialize if data is needed
    void deserialze();

    // returns the number of data associations
    inline size_t numAssociations()
    {
        return associations.size();
    }

    /**
     * @brief a reference to the association of the current pose
     *
     * @return boost::unordered_map<size_t, std::pair<Eigen::Vector3i, TSDFEntry>>&
     */
    inline boost::unordered_map<size_t, std::pair<Eigen::Vector3i, TSDFEntry>> &getAssociations()
    {
        return associations;
    }

    /**
     * @brief Get the pose associated with this association
     *
     * @return Pose
     */
    inline Pose *getPose()
    {
        return &pose;
    }

    inline int get_index()
    {
        return pose_number;
    }

    /**
     * @brief clears the data in the association
     *
     */
    void clear_data()
    {
        associations.clear();
    }

private:
    Pose pose; // pose of the association
    SerializationStrategy strat;
    std::string file_path;
    std::shared_ptr<LocalMap> local_map_ptr;
    std::shared_ptr<GlobalMap> global_map_ptr;

    int num_accesses_to_hm_r = 0;
    int num_accesses_to_hm_w = 0;

    // number of the associated pose (for global map relation mapping)
    int pose_number;

    // we might need a hashmap depending on the case to speedup the search for association on specific positions.
    // hashmap should uses indices rather than pure global positioning information
    // this datatype is enough vor MVP (minimal viable product) which uses the greedy strat
    // boost::unordered_map<std::string, TSDFEntry> associations; // associations between space and Value
    boost::unordered_map<size_t, std::pair<Eigen::Vector3i, TSDFEntry>> associations; // associations between space and Value

    // serializes the association in hdf5
    void serialize_HDF5();

    // serializes the association in sql
    void serialize_SQL();

    // serializes the association in json format
    void serialize_JSON();

    // deserializes the association in hdf5
    void deserialize_HDF5();

    // deserializes the association in sql
    void deserialize_SQL();

    // deserializes the association in json format
    void deserialize_JSON();
};
