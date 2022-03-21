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
#include "../util/point.h"
#include "../util/tsdf.h"
#include "../map/local_map.h"
#include <jsoncpp/json/json.h>
#include <fstream>

class Association
{

public:

    /**
     * @brief There will be multiple strategys for serializing the associations, which might be needed by different association strategies
     * 
     */
    enum SerializationStrategy {
        SQL,
        HDF5,
        JSON
    };

    Association(Pose start_pose, int num_pose, std::string base_path, SerializationStrategy ser_strat = SerializationStrategy::JSON);
    ~Association();
    // maybe use indexing here insead of actual global pose
    inline void addAssociation(Eigen::Vector3f pose, TSDFEntry entry) {
        associations.push_back(std::make_pair(pose, entry));
    }

    // serialize data if no longer needed
    void serialize();

    // deserialize if data is needed
    void deserialze();

    // returns the number of data associations
    inline size_t numAssociations() {
        return associations.size();
    }

    /**
     * @brief Get the Association object
     * 
     * @param index 
     * @return std::pair<TSDFEntry, Eigen::Vector3f>
     * @throws IndexOutOfBoundsException The requested association 
     */
    inline std::pair<Eigen::Vector3f, TSDFEntry> getAssociation(size_t index) {
        return associations[index];
    }

    /**
     * @brief Get the pose associated with this association
     * 
     * @return Pose 
     */
    inline Pose* getPose() {
        return &pose;
    }


private:
    Pose pose; // pose of the association
    SerializationStrategy strat;
    std::string file_path;
    std::shared_ptr<LocalMap> local_map_ptr;

    // we might need a hashmap depending on the case to speedup the search for association on specific positions.
    // hashmap should uses indices rather than pure global positioning information
    // this datatype is enough vor MVP (minimal viable product) which uses the greedy strat
    std::vector<std::pair<Eigen::Vector3f, TSDFEntry>> associations; // associations between space and Value

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
