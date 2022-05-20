#pragma once

/**
 * @file global_map.h
 * @author Steffen Hinderink
 * @author Juri Vana
 * @author Malte Hillmann
 */

#include <highfive/H5File.hpp>
#include <cmath>
#include <string>
#include <utility>
#include <sstream>
#include "../util/point.h"
#include "../util/tsdf.h"

struct ActiveChunk
{
    std::vector<TSDFEntry::RawType> data;
    // std::vector<TSDFEntry::IntersectStatus> intersects;
    Vector3i pos;
    int age;
};

/**
 * Global map containing containing truncated signed distance function (tsdf) values and weights.
 * The map is divided into chunks.
 * An HDF5 file is used to store the chunks.
 * Additionally poses can be saved.
 */
class GlobalMap
{

private:
    /**
     * HDF5 file in which the chunks are stored.
     * The file structure looks like this:
     *
     * file.h5
     * |
     * |-/map
     * | |
     * | |-0_0_0 \
     * | |-0_0_1  \
     * | |-0_1_0    chunk datasets named after their tag
     * | |-0_1_1  /
     * | |-1_0_0 /
     */
    HighFive::File file_;

    /// Initial default tsdf value.
    TSDFEntry initial_tsdf_value_;

    /**
     * Vector of active chunks.
     */
    std::vector<ActiveChunk> active_chunks_;

    /// Number of poses that are saved in the HDF5 file
    int num_poses_;

    /**
     * Returns the index of a global position in a chunk.
     * The returned index is that of the tsdf value.
     * The index of the weight is one greater.
     * @param pos the position
     * @return index in the chunk
     */
    int index_from_pos(Vector3i pos, const Vector3i &chunkPos);

public:
    /// Side length of the cube-shaped chunks
    static constexpr int CHUNK_SIZE = 64;

    /// Maximum number of active chunks.
    static constexpr int NUM_CHUNKS = 64;

    /**
     * Constructor of the global map.
     * It is initialized without chunks.
     * The chunks are instead created dynamically depending on which are used.
     * @param name name with path and extension (.h5) of the HDF5 file in which the map is stored
     * @param initial_tsdf_value default tsdf value
     * @param initial_weight initial default weight
     */
    GlobalMap(std::string name, TSDFEntry::ValueType initial_tsdf_value, TSDFEntry::WeightType initial_weight);

    /**
     * Returns a value pair consisting of a tsdf value and a weight from the map.
     * @param pos the position
     * @return value pair from the map
     */
    TSDFEntry get_value(const Vector3i &pos);

    /**
     * Sets a value pair consisting of a tsdf value and a weight on the map.
     * @param pos the position
     * @param value value pair that is set
     */
    void set_value(const Vector3i &pos, const TSDFEntry &value);

    /**
     * Activates a chunk and returns it by reference.
     * If the chunk was already active, it is simply returned.
     * Else the HDF5 file is checked for the chunk.
     * If it also doesn't exist there, a new empty chunk is created.
     * Chunks get replaced and written into the HDF5 file by a LRU strategy.
     * The age of the activated chunk is reset and all ages are updated.
     * @param chunk position of the chunk that gets activated
     * @return reference to the activated chunk
     */
    std::vector<TSDFEntry::RawType> &activate_chunk(const Vector3i &chunk);

    /**
     * Writes all active chunks into the HDF5 file.
     */
    void write_back();

    /**
     * @brief basic function, which checks if a specific chunk exists
     *
     * @param chunk_pos
     * @return true
     * @return false
     */
    bool chunk_exists(const Eigen::Vector3i &chunk_pos);

    /**
     * @brief returns all chunk poses of the global map
     * 
     * @param valid_for_path  boolean used to check, if all chunk poses are valid.
     * @return std::vector<Vector3i> 
     */
    std::vector<Vector3i> all_chunk_poses(Vector3i l_map_size = Vector3i(0,0,0));

    int num_chunks();

    /**
     * Given a position in a chunk the tag of the chunk gets returned.
     * @param pos the position
     * @return tag of the chunk
     */
    std::string tag_from_chunk_pos(const Vector3i &pos);

    /**
     * @brief gets the adjacent chunk positions of chunk_pos
     * 
     */
    std::vector<Vector3i> get_adjacent_chunks(Vector3i chunk_pos);

    /**
     * @brief gets the chunk pos from the tag
     * 
     * @param tag 
     * @return Vector3i 
     */
    Vector3i chunk_pos_from_tag(std::string tag) const;

    /**
     * @brief Checks if the area around the delivered chunk_pos, considering the localmap-size is already fully loaded,
     *        meaning every chunk in the area around the position is already present in the global map and does not need to be created
     * 
     * @param chunk_pos 
     * @param localmap_size 
     * @return true 
     * @return false 
     */
    bool is_fully_occupied(Vector3i &chunk_pos, Vector3i &localmap_size);

    /**
     * @brief returns the path from the global map, if present in the hdf5
     * 
     * @return std::vector<Pose> 
     */
    std::vector<Pose> get_path();
    
    /**
     * @brief Writes the delivered path in a /poses group to the global map
     * 
     * @param path 
     */
    void write_path(std::vector<Pose> &path);

    /**
     * @brief gets information about wether a globalmap hdf5 has a path or not
     * 
     * @return true 
     * @return false 
     */
    inline bool has_path() {
        if(file_.exist("/poses") && file_.getGroup("/poses").listObjectNames().size() > 0) {
            return true;
        }

        return false;
    }

    /**
     * @brief cleans up artifacts in the global map, caused by refraction, reflection, windows etc. (basic cleanup)
     *  -> walk through every chunk, load the dat
     * 
     */
    void cleanup_artifacts();
};
