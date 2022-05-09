/**
 * @author Steffen Hinderink
 * @author Juri Vana
 * @author Malte Hillmann
 */

#include <sstream>
#include <ros/ros.h>

#include "global_map.h"

GlobalMap::GlobalMap(std::string name, TSDFEntry::ValueType initial_tsdf_value, TSDFEntry::WeightType initial_weight)
    : file_{name, HighFive::File::OpenOrCreate}, // Truncate clears already existing file
      initial_tsdf_value_{initial_tsdf_value, initial_weight},
      active_chunks_{},
      num_poses_{0}
{
    if (!file_.exist("/map"))
    {
        file_.createGroup("/map");
    }
    else
    {
        ROS_INFO("[GLOBAL_MAP] Using existing HDF5 Parameter [/map]");
    }

    if (!file_.exist("/poses"))
    {
        file_.createGroup("/poses");
    }
    else
    {
        ROS_INFO("[GLOBAL_MAP] Using existing HDF5 Parameter [/poses]");
    }
}

std::string GlobalMap::tag_from_chunk_pos(const Vector3i &pos)
{
    std::stringstream ss;
    ss << pos.x() << "_" << pos.y() << "_" << pos.z();
    return ss.str();
}

int GlobalMap::index_from_pos(Vector3i pos, const Vector3i &chunkPos)
{
    pos -= chunkPos * CHUNK_SIZE;
    return (pos.x() * CHUNK_SIZE * CHUNK_SIZE + pos.y() * CHUNK_SIZE + pos.z());
}

std::vector<TSDFEntry::RawType> &GlobalMap::activate_chunk(const Vector3i &chunkPos)
{
    int index = -1;
    int n = active_chunks_.size();

    // instead maybe a hashmap of chunks?
    for (int i = 0; i < n; i++)
    {
        if (active_chunks_[i].pos == chunkPos)
        {
            // chunk is already active
            index = i;
        }
    }

    if (index == -1)
    {
        // chunk is not already active
        ActiveChunk newChunk;
        newChunk.pos = chunkPos;
        newChunk.age = 0;

        HighFive::Group g = file_.getGroup("/map");
        auto tag = tag_from_chunk_pos(chunkPos);

        if (g.exist(tag))
        {
            // read chunk from file
            HighFive::DataSet d = g.getDataSet(tag);
            d.read(newChunk.data);
        }
        else
        {
            // create new chunk
            newChunk.data = std::vector<TSDFEntry::RawType>(CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE, initial_tsdf_value_.raw());
        }
        // put new chunk into active_chunks_
        if (n < NUM_CHUNKS)
        {
            // there is still room for active chunks
            index = n;
            newChunk.age = n; // temporarily assign oldest age so that all other ages get incremented
            active_chunks_.push_back(newChunk);
        }
        else
        {
            // write oldest chunk into file
            int max = -1;
            for (int i = 0; i < n; i++)
            {
                if (active_chunks_[i].age > max)
                {
                    max = active_chunks_[i].age;
                    index = i;
                }
            }
            auto tag = tag_from_chunk_pos(active_chunks_[index].pos);

            if (g.exist(tag))
            {
                auto d = g.getDataSet(tag);
                d.write(active_chunks_[index].data);
            }
            else
            {
                g.createDataSet(tag, active_chunks_[index].data);
            }
            // overwrite with new chunk
            active_chunks_[index] = newChunk;
        }
    }
    // update ages
    int age = active_chunks_[index].age;
    for (auto &chunk : active_chunks_)
    {
        if (chunk.age < age)
        {
            chunk.age++;
        }
    }
    active_chunks_[index].age = 0;
    return active_chunks_[index].data;
}

TSDFEntry GlobalMap::get_value(const Vector3i &pos)
{
    Vector3i chunkPos = floor_divide(pos, CHUNK_SIZE);
    const auto &chunk = activate_chunk(chunkPos);
    int index = index_from_pos(pos, chunkPos);
    return TSDFEntry(chunk[index]);
}

void GlobalMap::set_value(const Vector3i &pos, const TSDFEntry &value)
{
    Vector3i chunkPos = floor_divide(pos, CHUNK_SIZE);
    auto &chunk = activate_chunk(chunkPos);
    int index = index_from_pos(pos, chunkPos);
    chunk[index] = value.raw();
}

void GlobalMap::write_back()
{
    HighFive::Group g = file_.getGroup("/map");
    for (auto &chunk : active_chunks_)
    {
        auto tag = tag_from_chunk_pos(chunk.pos);

        if (g.exist(tag))
        {
            auto d = g.getDataSet(tag);
            d.write(chunk.data);
        }
        else
        {
            g.createDataSet(tag, chunk.data);
        }
    }
    file_.flush();
}

bool GlobalMap::chunk_exists(const Vector3i &chunk_pos)
{
    HighFive::Group g = file_.getGroup("/map");

    auto tag = tag_from_chunk_pos(chunk_pos);

    std::cout << "Checking if chunk exists: " << tag << std::endl;

    if (g.exist(tag))
    {
        return true;
    }
    else
    {
        return false;
    }
}

std::vector<Vector3i> GlobalMap::get_adjacent_chunks(Vector3i chunk_pos)
{
    std::vector<Vector3i> chunks;
    chunks.push_back(chunk_pos + Vector3i(0, 0, 1));
    chunks.push_back(chunk_pos + Vector3i(0, 0, -1));
    chunks.push_back(chunk_pos + Vector3i(0, 1, 0));
    chunks.push_back(chunk_pos + Vector3i(0, -1, 0));
    chunks.push_back(chunk_pos + Vector3i(1, 0, 0));
    chunks.push_back(chunk_pos + Vector3i(-1, 0, 0));

    return chunks;
}

std::vector<Vector3i> GlobalMap::all_chunk_poses(Vector3i l_map_size)
{
    HighFive::Group g = file_.getGroup("/map");
    auto object_names = g.listObjectNames();
    std::vector<Vector3i> poses;

    // local map size (in chunks), already halved.
    Vector3i chunked_lmap_size = ceil_divide(l_map_size, CHUNK_SIZE * 2);

    std::cout << "Localmap size: " << std::endl << l_map_size << std::endl;
    std::cout << "Chunked map size: " << std::endl << chunked_lmap_size << std::endl;

    for (auto name : object_names)
    {
        if (!g.exist(name))
        {
            throw std::logic_error("Error when reading chunks from h5");
        }

        // if the local map size is passed to the function, it checks, whether all chunks in an area as big as the local map
        // around the current chunk are already loaded in the global map
        if (l_map_size != Vector3i(0, 0, 0))
        {
            Vector3i tmp = chunk_pos_from_tag(name);

            // 8 chunks need to be checked here ( 8 corners in 3d)
            std::vector<Vector3i> corners;
            corners.push_back(tmp + chunked_lmap_size);
            corners.push_back(tmp + Vector3i(chunked_lmap_size.x(), chunked_lmap_size.y(), -chunked_lmap_size.z()));
            corners.push_back(tmp + Vector3i(chunked_lmap_size.x(), -chunked_lmap_size.y(), chunked_lmap_size.z()));
            corners.push_back(tmp + Vector3i(chunked_lmap_size.x(), -chunked_lmap_size.y(), -chunked_lmap_size.z()));
            corners.push_back(tmp + Vector3i(-chunked_lmap_size.x(), chunked_lmap_size.y(), chunked_lmap_size.z()));
            corners.push_back(tmp + Vector3i(-chunked_lmap_size.x(), chunked_lmap_size.y(), -chunked_lmap_size.z()));
            corners.push_back(tmp + Vector3i(-chunked_lmap_size.x(), -chunked_lmap_size.y(), chunked_lmap_size.z()));
            corners.push_back(tmp - chunked_lmap_size);

            bool check = true;

            // check for every corner, if it exists.
            for (auto corner : corners)
            {
                if (!chunk_exists(corner))
                {
                    std::cout << "There is a non existing chunk here..." << std::endl;
                    check = false;
                    break;
                }
            }

            // if still every chunk around the current chunk is registered, it is a possible valid path pos
            if (check)
            {
                poses.push_back(chunk_pos_from_tag(name));
            }
        }
        else
        {
            poses.push_back(chunk_pos_from_tag(name));
        }
    }

    return poses;
}

Vector3i GlobalMap::chunk_pos_from_tag(std::string &tag)
{
    std::stringstream tmp(tag);
    std::string segment;
    std::vector<std::string> seglist;

    while (std::getline(tmp, segment, '_'))
    {
        seglist.push_back(segment);
    }

    if (seglist.size() != 3)
    {
        throw std::logic_error("Error when transforming a tag into a chunk pos");
    }

    // get coords
    int x = std::stoi(seglist[0]);
    int y = std::stoi(seglist[1]);
    int z = std::stoi(seglist[2]);

    return Vector3i(x, y, z);
}

int GlobalMap::num_chunks()
{
    HighFive::Group g = file_.getGroup("/map");
    auto object_names = g.listObjectNames();

    return object_names.size();
}