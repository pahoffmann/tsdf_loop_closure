/**
 * @author Steffen Hinderink
 * @author Juri Vana
 * @author Malte Hillmann
 * @author Patrick Hoffmann
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

    // prior to doing anything with the global map association data, we clear it completely
    if (has_path())
    {
        clear_association_data();
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

        auto intersect_tag = tag + "_int";

        if (g.exist(tag))
        {
            // read chunk from file
            HighFive::DataSet d = g.getDataSet(tag);
            d.read(newChunk.data);

            if (g.exist(intersect_tag))
            {
                // read chunk from file
                HighFive::DataSet d = g.getDataSet(tag);
                d.read(newChunk.intersect_data);
            }
            else
            {
                newChunk.intersect_data = std::vector<int32_t>(CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE, TSDFEntry::IntersectStatus::NO_INT);
            }
        }
        else
        {
            // create new chunk
            newChunk.data = std::vector<TSDFEntry::RawType>(CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE, initial_tsdf_value_.raw());
            /*std::stringstream ss;
            ss << "A new chunk was created, this should never happen. At least during path exploration " << std::endl << chunkPos << std::endl << "Tag: " << tag << std::endl;
            std::cout << ss.str() << std::endl;
            throw std::logic_error(ss.str());*/
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
            auto intersect_tag = tag + "_int";

            if (g.exist(tag))
            {
                auto d = g.getDataSet(tag);
                d.write(active_chunks_[index].data);
            }
            else
            {
                g.createDataSet(tag, active_chunks_[index].data);
            }

            if (g.exist(intersect_tag))
            {
                auto d = g.getDataSet(intersect_tag);
                d.write(active_chunks_[index].intersect_data);
            }
            else
            {
                g.createDataSet(intersect_tag, active_chunks_[index].intersect_data);
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

    if (g.exist(tag))
    {
        return true;
    }
    else
    {
        return false;
    }
}

std::vector<Vector3i> GlobalMap::get_26_neighborhood(Vector3i chunk_pos)
{
    std::vector<Vector3i> chunks;

    // manhatten
    chunks.push_back(chunk_pos + Vector3i(0, 0, 1));
    chunks.push_back(chunk_pos + Vector3i(0, 0, -1));
    chunks.push_back(chunk_pos + Vector3i(0, 1, 0));
    chunks.push_back(chunk_pos + Vector3i(0, -1, 0));
    chunks.push_back(chunk_pos + Vector3i(1, 0, 0));
    chunks.push_back(chunk_pos + Vector3i(-1, 0, 0));

    // different neighborhood vertices

    // same level as chunk (z same)
    chunks.push_back(chunk_pos + Vector3i(1, 1, 0));
    chunks.push_back(chunk_pos + Vector3i(-1, -1, 0));
    chunks.push_back(chunk_pos + Vector3i(1, -1, 0));
    chunks.push_back(chunk_pos + Vector3i(-1, 1, 0));

    // above
    chunks.push_back(chunk_pos + Vector3i(0, 1, 1));
    chunks.push_back(chunk_pos + Vector3i(0, -1, 1));
    chunks.push_back(chunk_pos + Vector3i(1, 0, 1));
    chunks.push_back(chunk_pos + Vector3i(-1, 0, 1));
    chunks.push_back(chunk_pos + Vector3i(1, 1, 1));
    chunks.push_back(chunk_pos + Vector3i(-1, -1, 1));
    chunks.push_back(chunk_pos + Vector3i(1, -1, 1));
    chunks.push_back(chunk_pos + Vector3i(-1, 1, 1));

    // below
    chunks.push_back(chunk_pos + Vector3i(0, 1, -1));
    chunks.push_back(chunk_pos + Vector3i(0, -1, -1));
    chunks.push_back(chunk_pos + Vector3i(1, 0, -1));
    chunks.push_back(chunk_pos + Vector3i(-1, 0, -1));
    chunks.push_back(chunk_pos + Vector3i(1, 1, -1));
    chunks.push_back(chunk_pos + Vector3i(-1, -1, -1));
    chunks.push_back(chunk_pos + Vector3i(1, -1, -1));
    chunks.push_back(chunk_pos + Vector3i(-1, 1, -1));

    return chunks;
}

std::vector<Vector3i> GlobalMap::all_chunk_poses(Vector3i l_map_size)
{
    HighFive::Group g = file_.getGroup("/map");
    auto object_names = g.listObjectNames();
    std::vector<Vector3i> poses;

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
            Vector3i global_tmp = tmp * CHUNK_SIZE;
            bool check = is_fully_occupied(global_tmp, l_map_size);

            // if still every chunk around the current chunk is registered, it is a possible valid path pos
            if (check)
            {
                poses.push_back(tmp);
            }
        }
        else
        {
            poses.push_back(chunk_pos_from_tag(name));
        }
    }

    return poses;
}

Vector3i GlobalMap::chunk_pos_from_tag(std::string tag) const
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

bool GlobalMap::is_fully_occupied(Eigen::Vector3i &pos, Eigen::Vector3i &localmap_size)
{
    const Vector3i &bottom_corner = pos - localmap_size / 2;
    const Vector3i &top_corner = pos + localmap_size / 2;

    Vector3i start = bottom_corner.cwiseMin(top_corner);
    Vector3i end = bottom_corner.cwiseMax(top_corner);

    // calculate the chunk index
    Vector3i chunk_start = floor_divide(start, CHUNK_SIZE);
    Vector3i chunk_end = floor_divide(end, CHUNK_SIZE);

    // std::cout << "[GlobalMap] Chunk start: " << std::endl << chunk_start << std::endl << "[GlobalMap] Chunk end: " << std::endl << chunk_end << std::endl;

    // check all chunks in the local-map sized bounding box around "pos"
    for (int chunk_x = chunk_start.x(); chunk_x <= chunk_end.x(); ++chunk_x)
    {
        for (int chunk_y = chunk_start.y(); chunk_y <= chunk_end.y(); ++chunk_y)
        {
            for (int chunk_z = chunk_start.z(); chunk_z <= chunk_end.z(); ++chunk_z)
            {
                if (!chunk_exists(Eigen::Vector3i(chunk_x, chunk_y, chunk_z)))
                {
                    return false;
                }
            }
        }
    }

    // when all chunks are already generated, the local map would be fully occupied and no further chunk would need to be generated.
    return true;
}

std::vector<Pose> GlobalMap::get_path()
{
    if (!file_.exist("/poses"))
    {
        throw std::logic_error("There are no poses in the delivered global map... aborting...");
    }

    HighFive::Group g = file_.getGroup("/poses");
    auto object_names = g.listObjectNames();

    std::vector<Pose> path;

    // if there is no path listed in the map, we simply return the empty vector
    if (object_names.size() == 0)
    {
        return path;
    }

    for (auto name : object_names)
    {
        if (!g.exist(name))
        {
            throw std::logic_error("Error when reading paths from hdf5");
        }

        std::vector<float> values;
        auto dataset = g.getDataSet(name);
        dataset.getAttribute("pose").read(values);

        // there need to be 7 values (x, y, z, x_quat, y_quat, z_quat, w_quat) for every Pose.
        if (values.size() != POSE_ATTRIBUTE_SIZE)
        {
            continue;
        }

        // get pose
        Pose pose;
        pose.pos = Vector3f(values[0], values[1], values[2]);
        pose.quat.x() = values[3];
        pose.quat.y() = values[4];
        pose.quat.z() = values[5];
        pose.quat.w() = values[6];

        path.push_back(pose);
    }

    return path;
}

void GlobalMap::write_path(std::vector<Pose> &poses)
{
    if (!file_.exist("/poses"))
    {
        file_.createGroup("/poses");
    }

    HighFive::Group g = file_.getGroup("/poses");

    // if there are poses in the globalmap, we abort
    if (g.listObjectNames().size() != 0)
    {
        std::cout << "[GlobalMap]: Path already exists in global-map, aborting writing path" << std::endl;
        return;
    }

    int identifier = 0;

    for (auto pose : poses)
    {
        std::vector<float> values(POSE_ATTRIBUTE_SIZE);

        // round to three decimal places here.
        values[0] = std::round(pose.pos.x() * 1000.0f) / 1000.0f;
        values[1] = std::round(pose.pos.y() * 1000.0f) / 1000.0f;
        values[2] = std::round(pose.pos.z() * 1000.0f) / 1000.0f;
        values[3] = std::round(pose.quat.x() * 1000.0f) / 1000.0f;
        values[4] = std::round(pose.quat.y() * 1000.0f) / 1000.0f;
        values[5] = std::round(pose.quat.z() * 1000.0f) / 1000.0f;
        values[6] = std::round(pose.quat.w() * 1000.0f) / 1000.0f;

        std::string dataset_str = std::to_string(identifier);

        std::cout << "Creating dataset in Path: " << dataset_str << std::endl;

        auto ds = g.createDataSet(dataset_str, NULL);
        
        ds.createAttribute("pose", values);
        identifier++;
    }

    file_.flush();
}

void GlobalMap::write_path_node(Pose &pose)
{
    if (!file_.exist("/poses"))
    {
        file_.createGroup("/poses");
    }

    HighFive::Group g = file_.getGroup("/poses");

    std::vector<float> values(POSE_ATTRIBUTE_SIZE);

    // round to three decimal places here.
    values[0] = std::round(pose.pos.x() * 1000.0f) / 1000.0f;
    values[1] = std::round(pose.pos.y() * 1000.0f) / 1000.0f;
    values[2] = std::round(pose.pos.z() * 1000.0f) / 1000.0f;
    values[3] = std::round(pose.quat.x() * 1000.0f) / 1000.0f;
    values[4] = std::round(pose.quat.y() * 1000.0f) / 1000.0f;
    values[5] = std::round(pose.quat.z() * 1000.0f) / 1000.0f;
    values[6] = std::round(pose.quat.w() * 1000.0f) / 1000.0f;

    g.createDataSet(tag_from_vec(Vector3f(values[0], values[1], values[2])), values);

    file_.flush();
}

void GlobalMap::cleanup_artifacts()
{
    auto map = file_.getGroup("/map");

    // check, if the map already is cleaned in terms of artifacts.
    if (map.hasAttribute("cleaned"))
    {
        std::cout << "[GlobalMap]: Map already cleaned, no cleanup necessary" << std::endl;
        return;
    }

    auto chunks = all_chunk_poses();
    int num_shitty = 0;

    for (int i = 0; i < chunks.size(); i++)
    {

        Vector3i start = chunks[i] * CHUNK_SIZE;
        Vector3i end = start + Vector3i(CHUNK_SIZE, CHUNK_SIZE, CHUNK_SIZE);

        for (int x = start.x(); x < end.x(); x++)
        {
            for (int y = start.y(); y < end.y(); y++)
            {
                for (int z = start.z(); z < end.z(); z++)
                {
                    auto current = get_value(Vector3i(x, y, z));

                    // if the current value is not interesting at all
                    if (current.value() == 600 || current.weight() == 0)
                    {
                        continue;
                    }

                    auto current_chunk = floor_divide(Vector3i(x, y, z), CHUNK_SIZE);

                    // seems counter-intuitive, but this method simply gets the 26-neighbordhood..
                    auto adj_cells = get_26_neighborhood(Vector3i(x, y, z));

                    int num_interesting = 0;

                    for (auto cell : adj_cells)
                    {
                        Vector3i chunkPos = floor_divide(cell, CHUNK_SIZE);

                        // ignore edge cases, where the chunk doesnt exist.
                        if (chunkPos != current_chunk && !chunk_exists(chunkPos))
                        {
                            continue;
                        }

                        auto tsdf = get_value(cell);

                        if (tsdf.value() < 600 && tsdf.weight() > 0)
                        {
                            num_interesting++;
                        }
                    }

                    if (num_interesting < 3)
                    {
                        // std::cout << "Found a cell which is rather shitty." << std::endl;

                        // mark as not so interesting in the tsdf...
                        current.weight(0);
                        current.value(600);

                        set_value(Vector3i(x, y, z), current);
                        num_shitty++;
                    }
                }
            }
        }

        std::cout << "[GlobalMap]: Chunk " << i + 1 << " of " << chunks.size() << " done. Currently found " << num_shitty << " cells" << std::endl;
    }

    write_back();

    map.createAttribute("cleaned", true);
}

void GlobalMap::write_association_data(std::vector<int> &association_data, int pose_number)
{
    // TODO: implement this
}

std::vector<int> GlobalMap::read_association_data(int pose_number)
{
    // TODO: implement this
    return std::vector<int>();
}

void GlobalMap::clear_association_data()
{
    auto g = file_.getGroup("/poses");

    auto object_names = g.listObjectNames();

    for(auto name : object_names) {
        auto ds = g.getDataSet(name);

        std::string channel_name = std::string("/poses") + name;

        std::cout << "Channel Name: " << channel_name << std::endl;

        std::vector<float> attribute_data;

        if(!ds.hasAttribute("pose")) {
            throw std::logic_error("[GlobalMap - clear_association_data] Dataset has no pose attribute");
        }

        ds.getAttribute("pose").read(attribute_data);

        if(attribute_data.size() != POSE_ATTRIBUTE_SIZE) 
        {
            throw std::logic_error("[GlobalMap - clear_association_data] Dataset attribute has wrong number of values");
        }
        
        // delete the dataset
        int status = H5Ldelete(file_.getId(), channel_name.data(), H5P_DEFAULT);

        // recreate empty ds
        auto new_ds = g.createDataSet(name, NULL);
        new_ds.createAttribute("pose", attribute_data);
    }
}
