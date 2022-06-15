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
    if (!file_.exist(hdf5_constants::MAP_GROUP_NAME))
    {
        file_.createGroup(hdf5_constants::MAP_GROUP_NAME);
    }
    else
    {
        std::cout << "[GLOBAL_MAP] Using existing HDF5 Parameter [/map]" << std::endl;
    }

    if (!file_.exist(hdf5_constants::POSES_GROUP_NAME))
    {
        file_.createGroup(hdf5_constants::POSES_GROUP_NAME);
    }
    else
    {
        std::cout << "[GLOBAL_MAP] Using existing HDF5 Parameter [/poses]" << std::endl;
    }

    // prior to doing anything with the global map association data, we clear it completely
    if (has_path())
    {
        clear_association_data();
        clear_intersection_data();
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

/**
 * @todo this is the problem, why no intersection data is written to the global map:
 * when activating a chunk, we simply return a vector of rawtypes, which does not include the intersection status, therefore
 * we can neihter write, nor read any intersection data
 *
 * The current approach here is to also write/read the intersection data for visualiuation by overriding a pointer to intersection data.
 * A pointer pointing to no memory is readjusted to point at the specific intersection data in the active chunks, or it is gathered from the hdf5.
 * when a chunk is supposed to be removed from the active chunks, the intersection data is also written to hdf5
 */
std::vector<TSDFEntry::RawType> &GlobalMap::activate_chunk(const Vector3i &chunkPos, std::vector<int> *&intersections)
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

        HighFive::Group g = file_.getGroup(hdf5_constants::MAP_GROUP_NAME);

        // create group if not exists
        if (!file_.exist(hdf5_constants::INTERSECTIONS_GROUP_NAME))
        {
            file_.createGroup(hdf5_constants::INTERSECTIONS_GROUP_NAME);
        }

        HighFive::Group g_int = file_.getGroup(hdf5_constants::INTERSECTIONS_GROUP_NAME);

        auto tag = tag_from_chunk_pos(chunkPos);

        if (g.exist(tag))
        {
            // read chunk from file
            HighFive::DataSet d = g.getDataSet(tag);
            d.read(newChunk.data);

            if (g_int.exist(tag))
            {
                // read chunk from file
                HighFive::DataSet d = g_int.getDataSet(tag);
                d.read(newChunk.intersect_data);
            }
            else
            {
                newChunk.intersect_data = std::vector<int>(CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE, TSDFEntry::IntersectStatus::NO_INT);
            }
        }
        else
        {
            // create new chunk
            newChunk.data = std::vector<TSDFEntry::RawType>(CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE, initial_tsdf_value_.raw());
            newChunk.intersect_data = std::vector<int>(CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE, TSDFEntry::IntersectStatus::NO_INT);
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

            // already exists in file
            if (g.exist(tag))
            {
                auto d = g.getDataSet(tag);
                d.write(active_chunks_[index].data);
            }
            else
            {
                // if it doesnt yet exist, we create a completely new one
                g.createDataSet(tag, active_chunks_[index].data);
            }

            // now write the intersection data accordingly
            if (g_int.exist(tag))
            {
                std::cout << "[GlobalMap::::::::::: activate_chunk] - WRITE" << std::endl;
                auto d = g_int.getDataSet(tag);
                d.write(active_chunks_[index].intersect_data);
            }
            else
            {
                std::cout << "[GlobalMap::::::::::: activate_chunk] - CREATE" << std::endl;
                g_int.createDataSet(tag, active_chunks_[index].intersect_data);
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

    // age of the newly added chunk is zero
    active_chunks_[index].age = 0;

    // point intersect pointer ref to intersection data of new chunk
    intersections = &(active_chunks_[index].intersect_data);

    if(intersections == NULL) {
        throw std::logic_error("Intersections NULL");
    }

    return active_chunks_[index].data;
}

TSDFEntry GlobalMap::get_value(const Vector3i &pos)
{
    Vector3i chunkPos = floor_divide(pos, CHUNK_SIZE);
    std::vector<int> *int_status;
    const auto &chunk = activate_chunk(chunkPos, int_status);

    int index = index_from_pos(pos, chunkPos);

    auto entry = TSDFEntry(chunk[index]);
    entry.setIntersect(static_cast<TSDFEntry::IntersectStatus>(int_status->operator[](index)));
    return entry;
}

void GlobalMap::set_value(const Vector3i &pos, const TSDFEntry &value)
{
    Vector3i chunkPos = floor_divide(pos, CHUNK_SIZE);
    std::vector<int> *int_status;
    auto &chunk = activate_chunk(chunkPos, int_status);
    int index = index_from_pos(pos, chunkPos);
    chunk[index] = value.raw();
    int_status->operator[](index) = value.getIntersect();
}

void GlobalMap::write_back()
{
    HighFive::Group g = file_.getGroup(hdf5_constants::MAP_GROUP_NAME);

    // create intersections group, if not exists.
    if (!file_.exist(hdf5_constants::INTERSECTIONS_GROUP_NAME))
    {
        file_.createGroup(hdf5_constants::INTERSECTIONS_GROUP_NAME);
    }

    HighFive::Group g_int = file_.getGroup(hdf5_constants::INTERSECTIONS_GROUP_NAME);

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

        // if intersection data is already written, get the dataset and override, else just create a new ds
        if (g_int.exist(tag))
        {
            auto d = g_int.getDataSet(tag);
            d.write(chunk.intersect_data);
        }
        else
        {
            g_int.createDataSet(tag, chunk.intersect_data);
        }
    }
    file_.flush();
}

bool GlobalMap::chunk_exists(const Vector3i &chunk_pos)
{
    HighFive::Group g = file_.getGroup(hdf5_constants::MAP_GROUP_NAME);

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
    HighFive::Group g = file_.getGroup(hdf5_constants::MAP_GROUP_NAME);
    auto object_names = g.listObjectNames();
    std::vector<Vector3i> poses;

    for (auto name : object_names)
    {
        // skip intersection data
        if (name.find(std::string("int")) != std::string::npos)
        {
            continue;
        }

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
    HighFive::Group g = file_.getGroup(hdf5_constants::MAP_GROUP_NAME);
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
    if (!file_.exist(hdf5_constants::POSES_GROUP_NAME))
    {
        throw std::logic_error("There are no poses in the delivered global map... aborting...");
    }

    HighFive::Group g = file_.getGroup(hdf5_constants::POSES_GROUP_NAME);
    auto object_names = g.listObjectNames();

    // as the identifier sorting in hdf5 is a string sorting, we need to sort the array based on the integer values behind the string
    sort_vector(object_names, true);

    std::vector<Pose> path;

    // if there is no path listed in the map, we simply return the empty vector
    if (object_names.size() == 0)
    {
        return path;
    }

    std::cout << "[GobalMap - getPath] Obtaining path" << std::endl;

    for (auto name : object_names)
    {
        if (!g.exist(name))
        {
            throw std::logic_error("Error when reading paths from hdf5");
        }

        std::cout << "Current identifier: " << name << std::endl;

        // every pose is a group of two datasets
        auto sub_g = g.getGroup(name);

        if (!sub_g.exist(hdf5_constants::POSE_DATASET_NAME))
        {
            continue;
        }

        auto pose_ds = sub_g.getDataSet(hdf5_constants::POSE_DATASET_NAME);

        std::vector<float> values;
        pose_ds.read(values);

        // there need to be 7 values (x, y, z, x_quat, y_quat, z_quat, w_quat) for every Pose.
        if (values.size() != hdf5_constants::POSE_DATASET_SIZE)
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

        std::cout << pose << std::endl;

        path.push_back(pose);
    }

    std::cout << "[GlobalMap - getPath] Obtained " << path.size() << " Poses from HDF5!" << std::endl;

    return path;
}

void GlobalMap::write_path(std::vector<Pose> &poses)
{
    if (!file_.exist(hdf5_constants::POSES_GROUP_NAME))
    {
        file_.createGroup(hdf5_constants::POSES_GROUP_NAME);
    }

    HighFive::Group g = file_.getGroup(hdf5_constants::POSES_GROUP_NAME);

    // if there are poses in the globalmap, we abort
    if (g.listObjectNames().size() != 0)
    {
        std::cout << "[GlobalMap]: Path already exists in global-map, aborting writing path" << std::endl;
        return;
    }

    int identifier = 0;

    for (auto pose : poses)
    {
        std::vector<float> values(hdf5_constants::POSE_DATASET_SIZE);

        // round to three decimal places here.
        values[0] = std::round(pose.pos.x() * 1000.0f) / 1000.0f;
        values[1] = std::round(pose.pos.y() * 1000.0f) / 1000.0f;
        values[2] = std::round(pose.pos.z() * 1000.0f) / 1000.0f;
        values[3] = std::round(pose.quat.x() * 1000.0f) / 1000.0f;
        values[4] = std::round(pose.quat.y() * 1000.0f) / 1000.0f;
        values[5] = std::round(pose.quat.z() * 1000.0f) / 1000.0f;
        values[6] = std::round(pose.quat.w() * 1000.0f) / 1000.0f;

        std::string subgroup_str = std::string(hdf5_constants::POSES_GROUP_NAME) + "/" + std::to_string(identifier);

        std::cout << "Creating group in Path: " << subgroup_str << std::endl;
        auto sub_g = g.createGroup(subgroup_str);
        sub_g.createDataSet(hdf5_constants::POSE_DATASET_NAME, values);

        identifier++;
    }

    file_.flush();
}

void GlobalMap::write_path_node(Pose &pose)
{
    if (!file_.exist(hdf5_constants::POSES_GROUP_NAME))
    {
        file_.createGroup(hdf5_constants::POSES_GROUP_NAME);
    }

    HighFive::Group g = file_.getGroup(hdf5_constants::POSES_GROUP_NAME);

    size_t identfier = g.listObjectNames().size();

    // create a new sub group for the new pose
    auto sub_g = g.createGroup(std::string(hdf5_constants::POSES_GROUP_NAME) + "/" + std::to_string(identfier));

    std::vector<float> values(hdf5_constants::POSE_DATASET_SIZE);

    // round to three decimal places here.
    values[0] = std::round(pose.pos.x() * 1000.0f) / 1000.0f;
    values[1] = std::round(pose.pos.y() * 1000.0f) / 1000.0f;
    values[2] = std::round(pose.pos.z() * 1000.0f) / 1000.0f;
    values[3] = std::round(pose.quat.x() * 1000.0f) / 1000.0f;
    values[4] = std::round(pose.quat.y() * 1000.0f) / 1000.0f;
    values[5] = std::round(pose.quat.z() * 1000.0f) / 1000.0f;
    values[6] = std::round(pose.quat.w() * 1000.0f) / 1000.0f;

    sub_g.createDataSet(hdf5_constants::POSE_DATASET_NAME, values);

    file_.flush();
}

/**
 * @brief
 *
 * @todo this method still needs some work
 */
void GlobalMap::cleanup_artifacts()
{
    auto map = file_.getGroup(hdf5_constants::MAP_GROUP_NAME);

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

    // create an attribute for the global map, to ensure it doesnt get cleaned multiple times
    map.createAttribute("cleaned", true);
}

void GlobalMap::write_association_data(std::vector<int> &association_data, int pose_number)
{
    std::cout << "[GlobalMap] Start writing associaton data for " << association_data.size() << " objects." << std::endl;

    // when there is no poses, we dont do nothing
    if (!file_.exist(hdf5_constants::POSES_GROUP_NAME))
    {
        return;
    }

    auto g = file_.getGroup(hdf5_constants::POSES_GROUP_NAME);

    // if there aren't any poses or the one specified by the function call doesn't exist
    if (g.listObjectNames().size() == 0 || !g.exist(std::string(hdf5_constants::POSES_GROUP_NAME) + "/" + std::to_string(pose_number)))
    {
        return;
    }

    auto sub_g = g.getGroup(std::string(hdf5_constants::POSES_GROUP_NAME) + "/" + std::to_string(pose_number));

    // get the dataset and write the data.

    if (sub_g.exist(hdf5_constants::ASSOCIATION_DATASET_NAME))
    {
        throw std::logic_error("Called the write_association_data method, without clearing the association data first");
    }

    sub_g.createDataSet(hdf5_constants::ASSOCIATION_DATASET_NAME, association_data);

    file_.flush();

    std::cout << "[GlobalMap] Done writing association data" << std::endl;
}

std::vector<int> GlobalMap::read_association_data(int pose_number)
{
    // when there is no poses, we dont do nothing
    if (!file_.exist(hdf5_constants::POSES_GROUP_NAME) || file_.getGroup(hdf5_constants::POSES_GROUP_NAME).listObjectNames().size() == 0)
    {
        std::vector<int>();
    }

    auto g = file_.getGroup(hdf5_constants::POSES_GROUP_NAME);

    // when the passed number (pose) does not exist, we also return
    if (!g.exist(std::string(hdf5_constants::POSES_GROUP_NAME) + "/" + std::to_string(pose_number)))
    {
        std::vector<int>();
    }

    // get the pose group
    auto sub_g = g.getGroup(std::string(hdf5_constants::POSES_GROUP_NAME) + "/" + std::to_string(pose_number));

    // when the current pose has no association dataset present in the hdf5
    if (!sub_g.exist(hdf5_constants::ASSOCIATION_DATASET_NAME))
    {
        return std::vector<int>();
    }

    // else get the association dataset and read the data
    auto d = sub_g.getDataSet(hdf5_constants::ASSOCIATION_DATASET_NAME);
    std::vector<int> data;
    d.read(data);

    return data;
}

void GlobalMap::clear_association_data()
{
    auto g = file_.getGroup(hdf5_constants::POSES_GROUP_NAME);

    auto object_names = g.listObjectNames();

    for (auto name : object_names)
    {
        auto sub_g = g.getGroup(name);

        // skip, if no assocication dataset exists in the first place
        if (!sub_g.exist(hdf5_constants::ASSOCIATION_DATASET_NAME))
        {
            continue;
        }

        std::string channel_name = std::string(hdf5_constants::POSES_GROUP_NAME) + "/" + name + "/" + hdf5_constants::ASSOCIATION_DATASET_NAME;

        std::cout << "Channel Name: " << channel_name << std::endl;

        // delete the dataset
        int status = H5Ldelete(file_.getId(), channel_name.data(), H5P_DEFAULT);

        std::cout << "[GlobalMap] Delete Status: " << status << std::endl;
    }

    file_.flush();
}

void GlobalMap::clear_intersection_data()
{
    // delete any intersection data associated with the current hdf5

    if (!file_.exist(hdf5_constants::INTERSECTIONS_GROUP_NAME))
    {
        return;
    }

    auto g = file_.getGroup(hdf5_constants::INTERSECTIONS_GROUP_NAME);

    auto object_names = g.listObjectNames();

    for (auto name : object_names)
    {
        std::string channel_name = std::string(hdf5_constants::INTERSECTIONS_GROUP_NAME) + "/" + name;
        std::cout << "Channel Name Intersections: " << channel_name << std::endl;

        // delete the dataset
        int status = H5Ldelete(file_.getId(), channel_name.data(), H5P_DEFAULT);

        std::cout << "[GlobalMap] Intersection Delete Status: " << status << std::endl;
    }

    std::cout << "[GlobalMap - clear_intersection_data] Cleared" << std::endl;

    file_.flush();
}
