/**
 * @author Steffen Hinderink
 * @author Juri Vana
 * @author Malte Hillmann
 * @author Patrick Hoffmann
 */

#include <sstream>
#include <ros/ros.h>

#include <loop_closure/map/global_map.h>

GlobalMap::GlobalMap(const MapParams &input_params)
    : initial_tsdf_value_{static_cast<TSDFEntry::ValueType>(input_params.tau), static_cast<TSDFEntry::WeightType>(input_params.initial_weight)},
      active_chunks_{},
      num_poses_{0}
{
    // Truncate clears already existing file
    file_.reset(new HighFive::File(std::string(input_params.filename.c_str()), HighFive::File::OpenOrCreate | HighFive::File::Truncate));
    if (!file_->exist(hdf5_constants::MAP_GROUP_NAME))
    {
        file_->createGroup(hdf5_constants::MAP_GROUP_NAME);
    }

    if (!file_->exist(hdf5_constants::POSES_GROUP_NAME))
    {
        file_->createGroup(hdf5_constants::POSES_GROUP_NAME);
    }

    params = input_params;

    write_meta(params);

    default_chunk_data = std::vector<TSDFEntry::RawType>(CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE, initial_tsdf_value_.raw());
}

GlobalMap::GlobalMap(std::string name, TSDFEntry::ValueType initial_tsdf_value, TSDFEntry::WeightType initial_weight, bool use_attributes)
    : initial_tsdf_value_{initial_tsdf_value, initial_weight},
      active_chunks_{},
      num_poses_{0}
{
    file_.reset(new HighFive::File(name, HighFive::File::OpenOrCreate));
    if (!file_->exist(hdf5_constants::MAP_GROUP_NAME))
    {
        file_->createGroup(hdf5_constants::MAP_GROUP_NAME);
    }
    else
    {
        std::cout << "[GLOBAL_MAP] Using existing HDF5 Parameter [/map]" << std::endl;
    }

    if (!file_->exist(hdf5_constants::POSES_GROUP_NAME))
    {
        file_->createGroup(hdf5_constants::POSES_GROUP_NAME);
    }
    else
    {
        std::cout << "[GLOBAL_MAP] Using existing HDF5 Parameter [/poses]" << std::endl;
    }

    auto map_group = file_->getGroup(hdf5_constants::MAP_GROUP_NAME);

    // check if attribute data should be considered, if so:
    // 1. check if the number of attributes fit the definition
    // 2. create a data model based on those attributes
    if (use_attributes && map_group.listAttributeNames().size() < hdf5_constants::NUM_GM_ATTRIBUTES)
    {
        throw std::invalid_argument("[GLOBAL_MAP] The delivered map does not contain attribute data");
    }
    else
    {
        int map_resolution, max_weight, tau;
        float map_size_x, map_size_y, map_size_z, max_distance;

        try
        {
            map_group.getAttribute("map_resolution").read(map_resolution);
            map_group.getAttribute("max_weight").read(max_weight);
            map_group.getAttribute("tau").read(tau);
            map_group.getAttribute("map_size_x").read(map_size_x);
            map_group.getAttribute("map_size_y").read(map_size_y);
            map_group.getAttribute("map_size_z").read(map_size_z);
            map_group.getAttribute("max_distance").read(max_distance);
        }
        catch (HighFive::AttributeException e)
        {
            std::cout << "[GlobalMap] Exception, when trying to read the map attributes" << std::endl;
            std::cout << e.what() << std::endl;

            exit(EXIT_FAILURE);
        }

        // create the attribute data model
        attribute_data_ = attribute_data_model(map_resolution, map_size_x, map_size_y, map_size_z, max_distance, max_weight, tau);

        // set data from attributes
        initial_tsdf_value_.value(tau);
        initial_tsdf_value_.weight(0);
    }

    // prior to doing anything with the global map association data, we clear it completely
    if (has_path())
    {
        clear_association_data();
    }

    // init default chunk
    default_chunk_data = std::vector<TSDFEntry::RawType>(CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE, initial_tsdf_value_.raw());
}

GlobalMap::~GlobalMap()
{
    // file_->flush();
    // H5Fclose(file_->getId());
    // std::cout << "Removing file with 1 name: " << file_->getName() << std::endl;
    // boost::filesystem::remove(file_->getName());

    // file_.reset();
}

void GlobalMap::clear()
{
    // std::cout << "Removing file with name: " << file_->getName() << std::endl;
    // H5Fclose(file_->getId());
    // boost::filesystem::remove(file_->getName());
    // file_->flush();
    // H5Fclose(file_->getId());
    // // std::cout << "Removing file with name: " << file_->getName() << std::endl;
    // // boost::filesystem::remove(file_->getName());

    // file_.reset();
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

        HighFive::Group g = file_->getGroup(hdf5_constants::MAP_GROUP_NAME);

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

    return active_chunks_[index].data;
}

TSDFEntry GlobalMap::get_value(const Vector3i &pos)
{
    Vector3i chunkPos = floor_divide(pos, CHUNK_SIZE);
    const auto &chunk = activate_chunk(chunkPos);

    int index = index_from_pos(pos, chunkPos);

    auto entry = TSDFEntry(chunk[index]);
    return entry;
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
    HighFive::Group g = file_->getGroup(hdf5_constants::MAP_GROUP_NAME);

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
    file_->flush();
}

bool GlobalMap::chunk_exists(const Vector3i &chunk_pos)
{
    HighFive::Group g = file_->getGroup(hdf5_constants::MAP_GROUP_NAME);

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

/**
 * @brief function, which finds all the chunks, which could have been part of the path
 *
 * @param l_map_size
 * @return std::vector<Vector3i>
 */
std::vector<Vector3i> GlobalMap::all_chunk_poses(Vector3i l_map_size)
{
    HighFive::Group g = file_->getGroup(hdf5_constants::MAP_GROUP_NAME);
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
    HighFive::Group g = file_->getGroup(hdf5_constants::MAP_GROUP_NAME);
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
    if (!file_->exist(hdf5_constants::POSES_GROUP_NAME))
    {
        throw std::logic_error("There are no poses in the delivered global map... aborting...");
    }

    HighFive::Group g = file_->getGroup(hdf5_constants::POSES_GROUP_NAME);
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

        path.push_back(pose);
    }

    std::cout << "[GlobalMap - getPath] Obtained " << path.size() << " Poses from HDF5!" << std::endl;

    return path;
}

void GlobalMap::write_path(std::vector<Pose> &poses)
{
    if (!file_->exist(hdf5_constants::POSES_GROUP_NAME))
    {
        file_->createGroup(hdf5_constants::POSES_GROUP_NAME);
    }

    HighFive::Group g = file_->getGroup(hdf5_constants::POSES_GROUP_NAME);

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

    file_->flush();
}

void GlobalMap::write_path_node(Pose &pose)
{
    if (!file_->exist(hdf5_constants::POSES_GROUP_NAME))
    {
        file_->createGroup(hdf5_constants::POSES_GROUP_NAME);
    }

    HighFive::Group g = file_->getGroup(hdf5_constants::POSES_GROUP_NAME);

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

    file_->flush();
}

/**
 * @brief
 *
 * @todo this method still needs some work
 */
std::vector<Vector3i> GlobalMap::cleanup_artifacts()
{
    auto map = file_->getGroup(hdf5_constants::MAP_GROUP_NAME);

    // check, if the map already is cleaned in terms of artifacts.
    if (map.hasAttribute("cleaned"))
    {
        std::cout << "[GlobalMap]: Map already cleaned, no cleanup necessary" << std::endl;
        return std::vector<Vector3i>();
    }

    TSDFEntry default_entry(600, 0);

    auto chunks = all_chunk_poses();
    int num_shitty = 0;

    // detect empty chunks
    auto empty_vec = chunks_empty();

    std::cout << "[GlobalMap] Identifying outliers while skipping empty chunks..." << std::endl;

    // vector to store the outliers to display them later on
    std::vector<Vector3i> shitty_cells;

    for (int i = 0; i < chunks.size(); i++)
    {
        // if the current chunk is empty, go to the next one
        if (empty_vec[i])
        {
            continue;
        }

        int wrong_counter = 0;

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

                    for (int i = 0; i < adj_cells.size(); i++)
                    {
                        auto cell = adj_cells[i];

                        Vector3i chunkPos = floor_divide(cell, CHUNK_SIZE);

                        // ignore edge cases, where the chunk doesnt exist.
                        if (chunkPos == current_chunk && chunk_exists(chunkPos))
                        {
                            auto tsdf = get_value(cell);

                            if (tsdf.value() < 600 && tsdf.weight() > 0)
                            {
                                num_interesting++;
                            }
                        }
                    }

                    if (num_interesting < 6)
                    {
                        // mark as not so interesting in the tsdf...
                        set_value(Vector3i(x, y, z), default_entry);

                        // test if has been set:
                        auto test = get_value(Vector3i(x, y, z));

                        if (test.value() != default_entry.value() || test.weight() != default_entry.weight())
                        {
                            wrong_counter++;
                        }

                        shitty_cells.push_back(Vector3i(x, y, z));
                        num_shitty++;
                    }
                }
            }
        }

        std::cout << "[GlobalMap]: Chunk " << i + 1 << " of " << chunks.size() << " done. Currently found " << num_shitty << " cells" << std::endl;
        std::cout << "[Globalmap] Errors when updating: " << wrong_counter << std::endl;

        write_back();
    }

    // create an attribute for the global map, to ensure it doesnt get cleaned multiple times
    map.createAttribute("cleaned", true);

    return shitty_cells;
}

void GlobalMap::write_association_data(std::vector<int> &association_data, int pose_number)
{
    std::cout << "[GlobalMap] Start writing associaton data for " << association_data.size() << " objects." << std::endl;

    // when there is no poses, we dont do nothing
    if (!file_->exist(hdf5_constants::POSES_GROUP_NAME))
    {
        return;
    }

    auto g = file_->getGroup(hdf5_constants::POSES_GROUP_NAME);

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

    file_->flush();

    std::cout << "[GlobalMap] Done writing association data" << std::endl;
}

std::vector<int> GlobalMap::read_association_data(int pose_number)
{
    // when there is no poses, we dont do nothing
    if (!file_->exist(hdf5_constants::POSES_GROUP_NAME) || file_->getGroup(hdf5_constants::POSES_GROUP_NAME).listObjectNames().size() == 0)
    {
        std::vector<int>();
    }

    auto g = file_->getGroup(hdf5_constants::POSES_GROUP_NAME);

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
    auto g = file_->getGroup(hdf5_constants::POSES_GROUP_NAME);

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

        // delete the dataset
        int status = H5Ldelete(file_->getId(), channel_name.data(), H5P_DEFAULT);
    }

    file_->flush();
}

std::vector<bool> GlobalMap::chunks_empty()
{
    auto chunks = all_chunk_poses();

    std::vector<bool> ret(chunks.size(), false);

    int empty_count = 0;

    for (int i = 0; i < chunks.size(); i++)
    {
        auto chunk = chunks[i];

        auto tag = tag_from_chunk_pos(chunk);

        auto g = file_->getGroup(hdf5_constants::MAP_GROUP_NAME);

        auto ds = g.getDataSet(tag);

        std::vector<TSDFEntry::RawType> data;

        ds.read(data);

        if (data == default_chunk_data)
        {
            ret[i] = true;
            empty_count++;
        }
    }

    std::cout << "Empty count: " << empty_count << std::endl;

    return ret;
}

std::vector<std::pair<Vector3i, TSDFEntry>> GlobalMap::get_full_data()
{
    std::vector<std::pair<Vector3i, TSDFEntry>> ret;

    auto chunks = all_chunk_poses();

    auto group = file_->getGroup(hdf5_constants::MAP_GROUP_NAME);

    int default_counter = 0;

    for (int i = 0; i < chunks.size(); i++)
    {
        auto chunk = chunks[i];

        auto chunk_pos = chunk * CHUNK_SIZE;

        auto tag = tag_from_chunk_pos(chunk);

        auto ds = group.getDataSet(tag);

        std::vector<TSDFEntry::RawType> data;
        ds.read(data);

        // std::cout << "[GlobalMap - get_full_data()] Read " << data.size() << " entries from h5!" << std::endl;

        // now determine all <Vector3i, TSDFEntry> entries

        for (int j = 0; j < data.size(); j++)
        {
            TSDFEntry tmp_tsdf(data[j]);

            // skip default values
            if (tmp_tsdf.value() == params.tau || tmp_tsdf.weight() == 0)
            {
                default_counter++;
                continue;
            }

            // determine x, y and z from index
            Vector3i index_pos = pos_from_index(j);

            // currently pos is a relative pos. to make it absolute, we add the chunk pos;
            index_pos += chunk_pos;

            ret.push_back(std::make_pair(index_pos, tmp_tsdf));
        }

        // std::cout << "Finished reading data from chunk " << i << std::endl;
    }

    std::cout << "[GlobalMap - get_full_data()] Read " << ret.size() << " values from the whole globalmap" << std::endl;
    std::cout << "[GlobalMap - get_full_data()] Read " << default_counter << " default values from the whole globalmap" << std::endl;

    return ret;
}

// parallelized, possibly broken
/**
std::vector<std::pair<Vector3i, TSDFEntry>> GlobalMap::get_full_data()
{

    int thread_count = omp_get_max_threads();

    std::cout << "Num threads: " << thread_count << std::endl;
    std::vector<int> counter_vec(thread_count, 0);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::vector<std::pair<Vector3i, TSDFEntry>> ret_final;
    std::vector<std::vector<std::pair<Vector3i, TSDFEntry>>> ret(thread_count);

    auto chunks = all_chunk_poses();

    auto group = file_->getGroup(hdf5_constants::MAP_GROUP_NAME);

    int default_counter = 0;

#pragma omp parallel for num_threads(thread_count) schedule(static)
    for (int i = 0; i < chunks.size(); i++)
    {
        auto chunk = chunks[i];

        auto chunk_pos = chunk * CHUNK_SIZE;

        auto tag = tag_from_chunk_pos(chunk);

        auto ds = group.getDataSet(tag);

        std::vector<TSDFEntry::RawType> data;
        ds.read(data);

        // std::cout << "[GlobalMap - get_full_data()] Read " << data.size() << " entries from h5!" << std::endl;

        // now determine all <Vector3i, TSDFEntry> entries

// #pragma omp parallel for num_threads(thread_count) schedule(static)
        for (int j = 0; j < data.size(); j++)
        {
            int thread_idx = omp_get_thread_num();
            //std::cout << "thread index: " << thread_idx << std::endl;

            TSDFEntry tmp_tsdf(data[j]);

            // skip default values
            if (tmp_tsdf.value() == params.tau || tmp_tsdf.weight() == 0)
            {
                counter_vec[thread_idx]++;
                continue;
            }

            // determine x, y and z from index
            Vector3i index_pos = pos_from_index(j);

            // currently pos is a relative pos. to make it absolute, we add the chunk pos;
            index_pos += chunk_pos;

            ret[thread_idx].push_back(std::make_pair(index_pos, tmp_tsdf));
        }
    }

    int final_size = 0;

    for (int i = 0; i < ret.size(); i++)
    {
        final_size += ret[i].size();
    }

    ret_final.reserve(final_size);

    // insert the thread stuff
    for (int i = 0; i < ret.size(); i++)
    {
        ret_final.insert(ret_final.end(), ret[i].begin(), ret[i].end());
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "[GlobalMap - get_full_data()] Read " << ret_final.size() << " values from the whole globalmap" << std::endl;
    std::cout << "[GlobalMap - get_full_data()] Read " << default_counter << " default values from the whole globalmap" << std::endl;
    std::cout << "[GlobalMap - get_full_data()] Took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms fo the data aquisition" << std::endl;

    return ret_final;
}
*/

Vector3i GlobalMap::pos_from_index(int i)
{
    int tmp = i;
    // reconstruct pos vector from data index
    int z = i % CHUNK_SIZE;
    i = (i - z) / CHUNK_SIZE;
    int y = i % CHUNK_SIZE;
    i = (i - y) / CHUNK_SIZE;
    int x = i;

    // std::cout << "Index: " << tmp << std::endl
    //           << "Vector: " << std::endl
    //           << Vector3i(x, y, z) << std::endl;

    return Vector3i(x, y, z);
}

void GlobalMap::write_meta(const MapParams &params)
{
    HighFive::Group g = file_->getGroup(hdf5_constants::MAP_GROUP_NAME);

    g.createAttribute("tau", params.tau);
    g.createAttribute("map_size_x", params.size.x());
    g.createAttribute("map_size_y", params.size.y());
    g.createAttribute("map_size_z", params.size.z());
    g.createAttribute("max_distance", params.max_distance);
    g.createAttribute("map_resolution", params.resolution);
    g.createAttribute("max_weight", params.max_weight);

    file_->flush();
}

std::vector<std::pair<std::string, std::vector<std::pair<TSDFEntry::ValueType, TSDFEntry::WeightType>>>> GlobalMap::read_old_format()
{
    struct TSDFValueHWOld
    {
        using ValueType = int16_t;
        using WeightType = int16_t;

        ValueType value;
        WeightType weight;
    };

    union
    {
        uint32_t raw;
        TSDFValueHWOld tsdf;
    } old_data;

    auto chunks = all_chunk_poses();

    auto group = file_->getGroup(hdf5_constants::MAP_GROUP_NAME);

    int default_counter = 0;

    std::vector<std::pair<std::string, std::vector<std::pair<TSDFEntry::ValueType, TSDFEntry::WeightType>>>> converted_data;

    for (int i = 0; i < chunks.size(); i++)
    {
        auto chunk = chunks[i];

        auto chunk_pos = chunk * CHUNK_SIZE;

        auto tag = tag_from_chunk_pos(chunk);

        auto ds = group.getDataSet(tag);

        // old format: 2 * 16 bit
        std::vector<uint32_t> data;
        ds.read(data);

        std::vector<std::pair<TSDFEntry::ValueType, TSDFEntry::WeightType>> chunk_converted;

        for (auto entry : data)
        {
            old_data.raw = entry;

            TSDFEntry new_entry(old_data.tsdf.value, old_data.tsdf.weight);
            chunk_converted.push_back(std::make_pair(new_entry.value(), new_entry.weight()));
        }

        converted_data.push_back(std::make_pair(tag, chunk_converted));

        std::cout << "[GlobalMap - convert_old_format()] Read " << data.size() << " entries from h5!" << std::endl;
    }

    return converted_data;
}

void GlobalMap::write_chunk(std::string tag, std::vector<TSDFEntry::RawType> data)
{
    HighFive::Group g = file_->getGroup(hdf5_constants::MAP_GROUP_NAME);

    if (g.exist(tag))
    {
        // read chunk from file
        HighFive::DataSet d = g.getDataSet(tag);
        d.write(data);
    }
    else
    {
        // create new chunk
        auto d = g.createDataSet(tag, data);
    }
}

void GlobalMap::clean_poses(std::vector<bool> to_be_cleaned)
{
    auto chunks = all_chunk_poses();

    auto group = file_->getGroup(hdf5_constants::MAP_GROUP_NAME);

    for (int i = 0; i < chunks.size(); i++)
    {
        //std::cout << "Cleaning chunk " << i << " of " << chunks.size() << std::endl;
        auto chunk = chunks[i];


        auto chunk_pos = chunk * CHUNK_SIZE;

        auto tag = tag_from_chunk_pos(chunk);

        auto ds = group.getDataSet(tag);

        std::vector<TSDFEntry::RawType> data;
        ds.read(data);

        if(data == default_chunk_data)
        {
            //std::cout << "Default chunk skipped" << std::endl;
            continue;
        }

        for (int j = 0; j < data.size(); j++)
        {
            TSDFEntry tmp_tsdf(data[j]);

            if(tmp_tsdf.pose_index() < 0) continue;

            if(to_be_cleaned[tmp_tsdf.pose_index()])
            {
                tmp_tsdf.value(params.tau);
                tmp_tsdf.weight(0);
                tmp_tsdf.intersect(0);
                tmp_tsdf.pose_index(-1);
            }
            
            data[j] = tmp_tsdf.raw();
        }

        ds.write(data);

        // std::cout << "Finished reading data from chunk " << i << std::endl;
    }

    active_chunks_.clear();
}
