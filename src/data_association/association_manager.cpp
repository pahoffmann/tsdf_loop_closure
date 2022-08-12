#include "association_manager.h"

/**
 * @file association_manager.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief Class used to manage (e.g. load and unload associations)
 * @version 0.1
 * @date 2022-03-14
 *
 * @copyright Copyright (c) 2022
 *
 */

AssociationManager::AssociationManager(Path *path, std::string file_path, RayTracer *tracer, std::shared_ptr<LocalMap> local_map_ptr_,
                                       std::shared_ptr<GlobalMap> global_map_ptr_) : base_path(file_path)
{
    this->path = path;
    auto poses = path->getPoses();
    ray_tracer = tracer;
    local_map_ptr = local_map_ptr_;
    global_map_ptr = global_map_ptr_;

    // create timestamp
    time = std::time(nullptr);
    std::string time_string(std::asctime(std::localtime(&time)));

    time_string = std::regex_replace(time_string, std::regex(" "), "_");
    time_string = std::regex_replace(time_string, std::regex(":"), "_");

    // somehow there is a line break in this string :D
    time_string = std::regex_replace(time_string, std::regex("\n"), "");

    // check if the file path contains a trailing slash, if not: add it
    if (file_path.at(file_path.length() - 1) != '/')
    {
        file_path += "/";
    }

    file_path += time_string + "/";

    // create serialization folder
    create_serialization_folder(file_path);

    for (int i = 0; i < poses.size(); i++)
    {
        // create new association and add it to the array
        Association association(poses[i], i, global_map_ptr, file_path, Association::SerializationStrategy::HDF5);
        associations.push_back(association);
    }
}

AssociationManager::~AssociationManager()
{
}

void AssociationManager::create_serialization_folder(std::string path)
{
    // create a folder inside the given filepath, for the current timestamp
    std::cout << "[AssociationManager] Creating new association directiory: " << path << std::endl;

    // std::filesystem::create_directories(path);
}

void AssociationManager::greedy_associations()
{
    // go backwards through the path and greedily add all the cells, that are visible.

    for (int i = associations.size() - 1; i >= 0; i--)
    {
        std::cout << std::endl
                  << "[AssociationManager]: Starting association estimation for pose: "
                  << i + 1 << " of " << associations.size() << std::endl
                  << std::endl;

        // configure and start the ray tracer for every iteration, which will fill each association
        // ray_tracer->update_map_pointer(local_map_ptr);

        // update ray tracer data for the next trace
        ray_tracer->update_pose(associations[i].getPose());
        ray_tracer->update_association(&associations[i]);
        ray_tracer->start_bresenham(); // start tracing using bresenham, given the current association.
        // ray_tracer->start(); // start tracing, given the current association.

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        associations[i].serialize(); // serialize data

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::cout << "[AssociationManager]: Time used for serialization: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    }
}

void AssociationManager::plane_limited_associations()
{
}

void AssociationManager::update_localmap(Path *new_path, int start_idx, int end_idx, UpdateMethod method)
{
    // 1.) Calc a vector of pose differences between old and new pose

    std::cout << "[AssociationManager - update_localmap] : Map Update requested between Pose " << start_idx << " and " << end_idx << " !!" << std::endl;

    std::vector<Matrix4f> pose_differences;

    for (int i = start_idx; i <= end_idx; i++)
    {
        Matrix4f previous_pose = associations[i].getPose()->getTransformationMatrix();
        Matrix4f current_pose = new_path->at(i)->getTransformationMatrix();

        std::cout << "test" << std::endl;

        // push pose diff as transformation matrix to vector
        pose_differences.push_back(previous_pose.inverse() * current_pose);

        std::cout << "Prev: " << std::endl
                  << previous_pose << std::endl
                  << "Current: " << std::endl
                  << current_pose << std::endl
                  << "Diff: " << std::endl
                  << pose_differences[i] << std::endl;
    }

    // 2.) now, that we got the relative transformations, we need to calc the new cell positions considering
    //     the update method
    // RESEARCH (TODO:) should this be initialized with a size? (might be resized every single time an element is inserted)
    boost::unordered_map<size_t, std::tuple<Vector3f, Vector3f, int>> previous_new_map;

    for (int i = start_idx; i < end_idx; i++)
    {
        auto a = associations[i];

        std::cout << "Getting data for pose " << i << std::endl;

        std::cout << "Deserializing data..." << std::endl;
        // read data from file
        a.deserialze();

        std::cout << "Deserialization done!" << std::endl;

        // get current association map
        auto &cur_associations = a.getAssociations();

        std::cout << "Retrieved " << cur_associations.size() << " associations from hdf5" << std::endl;

        // now iterate over the deserialzed association data
        for (auto data : cur_associations)
        {
            size_t hash = data.first;
            auto association_data = data.second;

            // get cell position in real word data
            auto vec_real = map_to_real(data.second.first);

            // transform vector by difference between old pose and new pose
            auto pos = new_path->at(i)->pos;
            Eigen::Vector4f pose_vec = Eigen::Vector4f(pos.x(), pos.y(), pos.z(), 1.0f);
            // transformation relative to old pose
            Eigen::Vector4f vec_transformed = pose_differences[a.get_index()] * (Eigen::Vector4f(vec_real.x(), vec_real.y(), vec_real.z(), 1) - pose_vec) + pose_vec;

            // normalize
            vec_transformed.normalize();

            // back to 3d (possibly because of normalization)
            Vector3f transformed_3d = Vector3f(vec_transformed.x(), vec_transformed.y(), vec_transformed.z());

            // check if exists
            if (previous_new_map.find(hash) != previous_new_map.end())
            {
                // if none yet exists, we just do some stuff.
                previous_new_map[hash] = std::make_tuple(vec_real, transformed_3d, 1);
            }
            else
            {
                // update the values
                auto &tuple = previous_new_map[hash];
                std::get<1>(tuple) += transformed_3d;
                std::get<2>(tuple) += 1;
            }
        }
    }

    std::cout << "[AssociationManager - update_localmap] : Got " << previous_new_map.size() << " Association cells which will be updated." << std::endl;

    int counter = 0;
    size_t hashmap_size = previous_new_map.size();
    size_t five_percent = hashmap_size / 20;
    int percent_counter = 0;

    // now iterate over the hashmap and update the map
    for (auto &pair : previous_new_map)
    {
        counter++;
        if (counter % five_percent == 0)
        {
            percent_counter += 5;
            std::cout << "Updated " << percent_counter << " percent of the map" << std::endl;
        }

        // temp for the new cell
        Vector3f new_cell;

        // we also need the old cell for a partial update
        Vector3i old_cell = real_to_map(std::get<0>(pair.second));

        switch (method)
        {
        case UpdateMethod::MEAN:

            // calc mean using counter
            new_cell = std::get<1>(pair.second) / std::get<2>(pair.second);

            break;

        case UpdateMethod::SINUS:
            // this needs to be implemented, when adding to the map, possibly same for the mean
            break;

        default:
            break;
        }

        Vector3i new_cell_map = real_to_map(new_cell);

        // here is the problem: we try to get the value of a cell, which is out of bounds.
        // --> TODO: fix this! where and when to shift the local map? this needs to be done very cautious!
        auto &old_tsdf = local_map_ptr->value(old_cell);
        auto &new_tsdf = local_map_ptr->value(new_cell_map);

        // TODO: use predefined factor here (parameter)

        auto old_val = old_tsdf.value();
        auto old_weight = old_tsdf.weight();
        auto new_val = new_tsdf.value();
        auto new_weight = new_tsdf.weight();

        // dont do this, use paramter instead. defines how much the new value should be weighted.
        float new_fac = 0.9f;

        TSDFEntryHW::ValueType calced_value;
        TSDFEntryHW::WeightType calced_weight;

        if (old_val != global_map_ptr->get_attribute_data().get_tau() || old_weight != 0)
        {
            // if the target cell already contains data, we fuse the values, defined by a parameter
            calced_value = (1.0f - new_fac) * old_val + new_fac * new_val;
            calced_weight = (1.0f - new_fac) * old_weight + new_fac * new_weight;
        }
        else
        {
            // if the cell contains default values, we overwrite them
            calced_value = new_val;
            calced_weight = new_weight;
        }

        old_tsdf.value(calced_value);
        old_tsdf.weight(calced_weight);
    }

    return;
}
