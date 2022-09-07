#include <loop_closure/data_association/association_manager.h>

/**
 * @file association_manager.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief Class used to manage (e.g. load and unload associations)
 *        Furthermore, this class is used to update the localmap after a loop closure, based on
 *        calculated association
 * @version 0.1
 * @date 2022-03-14
 *
 * @copyright Copyright (c) 2022
 *
 */

AssociationManager::AssociationManager(Path *path, std::string file_path, RayTracer *tracer, std::shared_ptr<LocalMap> local_map_ptr_,
                                       std::shared_ptr<GlobalMap> global_map_ptr_, Association::SerializationStrategy strat) : base_path(file_path)
{
    this->path = path;
    auto poses = path->getPoses();
    ray_tracer = tracer;
    local_map_ptr = local_map_ptr_;
    global_map_ptr = global_map_ptr_;

    l_map_size = local_map_ptr->get_size();
    l_map_size_half = l_map_size / 2;

    default_entry = TSDFEntry(global_map_ptr->get_attribute_data().get_tau(), 0); // default tsdf entry used to "reset" old cell locations

    // create marker.
    level_two_marker.header.frame_id = "map";
    level_two_marker.header.stamp = ros::Time();
    level_two_marker.ns = "tsdf";
    level_two_marker.id = 0;
    level_two_marker.lifetime.fromSec(1000);
    level_two_marker.type = visualization_msgs::Marker::POINTS;
    level_two_marker.action = visualization_msgs::Marker::ADD;
    level_two_marker.scale.x = level_two_marker.scale.y = MAP_RESOLUTION * 1.0 * 0.001; // 1.0 is the relative size of the marker

    // markers are same
    level_three_marker = level_two_marker;

    // only create this, if we use json as a serialzation, which we dont
    if (strat == Association::SerializationStrategy::JSON)
    {
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
    }

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

/*********************************************
 * PRIVATE METHODS                           *
 *                                           *
 * In execution order:                       *
 * *******************************************/

void AssociationManager::create_serialization_folder(std::string path)
{
    // create a folder inside the given filepath, for the current timestamp
    std::cout << "[AssociationManager] Creating new association directiory: " << path << std::endl;

    // std::filesystem::create_directories(path);
}

void AssociationManager::calculate_pose_differences(Path *new_path)
{
    // new path has the same size as the old one, currently an association is generated for each of the poses
    for (int i = 0; i < new_path->get_length(); i++)
    {
        Matrix4f previous_pose = associations[i].getPose()->getTransformationMatrix();
        Matrix4f current_pose = new_path->at(i)->getTransformationMatrix();

        // calculate the difference between the transformations via a transformation matrix
        pose_differences.push_back(getTransformationMatrixDiff(previous_pose, current_pose));
    }
}

void AssociationManager::generate_level_one_data(int start_idx, int end_idx)
{
    for (int i = start_idx; i < end_idx; i++)
    {
        // get association for current index
        auto &association = associations[i];

        // deserialize the data
        association.deserialze();

        // obtain the data
        auto &association_data = association.getAssociations();

        // for every pose of the loop, transform the associated data
        for (auto pair : association_data)
        {
            auto hash = pair.first;

            auto data = pair.second;

            // now transform the data according to the pose difference
            auto old_cell = data.first;
            auto tsdf = data.second;

            auto new_cell = calculate_new_cell_position(old_cell, pose_differences[i], associations[i].getPose());

            auto cell_diff = new_cell - old_cell;

            if (level_one_data.find(hash) == level_one_data.end())
            {
                // in this case, there has not been an entry for the old cell in the hashmap, so we generate it
                // the counter is 1 here, as at this time only one association has been handled for the old cell
                level_one_data[hash] = std::make_tuple(old_cell, new_cell, tsdf, 1);
            }
            else
            {
                // in this case, there is already an entry for the old cell in the hashmap, so we need to update it.
                // the new cells are added up, and the counter is increased to later calculate the mean cell
                auto &entry = level_one_data[hash];
                std::get<1>(entry) += new_cell;
                std::get<3>(entry) += 1;
            }
        }

        // after finishing filling the hashmap with the association data of a pose, it is removed from the RAM
        association.clear_data();
    }
}

void AssociationManager::generate_level_two_data(UpdateMethod method)
{
    // THE METHOD IS CURRENTLY IGNORED

    // transform level one data into level two data
    for (auto pair : level_one_data)
    {
        // calculate the mean new cell for every data tuple
        auto data_tuple = pair.second;
        Vector3i new_cell_avg = std::get<1>(data_tuple) / std::get<3>(data_tuple);
        Vector3i old_cell = std::get<0>(data_tuple);
        auto tsdf = std::get<2>(data_tuple);

        auto cell_diff = new_cell_avg - old_cell;

        level_two_marker.points.push_back(type_transform::eigen_point_to_ros_point(map_to_real(new_cell_avg)));
        level_two_marker.colors.push_back(Colors::color_from_name(Colors::ColorNames::navy));

// #ifdef DEBUG
//         if (cell_diff.x() != cell_diff.y() || cell_diff.y() != cell_diff.z() || cell_diff.x() != cell_diff.z() || cell_diff.x() != 46)
//         {
//             std::cout << "Cell diff after meaning: " << std::endl
//                       << cell_diff << std::endl
//                       << "Old:" << std::endl
//                       << old_cell << std::endl
//                       << "New:" << std::endl
//                       << new_cell_avg << std::endl;
//         }

// #endif

        size_t old_hash = pair.first;
        size_t new_hash = hash_from_vec(new_cell_avg);

        // fill level two data

        // old cell data needs to be resettet, this is done by setting it to the default entry
        // in the localmap. This information is also filled into the level two data,
        // but only if it is not overwritten by new cell tsdf data
        if (level_two_data.find(old_hash) == level_two_data.end())
        {
            // in this case, there is no entry for the old cell in the level two data, so we create it with default entry
            level_two_data[old_hash] = std::make_tuple(old_cell, default_entry, 1);
        }
        else
        {
            // in this case, there is already an entry, so in any case we dont to anything here
        }

        // do the same for the new cell, except in this case, we need to consider an update rule for the tsdf data
        // for testing purposes and also because it was stated in some paper, we also sum up the tsdf and weights and calculate a mean
        if (level_two_data.find(new_hash) == level_two_data.end())
        {
            // in this case, there is no entry for the old cell in the level two data, so we create it with default entry
            level_two_data[new_hash] = std::make_tuple(new_cell_avg, tsdf, 1);
        }
        else
        {
            // in this case, there is already an entry, so we update it using the update rule stated above
            auto cur_tsdf = std::get<1>(level_two_data[new_hash]);

            // update
            TSDFEntry updated_entry(cur_tsdf.value() + tsdf.value(), cur_tsdf.weight() + tsdf.weight());
            int updated_counter = std::get<2>(level_two_data[new_hash]);
            updated_counter++;

            level_two_data[new_hash] = std::make_tuple(new_cell_avg, updated_entry, updated_counter);
        }
    }
}

std::pair<Vector3i, Vector3i> AssociationManager::calculate_level_three_bounding_box()
{
    // get bounding box info
    float numeric_max = std::numeric_limits<float>::max();
    int bb_min_x = numeric_max;
    int bb_min_y = numeric_max;
    int bb_min_z = numeric_max;
    int bb_max_x = -numeric_max;
    int bb_max_y = -numeric_max;
    int bb_max_z = -numeric_max;

    // we need bounding box information to ensure minimum number of shifts when updating the localmap
    // TODO: this can probably also be done in the upper loop, not sure about the offset because of the if checks though, as they have to be done
    // a tone more often
    for (auto &pair : level_two_data)
    {
        auto tuple = pair.second;
        Vector3i cell = std::get<0>(tuple);

        // now using the coordinates of old and new cell, aim to finding the bounding box covering all the association cells
        if (cell.x() < bb_min_x)
        {
            bb_min_x = cell.x();
        }
        else if (cell.x() > bb_max_x)
        {
            bb_max_x = cell.x();
        }

        if (cell.y() < bb_min_y)
        {
            bb_min_y = cell.y();
        }
        else if (cell.y() > bb_max_y)
        {
            bb_max_y = cell.y();
        }

        if (cell.z() < bb_min_z)
        {
            bb_min_z = cell.z();
        }
        else if (cell.z() > bb_max_z)
        {
            bb_max_z = cell.z();
        }
    }

    // min and max vector of the bounding box
    Vector3i bb_min(bb_min_x, bb_min_y, bb_min_z);
    Vector3i bb_max(bb_max_x, bb_max_y, bb_max_z);

    return std::make_pair(bb_min, bb_max);
}

void AssociationManager::prepare_level_three_data(Vector3i bb_min, Vector3i bb_max)
{
    Vector3i bb_diff = (bb_max - bb_min).cwiseAbs();

    // now calculate the space seperation of the bounding box

    // if the bounding box of the space occupied by the associations is smaller than the localmap, we simply create one entry in
    // the seperation vector
    if (bb_diff.x() < l_map_size.x() && bb_diff.y() < l_map_size.y() && bb_diff.z() < l_map_size.z())
    {
        // center is exactly in the center of the bounding box
        Vector3i center = bb_min + (bb_diff) / 2;

        auto entry = std::make_pair(center, std::vector<std::pair<Vector3i, TSDFEntry>>());
        level_three_data.push_back(entry);

        return;
    }

    // if the bounding box is bigger than the localmap, we actually need to seperate the space in order to get at least shifts as possible

    // calc the number of lmaps in each direction for the bb
    int x_iterations = std::ceil(bb_diff.x() / (float)l_map_size.x());
    int y_iterations = std::ceil(bb_diff.y() / (float)l_map_size.y());
    int z_iterations = std::ceil(bb_diff.z() / (float)l_map_size.z());

#ifdef DEBUG
    std::cout << "There are " << x_iterations * y_iterations * z_iterations << " Localmaps which need to be considered!" << std::endl;
#endif

    // iterate over 3d
    for (int x = 0; x < x_iterations; x++)
    {
        for (int y = 0; y < y_iterations; y++)
        {
            for (int z = 0; z < z_iterations; z++)
            {
                Vector3i sub_vec = Vector3i(x * l_map_size.x(), y * l_map_size.y(), z * l_map_size.z()) + l_map_size_half;
                Vector3i center = bb_max - sub_vec;

                auto entry = std::make_pair(center, std::vector<std::pair<Vector3i, TSDFEntry>>());
                level_three_data.push_back(entry);

#ifdef DEBUG
                std::cout << "artificial localmap center for: [" << x << ", " << y << ", " << z << "] :" << std::endl
                          << center << std::endl;
#endif
            }
        }
    }
}

void AssociationManager::generate_level_three_data()
{
    // now that the shift locations are calculated, we need to assign the cells to each "shift location"

    int too_high_counter = 0;
    int too_low_counter = 0;

    for (auto pair : level_two_data)
    {
        // captures, which of the seperations is the closest
        int closest_idx = -1;

        Vector3i cell = std::get<0>(pair.second);
        TSDFEntry tsdf = std::get<1>(pair.second);

        int counter = std::get<2>(pair.second);

        tsdf.value(tsdf.value() / counter);
        tsdf.weight(tsdf.weight() / counter);

        if (tsdf.value() < 0)
        {
            if (tsdf.value() < -600)
            {
                too_low_counter++;
            }

            level_three_marker.points.push_back(type_transform::eigen_point_to_ros_point(map_to_real(cell)));

            level_three_marker.colors.push_back(Colors::color_from_name(Colors::ColorNames::red));
        }
        else
        {
            if (tsdf.value() > 600)
            {
                too_high_counter++;
            }
            else if (tsdf.value() < 600)
            {
                level_three_marker.points.push_back(type_transform::eigen_point_to_ros_point(map_to_real(cell)));

                level_three_marker.colors.push_back(Colors::color_from_name(Colors::ColorNames::green));
            }
        }

        // now run over each of the map seperations and determine which pseudo localmap the new and old cell belong to
        for (int i = 0; i < level_three_data.size(); i++)
        {
            Eigen::Vector3i tmp = (cell - level_three_data[i].first).cwiseAbs();

            // if the cell is inbounds, just skip all the other seperations
            if (tmp.x() <= l_map_size_half.x() && tmp.y() <= l_map_size_half.y() && tmp.z() <= l_map_size_half.z())
            {
                closest_idx = i;
            }
        }

        // add cells to the seperations
        level_three_data[closest_idx].second.push_back(std::make_pair(cell, tsdf));
    }

    std::cout << "NUMBER OF TOO HIGH TSDF VALUES: " << too_high_counter << std::endl;
    std::cout << "NUMBER OF TOO LOW TSDF VALUES: " << too_high_counter << std::endl;
}

void AssociationManager::update_localmap_level_three()
{
    for (auto seperation : level_three_data)
    {
        // skip empty seperations
        if (seperation.second.size() == 0)
        {
            continue;
        }

        // shift the local map into the right direction
        local_map_ptr->shift(seperation.first);

        // update the localmap by applying the level three data
        for (auto pair : seperation.second)
        {
            // simply overwrite the existing data
            local_map_ptr->value(pair.first) = pair.second;
        }
    }
}

Vector3i AssociationManager::calculate_new_cell_position(Vector3i &old_cell, Eigen::Matrix4f &transform, Pose *old_pose)
{
    Vector3i new_cell;
    Vector3f old_cell_real = map_to_real(old_cell);

    // extract rotation and translation matrix from the pose difference
    // the start index needs to be substacted here, which is important, as we want the correct pose transform
    Eigen::Matrix3f rot_mat = transform.block<3, 3>(0, 0);
    Vector3f transl_vec = transform.block<3, 1>(0, 3);

    // transform vector by difference between old pose and new pose
    Vector3f transformed_3d = rot_mat * (old_cell_real - old_pose->pos) + old_pose->pos + transl_vec;
    new_cell = real_to_map(transformed_3d);
    Vector3f new_cell_f = real_to_map_float(transformed_3d);

    return new_cell;
}

/*********************************************
 * PUBLIC METHODS                            *
 * *******************************************/

void AssociationManager::greedy_associations()
{
    // go backwards through the path and greedily add all the cells, that are visible.

    /**
     * @todo this is currently done for all poses, this approach will take a lot more time
     *       than necessary, as associations, which are not needed, are being generated..
     */

    int total_associations = 0;

    for (int i = associations.size() - 1; i >= 0; i--)
    {
        std::cout << std::endl
                  << "[AssociationManager]: Starting association estimation for pose: "
                  << i + 1 << " of " << associations.size() << std::endl
                  << std::endl;

        // update ray tracer data for the next trace
        ray_tracer->update_pose(associations[i].getPose());
        ray_tracer->update_association(&associations[i]);
        // ray_tracer->start_bresenham(); // start tracing using bresenham, given the current association.
        ray_tracer->start(); // start tracing, given the current association.

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

#ifdef DEBUG
        // add up number of associations
        total_associations += associations[i].getAssociations().size();
#endif

        associations[i].serialize(); // serialize data

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::cout << "[AssociationManager]: Time used for serialization: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    }
#ifdef DEBUG
    std::cout << std::endl
              << "[AssociationManager]: Total number of associations serialized: " << total_associations << std::endl
              << std::endl;
#endif
}

void AssociationManager::plane_limited_associations()
{
}

visualization_msgs::Marker AssociationManager::update_localmap(Path *new_path, int start_idx, int end_idx, UpdateMethod method)
{
    calculate_pose_differences(new_path);

    std::cout << "There are " << pose_differences.size() << " pose differences, that have been calculated" << std::endl;

    generate_level_one_data(start_idx, end_idx);

    std::cout << "In the level one data, there are " << level_one_data.size() << " entries" << std::endl;

    // ros marker which will be returned, for debugging
    auto marker = ROSViewhelper::init_TSDF_marker_from_hashmap(level_one_data);

    generate_level_two_data(method);

    std::cout << "In the level two data, there are " << level_two_data.size() << " entries" << std::endl;

    auto bb_pair = calculate_level_three_bounding_box();

    prepare_level_three_data(bb_pair.first, bb_pair.second);

    generate_level_three_data();

#ifdef DEBUG
    int counter = 0;
    for (auto sep : level_three_data)
    {
        counter += sep.second.size();
    }

    std::cout << "In the level three data, there are " << counter << " entries" << std::endl;

#endif

    // now update the localmap :)
    update_localmap_level_three();

    // return marker;
    // return level_two_marker;
    return level_three_marker;
}

void AssociationManager::test_associations()
{
    // create a cell map

    for (int i = 0; i < associations.size(); i++)
    {
        auto a = associations[i];

        // read data from file
        a.deserialze();

        // get current association map
        auto &cur_associations = a.getAssociations();

        std::cout << "Deserialized " << cur_associations.size() << " associations" << std::endl;

        // now iterate over the deserialzed association data
        for (auto data : cur_associations)
        {
            size_t hash = data.first;
            auto association_data = data.second;

            // add association stuff.
            if (level_two_data.find(hash) == level_two_data.end())
            {
                level_two_data[hash] = std::make_tuple(association_data.first, default_entry, 1);
            }
        }
    }

    std::cout << "When testing, there are " << level_two_data.size() << " cells in the hm" << std::endl;

    auto bb_pair = calculate_level_three_bounding_box();

    int max_value = std::numeric_limits<int>::max();
    int min_value = std::numeric_limits<int>::min();
    Vector3i default_max = Vector3i::Ones() * max_value;
    Vector3i default_min = Vector3i::Ones() * -1 * min_value;

#ifdef DEBUG
    if (bb_pair.first == default_min || bb_pair.second == default_max)
    {
        throw std::logic_error("[AssociationManager - test_associations() ]:The Bounding Box calculation returned default values");
    }
#endif

    // calc the seperations
    prepare_level_three_data(bb_pair.first, bb_pair.second);

    // fill the seperatoions
    generate_level_three_data();

    // now run over the seperations, shift the map and update the cells (void them as a test)

    for (auto seperation : level_three_data)
    {
        auto shift_location = seperation.first;

        local_map_ptr->shift(shift_location);

        for (auto new_cell : seperation.second)
        {
            auto cell = new_cell.first;
            local_map_ptr->value(cell) = new_cell.second;
        }

        local_map_ptr->write_back();
    }
}

void AssociationManager::cleanup()
{
    // cleanup hdf5
    global_map_ptr->clear_association_data();
    global_map_ptr->clear_intersection_data();

    // cleanup hashmaps and vectors
    level_one_data.clear();
    level_two_data.clear();
    level_three_data.clear();
    pose_differences.clear();

    level_two_marker.points.clear();
    level_two_marker.colors.clear();
}