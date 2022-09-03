#include <loop_closure/data_association/association_manager.h>

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
                                       std::shared_ptr<GlobalMap> global_map_ptr_, Association::SerializationStrategy strat) : base_path(file_path)
{
    this->path = path;
    auto poses = path->getPoses();
    ray_tracer = tracer;
    local_map_ptr = local_map_ptr_;
    global_map_ptr = global_map_ptr_;

    l_map_size = local_map_ptr->get_size();
    l_map_size_half = l_map_size / 2;

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
 * *******************************************/

void AssociationManager::create_serialization_folder(std::string path)
{
    // create a folder inside the given filepath, for the current timestamp
    std::cout << "[AssociationManager] Creating new association directiory: " << path << std::endl;

    // std::filesystem::create_directories(path);
}

std::vector<Matrix4f> AssociationManager::calculate_pose_differences(Path *new_path, int start_idx, int end_idx)
{
    // vector capturing the differences between poses
    std::vector<Matrix4f> pose_differences;

    for (int i = start_idx; i <= end_idx; i++)
    {
        Matrix4f previous_pose = associations[i].getPose()->getTransformationMatrix();
        Matrix4f current_pose = new_path->at(i)->getTransformationMatrix();

        pose_differences.push_back(getTransformationMatrixDiff(previous_pose, current_pose));

#ifdef DEBUG
        std::cout << std::endl
                  << "Prev: " << std::endl
                  << previous_pose << std::endl
                  << "Current: " << std::endl
                  << current_pose << std::endl
                  << "Diff Comp Wise: " << std::endl
                  << pose_differences[i] << std::endl
                  << std::endl;
#endif
    }

    return pose_differences;
}

void AssociationManager::fill_hashmap(
    boost::unordered_map<size_t, std::tuple<Vector3f, Vector3f, TSDFEntry, int>> &previous_new_map,
    std::vector<Matrix4f> &pose_differences, int start_idx, int end_idx)
{
    for (int i = start_idx; i < end_idx; i++)
    {
        auto a = associations[i];

#ifdef DEBUG
        std::cout << "Getting data for pose " << i << std::endl;

        std::cout << "Deserializing data..." << std::endl;
#endif

        // read data from file
        a.deserialze();

#ifdef DEBUG
        std::cout << "Deserialization done!" << std::endl;
#endif

        // get current association map
        auto &cur_associations = a.getAssociations();

#ifdef DEBUG
        std::cout << "Retrieved " << cur_associations.size() << " associations from hdf5" << std::endl;
#endif

        // now iterate over the deserialzed association data
        for (auto data : cur_associations)
        {
            size_t hash = data.first;
            auto association_data = data.second;

            // get cell position in real word data
            auto vec_real = map_to_real(data.second.first);

            // extract rotation and translation matrix from the pose difference
            // the start index needs to be substacted here, which is important, as we want the correct pose transform
            Eigen::Matrix3f rot_mat = pose_differences[a.get_index() - start_idx].block<3, 3>(0, 0);
            Vector3f transl_vec = pose_differences[a.get_index() - start_idx].block<3, 1>(0, 3);

            // transform vector by difference between old pose and new pose
            Vector3f transformed_3d = rot_mat * (vec_real - associations[i].getPose()->pos) + associations[i].getPose()->pos + transl_vec;

            // tsdf entry for old cell
            TSDFEntry old_tsdf_entry = data.second.second;

            // check if exists in hashmap
            if (previous_new_map.find(hash) == previous_new_map.end())
            {
                // if none yet exists, we just add it
                previous_new_map[hash] = std::make_tuple(vec_real, transformed_3d, old_tsdf_entry, 1);
            }
            else
            {
                // update the values
                // the index counter gets updated
                auto &tuple = previous_new_map[hash];
                std::get<1>(tuple) += transformed_3d;
                std::get<3>(tuple) += 1;

                // std::cout << "Updated some values. New index: " << std::get<3>(tuple) << std::endl;

#ifdef DEBUG
                if (std::get<3>(tuple) > associations.size())
                {
                    std::cout << "Problem for vertex: " << std::endl
                              << std::get<0>(tuple) << std::endl;
                }
#endif
            }
        }

#ifdef DEBUG
        for (auto data : cur_associations)
        {
            if (previous_new_map.find(data.first) == previous_new_map.end())
            {
                std::cout << "KEEEEEEEEEEEEEEEEEEEEEK" << std::endl;
            }
        }
#endif

    // clear data after every association is loaded to ensure RAM is not polluted
    a.clear_data();

    }
}

void AssociationManager::filter_duplicate_tagret_cells(boost::unordered_map<size_t, std::tuple<Vector3f, Vector3f, TSDFEntry, int>> &previous_new_map,
                                                       boost::unordered_map<size_t, std::tuple<Vector3i, TSDFEntry, int>> &new_tsdf_map,
                                                       UpdateMethod &method)
{
    // fill the map from above, considering, that after the update multiple different cells might "fall" on the same cell
    for (auto pair : previous_new_map)
    {
        auto tuple = pair.second;
        Vector3f old_vec = std::get<0>(tuple);
        TSDFEntry tsdf = std::get<2>(tuple);

        Vector3i old_cell = real_to_map(old_vec);
        Vector3i new_cell;

        switch (method)
        {
        case UpdateMethod::MEAN:

            // calc mean using counter
            new_cell = real_to_map(std::get<1>(tuple) / (float)std::get<3>(tuple));

#ifdef DEBUG
            if (old_cell == new_cell)
            {
                std::cout << "OLD AND NEW CELL HAVE THE SAME COORDINATES" << std::endl;
            }
#endif

            break;

        case UpdateMethod::SINUS:
            // this needs to be implemented, when adding to the map, possibly same for the mean
            break;

        default:
            break;
        }

        size_t hash = hash_from_vec(new_cell);
        size_t hash_old = hash_from_vec(old_cell);

        // check if exists. if exists and default value is in place, overwrite
        /*if (new_tsdf_map.find(hash) == new_tsdf_map.end() ||
            (std::get<1>(new_tsdf_map[hash]).weight() == 0 && std::get<1>(new_tsdf_map[hash]).value() == global_map_ptr->get_attribute_data().get_tau()))
        {
            new_tsdf_map[hash] = std::make_tuple(new_cell, tsdf, 1);
        }
        else
        {
            // the cell already exists -> there are multiple updates to the same cell, mean them
            auto &data = new_tsdf_map[hash];
            auto &tsdf_hm = std::get<1>(data);
            auto &counter = std::get<2>(data);
            tsdf_hm.value(tsdf_hm.value() + tsdf.value());
            tsdf_hm.weight(tsdf_hm.weight() + tsdf.weight());
            tsdf_hm.setIntersect(tsdf.getIntersect());
            counter += 1;
        }*/

        // now "reset" old poses with a default entry, still considering, that there may be some new cells also updating this

        // if there is already a value present in the hashmap, we skip the overwriting (as this would verfälschen the values)
        if (new_tsdf_map.find(hash_old) != new_tsdf_map.end())
        {
            continue;
        }
        else
        {
            // no update seen yet, so we place the default entry here, still considering, that it may be updated later on.
            new_tsdf_map[hash_old] = std::make_tuple(old_cell, default_entry, 1);
        }
    }
}

std::pair<Vector3i, Vector3i> AssociationManager::calculate_bounding_box(
    boost::unordered_map<size_t, std::tuple<Vector3i, TSDFEntry, int>> &new_tsdf_map)
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
    for (auto &pair : new_tsdf_map)
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

void AssociationManager::calc_map_seperations(Vector3i bb_min, Vector3i bb_max,
                                              std::vector<std::pair<Vector3i, std::vector<std::pair<Vector3i, TSDFEntry>>>> &map_seperations)
{
    Vector3i bb_diff = (bb_max - bb_min).cwiseAbs();

#ifdef DEBUG
    std::cout << "[AssociationManager - update_localmap()] - Found OLD Bounding Box between: " << std::endl
              << bb_min << std::endl
              << "and" << std::endl
              << bb_max << std::endl
              << "With localmap size: " << std::endl
              << l_map_size << std::endl
              << "With localmaphalf size: " << std::endl
              << l_map_size_half << std::endl;
#endif

    // now calculate the space seperation of the bounding box

    // if the bounding box of the space occupied by the associations is smaller than the localmap, we simply create one entry in
    // the seperation vector
    if (bb_diff.x() < l_map_size.x() && bb_diff.y() < l_map_size.y() && bb_diff.z() < l_map_size.z())
    {
        // center is exactly in the center of the bounding box
        Vector3i center = bb_min + (bb_diff) / 2;

        auto entry = std::make_pair(center, std::vector<std::pair<Vector3i, TSDFEntry>>());
        map_seperations.push_back(entry);
    }
    else
    {
        // in this case, we actually need to seperate the space in order to get at least shifts as possible

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
                    map_seperations.push_back(entry);

#ifdef DEBUG
                    std::cout << "artificial localmap center for: [" << x << ", " << y << ", " << z << "] :" << std::endl
                              << center << std::endl;
#endif
                }
            }
        }
    }
}

void AssociationManager::fill_map_seperations(std::vector<std::pair<Vector3i, std::vector<std::pair<Vector3i, TSDFEntry>>>> &map_seperations,
                                              boost::unordered_map<size_t, std::tuple<Vector3i, TSDFEntry, int>> &new_tsdf_map)
{
    // now that the shift locations are calculated, we need to assign the cells to each "shift location"

    for (auto pair : new_tsdf_map)
    {
        // captures, which of the seperations is the closest
        int closest_idx = -1;

        Vector3i cell = std::get<0>(pair.second);
        TSDFEntry tsdf = std::get<1>(pair.second);
        int counter = std::get<2>(pair.second);

        if (counter > 1)
        {
            tsdf.value(tsdf.value() / counter);
            tsdf.weight(tsdf.weight() / counter);
        }

        // now run over each of the map seperations and determine which pseudo localmap the new and old cell belong to
        for (int i = 0; i < map_seperations.size(); i++)
        {

#ifdef DEBUG
            // TODO: look into this
            /*if (new_cell_map == Vector3i(0, 0, 0) || old_cell == Vector3i(0, 0, 0))
            {
                std::cout << "Vector is null vector " << std::endl
                          << "Previous: (with counter: " << std::get<3>(pair.second) << ")" << std::endl
                          << std::get<1>(pair.second) << std::endl;
            }*/
#endif

            Eigen::Vector3i tmp = (cell - map_seperations[i].first).cwiseAbs();

            // if the cell is inbounds, just skip all the other seperations
            if (tmp.x() <= l_map_size_half.x() && tmp.y() <= l_map_size_half.y() && tmp.z() <= l_map_size_half.z())
            {
                closest_idx = i;
            }
        }

        // add cells to the seperations array

#ifdef DEBUG

        // catch possible bugs, this is the location, where errors are most common
        if (closest_idx == -1)
        {
            std::stringstream ss;
            ss << "[AssociationManager] A cell coulld not be assigned to a seperation group, this is a bug: " << std::endl
               << cell << std::endl;
            throw std::logic_error(ss.str());
        }

#endif

        // add cells to the seperations
        map_seperations[closest_idx].second.push_back(std::make_pair(cell, tsdf));
    }

    // now: run through the seperations and update the individual localmaps
    // whilst checking in the copy, if the value might have already been updated
}

/*********************************************
 * PUBLIC METHODS                            *
 * *******************************************/

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

visualization_msgs::Marker AssociationManager::update_localmap(Path *new_path, int start_idx, int end_idx, UpdateMethod method)
{
#ifdef DEBUG
    std::cout << "[AssociationManager - update_localmap] : Map Update requested between Pose " << start_idx << " and " << end_idx << " !!" << std::endl;
#endif
    /**********************/
    /** Global variables **/
    /**********************/
    default_entry = TSDFEntry(global_map_ptr->get_attribute_data().get_tau(), 0); // default tsdf entry

    std::vector<Matrix4f> pose_differences; // will hold the transformations between each pose of the new and old path

    boost::unordered_map<size_t, std::tuple<Vector3f, Vector3f, TSDFEntry, int>> previous_new_map; // map holding the old and new cell positions

    boost::unordered_map<size_t, std::tuple<Vector3i, TSDFEntry, int>> new_tsdf_map; // a map which will contain a pair of a cell and its new tsdf value ( and an int as a counter)

    std::vector<std::pair<Vector3i, std::vector<std::pair<Vector3i, TSDFEntry>>>> map_seperations; // map holding the map seperations in localmap sized chunks

    /********************************************************************/
    /** 1.) Calc a vector of pose differences between old and new pose **/
    /********************************************************************/

    pose_differences = calculate_pose_differences(new_path, start_idx, end_idx);

    /***********************************************************************************************************/
    /** 2.) now, that we got the relative transformations, we need to calc the new cell positions considering **/
    /**     the update method                                                                                 **/
    /**     RESEARCH (TODO:) should this be initialized with a size?                                          **/
    /**     (might be resized every single time an element is inserted)                                       **/
    /***********************************************************************************************************/

    fill_hashmap(previous_new_map, pose_differences, start_idx, end_idx);

#ifdef DEBUG
    std::cout << "[AssociationManager - update_localmap] : Got " << previous_new_map.size() << " Association cells which will be updated." << std::endl;
#endif

    // ros marker which will be returned, for debugging
    auto marker = ROSViewhelper::init_TSDF_marker_from_hashmap(previous_new_map);

    // filter duplicate destination cells, aka if after the update two cells fall on the same target cell, this should be considered
    filter_duplicate_tagret_cells(previous_new_map, new_tsdf_map, method);

#ifdef DEBUG
    std::cout << "[Associationmanager: After filtering duplicate cell destinations and" << std::endl
              << "adding old cell destinations, the number of updates to the map, which are necessary, is:" << new_tsdf_map.size() << std::endl;
#endif
    // calculate the bounding box of the remaining cell destinations
    auto bounding_box = calculate_bounding_box(new_tsdf_map);

    // the bounding box covered by the association data is split into multiple parts of localmap size with center C (first part of the pair)
    // this ensures, that only a very limited number of shifts is necessary.

    Vector3i bb_min = bounding_box.first;
    Vector3i bb_max = bounding_box.second;

    // calculate the map seperations
    calc_map_seperations(bb_min, bb_max, map_seperations);

#ifdef DEBUG
    std::cout << "[Associationmanager]: Got " << map_seperations.size() << " map seperations for this run!!" << std::endl;
#endif

    // fill the seperations with the data from the associations
    // this ensures a minimum number of localmap shifts, thus reducing the runtime significantly
    fill_map_seperations(map_seperations, new_tsdf_map);

#ifdef DEBUG
    int counter = 0;
    // * 2, as there are updates for new and old cells ;)
    size_t hashmap_size = new_tsdf_map.size();
    size_t five_percent = hashmap_size / 20;
    int percent_counter = 0;
#endif

    for (auto seperation : map_seperations)
    {
        // completely skip empty seperations
        if (seperation.second.size() == 0)
        {

#ifdef DEBUG
            std::cout << "Empty seperation skipped " << std::endl;
#endif

            continue;
        }

        // every seperation has a center vertex, the localmap needs to be shifted there
        Vector3i shift_location = seperation.first;
        local_map_ptr->shift(shift_location);

#ifdef DEBUG
        std::cout << "SHIFT! TO: " << std::endl
                  << local_map_ptr->get_pos() << std::endl;
#endif

        // obtain a copy of the local map to check if a cell has already been updated before
        LocalMap localmap_copy(*local_map_ptr);

        for (auto &pair : seperation.second)
        {

#ifdef DEBUG
            // counter stuff
            counter++;
            if (counter % five_percent == 0)
            {
                percent_counter += 5;
                std::cout << "Updated " << percent_counter << " percent of the map" << std::endl;
            }
#endif
            // update the cells

#ifdef DEBUG
            // if the cell is not inbounds the localmap, there has been an error in the math/indexing
            if (!local_map_ptr->in_bounds(pair.first))
            {
                std::stringstream ss;

                ss << "Every cell should be inbound now...." << std::endl
                   << "Localmap - pos: " << std::endl
                   << local_map_ptr->get_pos() << std::endl
                   << "Cell: " << std::endl
                   << pair.first << std::endl;

                throw std::logic_error(ss.str());
            }
#endif

            auto &tsdf = local_map_ptr->value(pair.first);
            auto &tsdf_copy = localmap_copy.value(pair.first);

            // TODO: use predefined factor here (parameter)

            auto old_val = tsdf.value();
            auto old_weight = tsdf.weight();
            auto new_val = pair.second.value();
            auto new_weight = pair.second.weight();

            // dont do this, use paramter instead. defines how much the new value should be weighted.
            float new_fac = 0.9f;

            TSDFEntryHW::ValueType calced_value;
            TSDFEntryHW::WeightType calced_weight;

            if (old_val == global_map_ptr->get_attribute_data().get_tau() && old_weight == 0)
            {
                // if the cell contains default values, we overwrite them
                calced_value = new_val;
                calced_weight = new_weight;
            }
            else if (tsdf.value() != tsdf_copy.value() || tsdf.weight() != tsdf_copy.weight())
            {
                // two updates to the same cell, this is not yet covered...
                // if the cell contains default values, we overwrite them

                // TODO: dont do this!
                calced_value = (1.0f - new_fac) * old_val + new_fac * new_val;
                calced_weight = (1.0f - new_fac) * old_weight + new_fac * new_weight;
            }
            else
            {
                // else the cell is non zero, but also not yet updated...
                // if the target cell is a default cell, we completely overwrite it
                calced_value = (1.0f - new_fac) * old_val + new_fac * new_val;
                calced_weight = (1.0f - new_fac) * old_weight + new_fac * new_weight;
            }

            tsdf.value(calced_value);
            tsdf.weight(calced_weight);
            tsdf.setIntersect(pair.second.getIntersect());
        }
    }

    local_map_ptr->write_back();

    return marker;
}

void AssociationManager::test_associations()
{
    boost::unordered_map<size_t, std::tuple<Vector3i, TSDFEntry, int>> new_tsdf_map;
    std::vector<std::pair<Vector3i, std::vector<std::pair<Vector3i, TSDFEntry>>>> map_seperations; // map holding the map seperations in localmap sized chunks

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
            if (new_tsdf_map.find(hash) == new_tsdf_map.end())
            {
                new_tsdf_map[hash] = std::make_tuple(association_data.first, default_entry, 1);
            }
        }
    }

    std::cout << "When testing, there are " << new_tsdf_map.size() << " cells in the hm" << std::endl;

    auto bb_pair = calculate_bounding_box(new_tsdf_map);

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
    calc_map_seperations(bb_pair.first, bb_pair.second, map_seperations);

    // fill the seperatoions
    fill_map_seperations(map_seperations, new_tsdf_map);

    // now run over the seperations, shift the map and update the cells (void them as a test)

    for (auto seperation : map_seperations)
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