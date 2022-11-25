/**
 * @file test_mode_update.cpp
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief Testing the map update for the loop closure
 * @version 0.1
 * @date 2022-03-14
 *
 * @copyright Copyright (c) 2022
 *
 */

// ros related includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

// c++ standard related includes
#include <sstream>
#include <iostream>

// external bibs
#include <highfive/H5File.hpp>

#include <loop_closure/params/loop_closure_params.h>

#include <loop_closure/map/global_map.h>
#include <loop_closure/map/local_map.h>
#include <loop_closure/util/colors.h>
#include <loop_closure/path/path.h>
#include <loop_closure/util/point.h>
#include <loop_closure/ray_tracer/ray_tracer.h>
#include <loop_closure/serialization/read_path_json.h>
#include <loop_closure/data_association/association_manager.h>
#include <loop_closure/visualization/ros_viewhelper.h>
#include <loop_closure/path/path_exploration.h>
#include <loop_closure/test/math_test.h>


LoopClosureParams params;

// ROS STUFF //
visualization_msgs::Marker path_marker;
visualization_msgs::Marker tsdf_map;

/// Map Stuff ///
std::shared_ptr<GlobalMap> global_map_ptr_;
std::shared_ptr<LocalMap> local_map_ptr_;

/// Ray Tracer ///
RayTracer *ray_tracer;

/// Path ///
Path *path;

/**
 * @brief initializes the local and global map
 *
 */
void initMaps()
{
    global_map_ptr_ = std::make_shared<GlobalMap>(params.map.filename.string()); // create a global map, use it's attributes
    auto attribute_data = global_map_ptr_->get_attribute_data();

    // local_map_ptr_ = std::make_shared<LocalMap>(312.5, 312.5, 312.5, global_map_ptr_, true); // not used anymore, though good 2 know

    local_map_ptr_ = std::make_shared<LocalMap>(attribute_data.get_map_size_x(),
                                                attribute_data.get_map_size_y(),
                                                attribute_data.get_map_size_z(),
                                                global_map_ptr_, true); // still hardcoded af

    std::cout << "Finished init the maps" << std::endl;
}

/**
 * @brief testing what happens to the map, if we add a default value for all the cells of the globalmap via the localmap
 *
 */
void test_localmap_void()
{
    // create a default entry
    TSDFEntry default_entry(global_map_ptr_->get_attribute_data().get_tau(), 0);
    default_entry.intersect(static_cast<TSDFEntryHW::IntersectionType>(TSDFEntry::IntersectStatus::INT_ZERO));

    float percent = 100.0f / path->get_length();
    float percent_counter = 0.0f;

    std::cout << std::endl;

    for (auto pose : path->getPoses())
    {

        // shift to the pos
        local_map_ptr_->shift(real_to_map(pose.pos));

        // now iterate over 3d space
        Vector3i current_pose = local_map_ptr_->get_pos();
        Vector3i l_size = local_map_ptr_->get_size();
        Vector3i l_size_half = (l_size - Vector3i::Ones()) / 2;
        Vector3i bb_min = current_pose - l_size_half;
        Vector3i bb_max = current_pose + l_size_half;

        for (int x = bb_min.x(); x < bb_max.x(); x++)
        {
            for (int y = bb_min.y(); y < bb_max.y(); y++)
            {
                for (int z = bb_min.z(); z < bb_max.z(); z++)
                {
                    // default the value
                    local_map_ptr_->value(x, y, z) = default_entry;
                }
            }
        }

        // start deleting lines only, when the first one has been printed
        if (percent_counter != 0.0f)
        {
            std::cout << "\033[A\33[2K\r";
        }

        percent_counter += percent;

        std::cout << std::fixed;
        std::cout << std::setprecision(2);
        std::cout << "[TestLocalmap]: " << percent_counter << " \% done filling localmap with void" << std::endl;
    }

    std::cout << std::endl;

    local_map_ptr_->write_back();
}

/**
 * @brief tests, what happens, when the intersection status gets set for all cells
 *
 */
void test_localmap_intersect()
{
    // create a default entry

    float percent = 100.0f / path->get_length();
    float percent_counter = 0.0f;

    std::cout << std::endl;

    for (auto pose : path->getPoses())
    {

        // shift to the pos
        local_map_ptr_->shift(real_to_map(pose.pos));

        // now iterate over 3d space
        Vector3i current_pose = local_map_ptr_->get_pos();
        Vector3i l_size = local_map_ptr_->get_size();
        Vector3i l_size_half = (l_size - Vector3i::Ones()) / 2;
        Vector3i bb_min = current_pose - l_size_half;
        Vector3i bb_max = current_pose + l_size_half;

        for (int x = bb_min.x(); x < bb_max.x(); x++)
        {
            for (int y = bb_min.y(); y < bb_max.y(); y++)
            {
                for (int z = bb_min.z(); z < bb_max.z(); z++)
                {
                    // default the value
                    auto &tsdf = local_map_ptr_->value(x, y, z);
                    tsdf.intersect(static_cast<TSDFEntryHW::IntersectionType>(TSDFEntry::IntersectStatus::INT_ZERO));
                }
            }
        }

        // start deleting lines only, when the first one has been printed
        if (percent_counter != 0.0f)
        {
            std::cout << "\033[A\33[2K\r";
        }

        percent_counter += percent;

        std::cout << std::fixed;
        std::cout << std::setprecision(2);
        std::cout << "[TestLocalmap]: " << percent_counter << " \% done updating intersect status" << std::endl;
    }

    std::cout << std::endl;

    local_map_ptr_->write_back();
}

/**
 * @brief Main method
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "loop_closure_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // read options from cmdline
    params = LoopClosureParams(nh);

    // init local and global maps
    initMaps();

    // define stuff for raytracer
    ray_tracer = new RayTracer(params, local_map_ptr_, global_map_ptr_);

    /******************************************************
     *  Get Path                                          *
     ******************************************************/

    // init path and read from json
    path = new Path();
    path->attach_raytracer(ray_tracer);

    // retrieve path from global map
    try
    {
        auto path_poses = global_map_ptr_->get_path();

        // copy the read poses to the path
        path->getPoses() = path_poses;

        if (path->get_length() == 0)
        {
            throw std::invalid_argument("There are no poses in the path, hence the arguments listed seem to be wrong. Check them.");
        }
    }
    catch (std::exception &ex)
    {
        std::cerr << "[LoopClosureNode] Error when defining the path: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }

    /******************************************************
     *  End Path Creation                                 *
     ******************************************************/

    //test_localmap_void();
    test_localmap_intersect();

    // generate publishers
    ros::Publisher path_publisher = n.advertise<visualization_msgs::Marker>("path", 1, true);
    ros::Publisher map_publisher = n.advertise<visualization_msgs::Marker>("tsdf_map", 1, true);

    tsdf_map = ROSViewhelper::initTSDFmarkerPath(local_map_ptr_, path, true);

    if (tsdf_map.points.size() > 0)
    {
        std::cout << "There are still " << tsdf_map.points.size() << " cells, which have not been reset" << std::endl;
    }
    else
    {
        std::cout << "Where there are no points, there is nothing to be afraid off. Test SUCCESSFUL!" << std::endl;
    }

    path_marker = ROSViewhelper::initPathMarker(path);

    // publish it all
    path_publisher.publish(path_marker);
    map_publisher.publish(tsdf_map);

    // specify ros loop rate
    ros::Rate loop_rate(10);

    ros::spinOnce();

    // ros loop
    while (ros::ok())
    {
        // publish the individual messages
        path_publisher.publish(path_marker);

        // more ros related stuff
        ros::spinOnce();

        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}