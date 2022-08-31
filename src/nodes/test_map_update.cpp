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

// relative includes
#include <loop_closure/map/global_map.h>
#include <loop_closure/map/local_map.h>
#include <loop_closure/util/colors.h>
#include <loop_closure/path/path.h>
#include <loop_closure/util/point.h>
#include <loop_closure/ray_tracer/ray_tracer.h>
#include <loop_closure/serialization/read_path_json.h>
#include <loop_closure/data_association/association_manager.h>
#include <loop_closure/options/options_reader.h>
#include <loop_closure/visualization/ros_viewhelper.h>
#include <loop_closure/path/path_exploration.h>
#include <loop_closure/test/math_test.h>

// Configuration stuff //
lc_options_reader *options;

// ROS STUFF //
visualization_msgs::Marker before_path;
visualization_msgs::Marker after_path;

/// Map Stuff ///
std::shared_ptr<GlobalMap> global_map_ptr_;
std::shared_ptr<LocalMap> local_map_ptr_;

/// Ray Tracer ///
RayTracer *ray_tracer;

/// Path ///
Path *path;

/// Association Manager, used to manage and keep track of the associations ///
AssociationManager *manager;

/**
 * @brief initializes the local and global map
 *
 */
void initMaps()
{
    global_map_ptr_ = std::make_shared<GlobalMap>(options->get_map_file_name()); // create a global map, use it's attributes
    auto attribute_data = global_map_ptr_->get_attribute_data();

    // local_map_ptr_ = std::make_shared<LocalMap>(312.5, 312.5, 312.5, global_map_ptr_, true); // not used anymore, though good 2 know

    local_map_ptr_ = std::make_shared<LocalMap>(attribute_data.get_map_size_x(),
                                                attribute_data.get_map_size_y(),
                                                attribute_data.get_map_size_z(),
                                                global_map_ptr_, true); // still hardcoded af

    std::cout << "Finished init the maps" << std::endl;
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
    options = new lc_options_reader();
    int status = options->read_options(argc, argv);

    if (status == 1)
    {
        std::cout << "[CLI] Terminate node, because there were some errors while reading from cmd-line" << std::endl;

        return 1;
    }
    else if (status == 2)
    {
        std::cout << "[CLI] Terminate node, because help was requested" << std::endl;

        return 0;
    }

    // retrieve path method from options (0 = from globalmap, 1 = from path extraction, 2 = from json)
    int path_method = options->get_path_method();

    // init local and global maps
    initMaps();

    // cleans up global map a bit
    // global_map_ptr_->cleanup_artifacts();

    // define stuff for raytracer
    ray_tracer = new RayTracer(options, local_map_ptr_, global_map_ptr_);

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

        for (auto pose : path_poses)
        {
            path->add_pose(pose);
        }

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

    // generate publishers
    ros::Publisher path_before_publisher = n.advertise<visualization_msgs::Marker>("path_before", 1, true);
    ros::Publisher path_after_publisher = n.advertise<visualization_msgs::Marker>("path_after", 1, true);
    ros::Publisher wall_before_publisher = n.advertise<visualization_msgs::Marker>("wall_before", 1, true);
    ros::Publisher wall_after_publisher = n.advertise<visualization_msgs::Marker>("wall_after", 1, true);

    // rotate the path for testing
    // Path rotated_path = path->rotate_ret(90, 0, 0);
    Path translated_path = path->translate_ret(Vector3f(2, 2, 2), 0, path->get_length() - 1);
    translated_path = translated_path.rotate_ret(0, 45, 90);

    before_path = ROSViewhelper::initPathMarker(path);
    // after_path = ROSViewhelper::initPathMarker(&rotated_path);
    after_path = ROSViewhelper::initPathMarker(&translated_path);

    Vector3i wall_base(-100, -50, -100);
    Vector3i dir_vec_1(3, 0, 0);
    Vector3i dir_vec_2(0, 3, 0);
    Vector3i dir_vec_3(0, 0, 3);
    std::vector<Vector3i> cell_wall = Testing::generate_cell_wall(wall_base, dir_vec_1, dir_vec_2, 50, 100);
    std::vector<Vector3i> cell_cube = Testing::generate_cell_cube(wall_base, dir_vec_1, dir_vec_2, dir_vec_3, 50, 100, 20);
    // auto vis_pair = Testing::test_cell_transformation(cell_wall, path, &rotated_path);
    // auto vis_pair = Testing::test_cell_transformation(cell_wall, path, &rotated_path, 0, 0);
    // auto vis_pair = Testing::test_cell_transformation(cell_wall, path, &translated_path, 0, 0);
    auto vis_pair = Testing::test_cell_transformation(cell_cube, path, &translated_path);
    // auto vis_pair = Testing::test_cell_transformation_rnd(cell_cube, path, &translated_path);
    // auto vis_pair = Testing::test_cell_transformation_rnd(cell_cube, path, &rotated_path);
    // auto vis_pair = Testing::test_cell_transformation_weighted(cell_cube, path, &rotated_path);

    // publish it all
    path_before_publisher.publish(before_path);
    path_after_publisher.publish(after_path);
    wall_before_publisher.publish(vis_pair.first);
    wall_after_publisher.publish(vis_pair.second);

    // specify ros loop rate
    ros::Rate loop_rate(10);
    ros::spinOnce();

    // ros loop
    while (ros::ok())
    {
        // publish the individual messages
        path_before_publisher.publish(before_path);
        path_after_publisher.publish(after_path);

        // more ros related stuff
        ros::spinOnce();

        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}