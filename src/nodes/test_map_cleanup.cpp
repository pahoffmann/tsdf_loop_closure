/**
 * @file test_map_cleanup.cpp
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief Node used to test the global map cleanup
 * @version 0.1
 * @date 2022-09-08
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
#include <loop_closure/util/point.h>
#include <loop_closure/path/path.h>
#include <loop_closure/options/options_reader.h>
#include <loop_closure/visualization/ros_viewhelper.h>

// Configuration stuff //
lc_options_reader *options;

// ROS STUFF //
visualization_msgs::Marker tsdf_map_full_before; // marker for the full tsdf map (global) (before map update)
visualization_msgs::Marker tsdf_map_full_after;  // marker for the full tsdf map (global) (after map update)
visualization_msgs::Marker removed_cells_marker;  // marker for the full tsdf map (global) (after map update)
visualization_msgs::Marker path_marker;          // marker for the full tsdf map (global) (after map update)

// ros publishers declaration
ros::Publisher tsdf_before_publisher;
ros::Publisher tsdf_after_publisher;
ros::Publisher removed_cells_publisher;
ros::Publisher path_publisher;

/// Map Stuff ///
std::shared_ptr<GlobalMap> global_map_ptr_;
std::shared_ptr<LocalMap> local_map_ptr_;

// Path //
Path *path;

/**
 * @brief initializes the local and global map
 *
 */
void initMaps(bool cleanup)
{
  global_map_ptr_ = std::make_shared<GlobalMap>(options->get_map_file_name()); // create a global map, use it's attributes
  auto attribute_data = global_map_ptr_->get_attribute_data();

  // local_map_ptr_ = std::make_shared<LocalMap>(312.5, 312.5, 312.5, global_map_ptr_, true); // not used anymore, though good 2 know

  local_map_ptr_ = std::make_shared<LocalMap>(attribute_data.get_map_size_x(),
                                              attribute_data.get_map_size_y(),
                                              attribute_data.get_map_size_z(),
                                              global_map_ptr_, true); // still hardcoded af

  auto &size = local_map_ptr_.get()->get_size();

  std::cout << "Finished init the maps" << std::endl;

  // cleanup the maps, when specified
  if (cleanup)
  {
    // cleans up global map a bit
    global_map_ptr_->cleanup_artifacts();
  }
}

/**
 * @brief initializes the path and tries to find path positions, if none was specified
 *
 * @param path_method
 */
void initialize_path(int path_method = 0)
{
  /******************************************************
   *  Get Path                                          *
   ******************************************************/

  // init path and read from specified source (mostly hdf5)
  path = new Path();

  // retrieve path from various locations
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
    exit(EXIT_FAILURE);
  }

  /******************************************************
   *  End Path Creation                                 *
   ******************************************************/
}

/**
 * @brief checks the options status and terminates the program if necessary
 *
 * @param status
 */
void check_options_status(int status)
{
  if (status == 1)
  {
    std::cout << "[CLI] Terminate node, because there were some errors while reading from cmd-line" << std::endl;

    return exit(EXIT_FAILURE);
  }
  else if (status == 2)
  {
    std::cout << "[CLI] Terminate node, because help was requested" << std::endl;

    return exit(EXIT_SUCCESS);
  }
}

/**
 * @brief Populate the ros publishers
 *
 */
void populate_publishers(ros::NodeHandle &n)
{
  tsdf_before_publisher = n.advertise<visualization_msgs::Marker>("tsdf_before", 1, true);
  tsdf_after_publisher = n.advertise<visualization_msgs::Marker>("tsdf_after", 1, true);
  removed_cells_publisher = n.advertise<visualization_msgs::Marker>("removed_cells", 1, true);
  path_publisher = n.advertise<visualization_msgs::Marker>("path", 1, true);
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

  // check the status returned from the options reading
  check_options_status(status);

  // init local and global maps
  initMaps(false);

  // retrieve path method from options (0 = from globalmap, 1 = from path extraction, 2 = from json)
  initialize_path(options->get_path_method());
  path_marker = ROSViewhelper::initPathMarker(path);

  // generate publishers
  populate_publishers(n);

  // generate marker for before map
  tsdf_map_full_before = ROSViewhelper::initTSDFmarkerPath(local_map_ptr_, path, true);

  // cleanup the map
  auto data = global_map_ptr_->cleanup_artifacts();
  std::cout << "Got " << data.size() << " cells which should have been deleted from cleanup process" << std::endl;

  removed_cells_marker = ROSViewhelper::marker_from_map_points(data);

  //global_map_ptr_->chunks_empty();

  tsdf_map_full_after = ROSViewhelper::initTSDFmarkerPath(local_map_ptr_, path, true);

  // specify ros loop rate
  ros::Rate loop_rate(10);

  path_publisher.publish(path_marker);

  tsdf_before_publisher.publish(tsdf_map_full_before);
  tsdf_after_publisher.publish(tsdf_map_full_after);
  removed_cells_publisher.publish(removed_cells_marker);

  ros::spinOnce();

  // ros loop
  while (ros::ok())
  {

    // more ros related stuff
    ros::spinOnce();

    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}