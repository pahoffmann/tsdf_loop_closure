/**
 * @file loop_closure_node.cpp
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief Loop Closure Node used to detect and apply loop closure to a tsdf based slam
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
#include "map/global_map.h"
#include "map/local_map.h"
#include "util/colors.h"
#include "path/path.h"
#include "util/point.h"
#include "ray_tracer/ray_tracer.h"
#include "serialization/read_path_json.h"
#include "data_association/association_manager.h"
#include "options/options_reader.h"
#include "visualization/ros_viewhelper.h"
#include "path/path_exploration.h"

// Configuration stuff //
lc_options_reader *options;

// ROS STUFF //
visualization_msgs::Marker ray_markers;
visualization_msgs::Marker bb_marker;
visualization_msgs::Marker tsdf_map;      // marker for the tsdf map (local)
visualization_msgs::Marker tsdf_map_full; // marker for the full tsdf map (global)

// both of these side lengths are in real world coordinates
float side_length_xy = 0;
float side_length_z = 0;

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
 * @todo don't harcode this
 *
 */
void initMaps()
{
  global_map_ptr_ = std::make_shared<GlobalMap>(options->get_map_file_name(), 0.0, 0.0);
  // todo: this is currently hardcoded. is there a way to retrieve the local map size from the hdf5?
  // FIXME: there is currently no way to read metadata from the map, we should introduce this in the map implementation
  local_map_ptr_ = std::make_shared<LocalMap>(201, 201, 95, global_map_ptr_, true); // still hardcoded af

  auto &size = local_map_ptr_.get()->get_size();
  side_length_xy = size.x() * MAP_RESOLUTION / 1000.0f;
  side_length_z = size.z() * MAP_RESOLUTION / 1000.0f;
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

  //cleans up global map a bit
  global_map_ptr_->cleanup_artifacts();

  // define stuff for raytracer
  ray_tracer = new RayTracer(options, local_map_ptr_);

  /******************************************************
   *  Get Path                                          *
   ******************************************************/

  // init path and read from json
  path = new Path();

  // retrieve path from various locations
  try
  {
    if (path_method == 0)
    {
      // if the global map does not have a path, we do path exploration, just in case.
      if (!global_map_ptr_->has_path())
      {
        path_exploration exploration(local_map_ptr_, global_map_ptr_, path, ray_tracer);
        exploration.dijsktra();
        global_map_ptr_->write_path(path->getPoses());
      }
      else
      {
        auto path_poses = global_map_ptr_->get_path();

        for (auto pose : path_poses)
        {
          path->add_pose(pose);
        }
      }
    }
    else if (path_method == 1)
    {
      path_exploration exploration(local_map_ptr_, global_map_ptr_, path, ray_tracer);
      exploration.dijsktra();
    }
    else
    {
      path->fromJSON(options->get_poses_file_name());
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
  ros::Publisher cube_publisher = n.advertise<visualization_msgs::Marker>("cubes", 1, true);
  ros::Publisher pose_publisher = n.advertise<visualization_msgs::Marker>("ray_trace_pose", 1, true);
  ros::Publisher path_publisher = n.advertise<visualization_msgs::Marker>("path", 1, true);
  ros::Publisher ray_publisher = n.advertise<visualization_msgs::Marker>("rays", 100);
  ros::Publisher bb_publisher = n.advertise<visualization_msgs::Marker>("bounding_box", 1, true);
  ros::Publisher chunk_publisher = n.advertise<visualization_msgs::Marker>("chunk_poses", 1, true);

  // specify ros loop rate
  ros::Rate loop_rate(10);

  // create associationmanager
  manager = new AssociationManager(path, options->get_base_file_name(), ray_tracer, local_map_ptr_, global_map_ptr_);
  manager->greedy_associations();

  // obtain the ros marker for visualization
  ray_markers = ray_tracer->get_ros_marker();

  // get markers
  auto pose_marker = ROSViewhelper::initPoseMarker(path->at(0));
  auto path_marker = ROSViewhelper::initPathMarker(path);
  auto chunk_marker = ROSViewhelper::initPathExtractionVisualizion(global_map_ptr_, local_map_ptr_);

  // initialize the bounding box
  bb_marker = ROSViewhelper::getBoundingBoxMarker(side_length_xy, side_length_z, path->at(0));

  // init tsdf
  // tsdf_map = ROSViewhelper::initTSDFmarkerPose(local_map_ptr_, path->at(0));

  // full tsdf map for display, very ressource intensive, especially for large maps..
  tsdf_map_full = ROSViewhelper::initTSDFmarkerPath(local_map_ptr_, path);

  // some stuff doesnt need to be published every iteration...
  bb_publisher.publish(bb_marker);
  pose_publisher.publish(pose_marker);
  path_publisher.publish(path_marker);
  // cube_publisher.publish(tsdf_map);
  cube_publisher.publish(tsdf_map_full);
  chunk_publisher.publish(chunk_marker);

  ros::spinOnce();

  // ros loop
  while (ros::ok())
  {
    // publish the individual messages
    bb_publisher.publish(bb_marker);
    ray_publisher.publish(ray_markers);

    // more ros related stuff
    ros::spinOnce();

    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}