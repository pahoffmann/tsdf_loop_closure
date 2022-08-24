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
visualization_msgs::Marker ray_markers;
visualization_msgs::Marker bb_marker;
visualization_msgs::Marker tsdf_map;             // marker for the tsdf map (local)
visualization_msgs::Marker tsdf_map_full_before; // marker for the full tsdf map (global) (before map update)
visualization_msgs::Marker tsdf_map_full_after;  // marker for the full tsdf map (global) (after map update)

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

  auto &size = local_map_ptr_.get()->get_size();
  side_length_xy = size.x() * MAP_RESOLUTION / 1000.0f;
  side_length_z = size.z() * MAP_RESOLUTION / 1000.0f;

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
  path = new Path(ray_tracer);

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
  ros::Publisher tsdf_before_publisher = n.advertise<visualization_msgs::Marker>("tsdf_before", 1, true);
  ros::Publisher tsdf_after_publisher = n.advertise<visualization_msgs::Marker>("tsdf_after", 1, true);
  ros::Publisher tsdf_read_publisher = n.advertise<visualization_msgs::Marker>("tsdf_read", 1, true);
  ros::Publisher pose_publisher = n.advertise<visualization_msgs::Marker>("ray_trace_pose", 1, true);
  ros::Publisher path_publisher = n.advertise<visualization_msgs::Marker>("path", 1, true);
  ros::Publisher pathblur_publisher = n.advertise<visualization_msgs::Marker>("path_blur", 1, true);
  ros::Publisher ray_publisher = n.advertise<visualization_msgs::Marker>("rays", 100);
  ros::Publisher bb_publisher = n.advertise<visualization_msgs::Marker>("bounding_box", 1, true);
  ros::Publisher chunk_publisher = n.advertise<visualization_msgs::Marker>("chunk_poses", 1, true);
  ros::Publisher bresenham_int_publisher = n.advertise<visualization_msgs::Marker>("bresenham_intersections", 1, true);
  ros::Publisher loop_pub = n.advertise<visualization_msgs::Marker>("/loop", 1);

  // specify ros loop rate
  ros::Rate loop_rate(10);

  // check for loops: THE PARAMETERS HERE ARE SOMEWHAT RANDOM... :D
  int start_idx = 0;
  int end_idx = path->get_length() - 1;
  bool is_ok = true;
  std::vector<visualization_msgs::Marker> loop_visualizations;

  // create a path for blurring
  Path blurred_path(*path);

  std::vector<std::pair<int, int>> lc_pairs;

  // TODO: this should probably be outsourced somehow
  while (is_ok)
  {
    // find path, including visibility check
    auto res = path->find_loop_greedy(start_idx, 3.0f, 10.0f, true);

    // found
    if (res.first != -1 && res.second != -1)
    {
      // update start index for possible second closure
      start_idx = res.second;

      std::cout << "Found a closed loop! Between index " << res.first << " and index " << res.second << std::endl;

      // create a visualization marker
      loop_visualizations.push_back(ROSViewhelper::init_loop_detected_marker(path->at(res.first)->pos, path->at(res.second)->pos));

      // save the loop closure index pair
      lc_pairs.push_back(res);
    }
    else
    {
      std::cout << "No further Loop found in the current hdf5. " << std::endl;
      is_ok = false;
    }
  }

  for (auto pair : lc_pairs)
  {
    // blur with radius of 0.5m (todo: param)
    blurred_path = blurred_path.blur_ret(pair.first, pair.second, 0.5f);
  }

  // rotate the path for testing
  Path rotated_path = path->rotate_ret(0, 90, 0);

  // when no loop is found, we terminate early
  if (loop_visualizations.size() == 0)
  {
    std::cout << "[Main]: No Loops found in the current HDF5, terminating..." << std::endl;
    exit(EXIT_SUCCESS);
  }
  else
  {
    std::cout << "[Main]: Found " << loop_visualizations.size() << " loop(s)" << std::endl;
  }

  auto path_marker_blurred = ROSViewhelper::initPathMarker(&blurred_path);
  auto path_marker_rotated = ROSViewhelper::initPathMarker(&rotated_path);

  std::cout << "[Main]: " << blurred_path.get_length() << " Poses in blurred path" << std::endl;

  // create associationmanager
  manager = new AssociationManager(path, options->get_base_file_name(), ray_tracer, local_map_ptr_, global_map_ptr_);
  manager->greedy_associations();

  // obtain the ros marker for visualization
  // ray_markers = ray_tracer->get_ros_marker();

  // now do the update process
  // for (auto pair : lc_pairs)
  // {
  //   manager->update_localmap(&blurred_path, pair.first, pair.second, AssociationManager::UpdateMethod::MEAN);
  // }

  tsdf_map_full_before = ROSViewhelper::initTSDFmarkerPath(local_map_ptr_, path, true);
  // tsdf_map_full_before = ROSViewhelper::initTSDFmarkerPath(local_map_ptr_, path, false);

  auto tsdf_read_marker = manager->update_localmap(&rotated_path, 0, rotated_path.get_length() - 1, AssociationManager::UpdateMethod::MEAN);

  auto bresenham_marker = ray_tracer->get_bresenham_intersection_marker();

  // get markers
  auto pose_marker = ROSViewhelper::initPoseMarker(path->at(0));
  auto path_marker = ROSViewhelper::initPathMarker(path);
  auto chunk_marker = ROSViewhelper::initPathExtractionVisualizion(global_map_ptr_, local_map_ptr_);

  // initialize the bounding box
  bb_marker = ROSViewhelper::getBoundingBoxMarker(side_length_xy, side_length_z, path->at(0));

  // init tsdf
  // tsdf_map = ROSViewhelper::initTSDFmarkerPose(local_map_ptr_, path->at(0));

  // full tsdf map for display, very ressource intensive, especially for large maps..
  tsdf_map_full_after = ROSViewhelper::initTSDFmarkerPath(local_map_ptr_, &rotated_path, true);
  // tsdf_map_full_after = ROSViewhelper::initTSDFmarkerPath(local_map_ptr_, path, false);

#ifdef DEBUG
  std::cout << "Before update size: " << tsdf_map_full_before.points.size() << std::endl;
  std::cout << "After update size: " << tsdf_map_full_after.points.size() << std::endl;
#endif

  // auto single_marker = ROSViewhelper::initPoseAssociationVisualization(global_map_ptr_, path->at(0), 0);

  // some stuff doesnt need to be published every iteration...
  bb_publisher.publish(bb_marker);
  pose_publisher.publish(pose_marker);
  path_publisher.publish(path_marker);
  // pathblur_publisher.publish(path_marker_blurred);
  pathblur_publisher.publish(path_marker_rotated);
  // tsdf_publisher.publish(tsdf_map);
  tsdf_before_publisher.publish(tsdf_map_full_before);
  tsdf_after_publisher.publish(tsdf_map_full_after);
  // tsdf_publisher.publish(single_marker);
  tsdf_read_publisher.publish(tsdf_read_marker);
  chunk_publisher.publish(chunk_marker);
  bresenham_int_publisher.publish(bresenham_marker);

  // publish path and loop detects
  for (auto marker : loop_visualizations)
  {
    loop_pub.publish(marker);
  }

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