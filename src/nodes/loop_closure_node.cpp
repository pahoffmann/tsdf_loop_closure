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

// Configuration stuff //

LoopClosureParams params;

// ROS STUFF //
visualization_msgs::Marker ray_markers;
visualization_msgs::Marker bb_marker;
visualization_msgs::Marker tsdf_map;             // marker for the tsdf map (local)
visualization_msgs::Marker tsdf_map_full_before; // marker for the full tsdf map (global) (before map update)
visualization_msgs::Marker tsdf_map_full_after;  // marker for the full tsdf map (global) (after map update)

std::vector<visualization_msgs::Marker> loop_visualizations;
visualization_msgs::Marker path_marker;
visualization_msgs::Marker updated_path_marker;
visualization_msgs::Marker pose_marker;
visualization_msgs::Marker chunk_marker;
visualization_msgs::Marker tsdf_read_marker;
visualization_msgs::Marker bresenham_marker;

// used to store sinuglar updates between the path and the path after all the updates
std::vector<visualization_msgs::Marker> path_marker_updates;

// ros publishers declaration
ros::Publisher tsdf_before_publisher;
ros::Publisher tsdf_after_publisher;
ros::Publisher tsdf_read_publisher;
ros::Publisher pose_publisher;
ros::Publisher path_publisher;
ros::Publisher updated_path_publisher;
ros::Publisher ray_publisher;
ros::Publisher bb_publisher;
ros::Publisher chunk_publisher;
ros::Publisher bresenham_int_publisher;
ros::Publisher loop_pub;

/// Map Stuff ///
std::shared_ptr<GlobalMap> global_map_ptr_;
std::shared_ptr<LocalMap> local_map_ptr_;

/// Ray Tracer ///
RayTracer *ray_tracer;

/// Path ///
Path *path;
Path updated_path;

/// Association Manager, used to manage and keep track of the associations ///
AssociationManager *manager;

// an enum used to update the path as a test, when no loop optimization is possible
enum PathUpdateTestMethod
{
  ROTATION,
  TRANSLATION,
  BLURRING,
  TRANSLATION_AND_ROTATION
};

/**
 * @brief initializes the local and global map
 *
 */
void initMaps(bool cleanup)
{
  global_map_ptr_ = std::make_shared<GlobalMap>(params.map.filename.string()); // create a global map, use it's attributes
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
 * @brief functions, which finds the next loop, giving a path and a start index
 * @todo find suitable parameters for the loop findings
 *
 * @param path
 * @return std::pair<int, int>
 */
std::pair<int, int> find_next_loop(Path *path, int start_idx = 0)
{
  // dont hardcode these, or if - make sure they fit
  float MAX_DISTANCE = 3.0f;
  float MIN_TRAVELED = 10.0f;

  // find path, including visibility check
  auto res = path->find_loop_kd_min_dist(start_idx, params.loop_closure.max_dist_lc, params.loop_closure.min_traveled_lc, true);

  if (res.first == -1 || res.second == -1)
  {
    std::cout << "No further loop found for the current parametrization" << std::endl;
  }
  else
  {
    std::cout << "Found a closed loop! Between index " << res.first << " and index " << res.second << std::endl;
  }

  return res;
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
  path->attach_raytracer(ray_tracer);

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
 * @brief Populate the ros publishers
 *
 */
void populate_publishers(ros::NodeHandle &n)
{
  tsdf_before_publisher = n.advertise<visualization_msgs::Marker>("tsdf_before", 1, true);
  tsdf_after_publisher = n.advertise<visualization_msgs::Marker>("tsdf_after", 1, true);
  tsdf_read_publisher = n.advertise<visualization_msgs::Marker>("tsdf_read", 1, true);
  pose_publisher = n.advertise<visualization_msgs::Marker>("ray_trace_pose", 1, true);
  path_publisher = n.advertise<visualization_msgs::Marker>("path", 1, true);
  updated_path_publisher = n.advertise<visualization_msgs::Marker>("path_updated", 1, true);
  ray_publisher = n.advertise<visualization_msgs::Marker>("rays", 100);
  bb_publisher = n.advertise<visualization_msgs::Marker>("bounding_box", 1, true);
  chunk_publisher = n.advertise<visualization_msgs::Marker>("chunk_poses", 1, true);
  bresenham_int_publisher = n.advertise<visualization_msgs::Marker>("bresenham_intersections", 1, true);
  loop_pub = n.advertise<visualization_msgs::Marker>("loop", 1);
}

/**
 * @brief
 *
 * @param path
 * @param method
 * @return Path
 */
Path update_path_test(Path *path, PathUpdateTestMethod method, int start_idx = -1, int end_idx = -1)
{
  if (start_idx == -1)
  {
    start_idx = 0;
  }

  if (end_idx == -1)
  {
    end_idx = path->get_length() - 1;
  }

  switch (method)
  {
  case PathUpdateTestMethod::BLURRING:
    // blur with radius of 0.5m
    return path->blur_ret(start_idx, end_idx, 0.5f);

  case PathUpdateTestMethod::ROTATION:
    // rotate the whole
    return path->rotate_ret(0, 90, 0);

  case PathUpdateTestMethod::TRANSLATION:
    return path->translate_ret(Vector3f(3.0f, 3.0f, 3.0f), start_idx, end_idx);

  case PathUpdateTestMethod::TRANSLATION_AND_ROTATION:
    return path->translate_ret(Vector3f(3.0f, 3.0f, 3.0f), start_idx, end_idx).rotate_ret(0, 0, 90);

  default:
    // return an empty path
    return Path();
  }
}

/**
 * @brief updates the path after finding a loop
 * @todo IMPLEMENT THIS
 *
 * @return Path
 */
Path update_path()
{
  return Path();
}

void populate_markers()
{
  // create a marker for the updated map
  updated_path_marker = ROSViewhelper::initPathMarker(&updated_path);

  // tsdf_map_full_before = ROSViewhelper::initTSDFmarkerPath(local_map_ptr_, path, false);

  // auto tsdf_read_marker = manager->update_localmap(&rotated_path, 0, rotated_path.get_length() - 1, AssociationManager::UpdateMethod::MEAN);
  // tsdf_read_marker = manager->update_localmap(&updated_path, 0, updated_path.get_length() - 1, AssociationManager::UpdateMethod::MEAN);

  bresenham_marker = ray_tracer->get_bresenham_intersection_marker();

  // get markers
  pose_marker = ROSViewhelper::initPoseMarker(path->at(0));
  chunk_marker = ROSViewhelper::initPathExtractionVisualizion(global_map_ptr_, local_map_ptr_);

  // initialize the bounding box
  bb_marker = ROSViewhelper::getBoundingBoxMarker(map_to_real(local_map_ptr_->get_size()), path->at(0));

  // full tsdf map for display, very ressource intensive, especially for large maps..
  //tsdf_map_full_after = ROSViewhelper::initTSDFmarkerPath(local_map_ptr_, &updated_path, true);
  // tsdf_map_full_after = ROSViewhelper::initTSDFmarkerPath(local_map_ptr_, path, false);

  auto gm_data = global_map_ptr_->get_full_data();
  tsdf_map_full_after = ROSViewhelper::marker_from_gm_read(gm_data);
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

  // read params
  params = LoopClosureParams(nh);

  // init local and global maps
  initMaps(false);

  // define stuff for raytracer
  ray_tracer = new RayTracer(params, local_map_ptr_, global_map_ptr_);

  // retrieve path method from options (0 = from globalmap, 1 = from path extraction)
  initialize_path(params.loop_closure.path_method);
  path_marker = ROSViewhelper::initPathMarker(path);

  // generate publishers
  populate_publishers(n);

  // generate marker for before map
  auto gm_data = global_map_ptr_->get_full_data();
  tsdf_map_full_before = ROSViewhelper::marker_from_gm_read(gm_data);

  // local variables used to handle the big while loop following:
  bool is_ok = true;
  int num_loops = 0;
  int current_start_idx = 0;

  while (is_ok)
  {
    std::pair<int, int> lc_pair = find_next_loop(path, current_start_idx);

    // break, if no loop is found, aka at least one of the returned indices is -1
    if (lc_pair.first == -1 || lc_pair.second == -1)
    {
      is_ok = false;

      break;
    }

    current_start_idx = lc_pair.second;

    // we found a loop!
    num_loops++;

    // create associationmanager and find the associations for the current path
    manager = new AssociationManager(path, params.loop_closure.json_dirname, ray_tracer, local_map_ptr_, global_map_ptr_);

    // TODO: this function should only generate associations of poses which 'participate' in the loop closure
    manager->greedy_associations();

    // update the path after finding the loop!
    // updated_path = update_path();
    updated_path = update_path_test(path, PathUpdateTestMethod::TRANSLATION, lc_pair.first, lc_pair.second);

    // create a visualization marker
    loop_visualizations.push_back(ROSViewhelper::init_loop_detected_marker(path->at(lc_pair.first)->pos, path->at(lc_pair.second)->pos));

    // update the localmap with the updated path
    tsdf_read_marker = manager->update_localmap(&updated_path, lc_pair.first, lc_pair.second, AssociationManager::UpdateMethod::MEAN);

    std::cout << "[MAIN] IN THE LEVEL X DATA, THERE ARE " << tsdf_read_marker.points.size() << " POINTS!" << std::endl;
    // manager->test_associations();

    // after every run, the data needs to be cleaned
    manager->cleanup();
  }

  // when no loop is found, we terminate early
  if (num_loops == 0)
  {
    std::cout << "[Main]: No Loops found in the current HDF5, terminating..." << std::endl;
    exit(EXIT_SUCCESS);
  }
  else
  {
    std::cout << "[Main]: Found " << num_loops << " loop(s)" << std::endl;
  }

  // write back the data
  local_map_ptr_->write_back();

  // now populate the markers
  populate_markers();

#ifdef DEBUG
  std::cout
      << "Before update size: " << tsdf_map_full_before.points.size() << std::endl;
  std::cout << "After update size: " << tsdf_map_full_after.points.size() << std::endl;
#endif

  // auto single_marker = ROSViewhelper::initPoseAssociationVisualization(global_map_ptr_, path->at(0), 0);

  // specify ros loop rate
  ros::Rate loop_rate(10);

  // some stuff doesnt need to be published every iteration...
  bb_publisher.publish(bb_marker);
  pose_publisher.publish(pose_marker);
  path_publisher.publish(path_marker);
  updated_path_publisher.publish(updated_path_marker);
  // tsdf_publisher.publish(tsdf_map);
  tsdf_before_publisher.publish(tsdf_map_full_before);
  tsdf_after_publisher.publish(tsdf_map_full_after);
  // tsdf_publisher.publish(single_marker);
  tsdf_read_publisher.publish(tsdf_read_marker);
  chunk_publisher.publish(chunk_marker);

  if (bresenham_marker.points.size() != 0)
  {
    bresenham_int_publisher.publish(bresenham_marker);
  }

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