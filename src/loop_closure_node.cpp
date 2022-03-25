/**
 * @file loop_closure_node.cpp
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief 
 * @version 0.1
 * @date 2022-03-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <loop_closure/LoopClosureConfig.h>
#include <loop_closure/RayTracerConfig.h>
#include <iostream>
#include <highfive/H5File.hpp>
#include "map/global_map.h"
#include "map/local_map.h"
#include "util/colors.h"
#include "path/path.h"
#include "util/point.h"
#include "ray_tracer/ray_tracer.h"
//#include "ray_tracer/tracer.h"
#include "serialization/read_path_json.h"
#include "data_association/association_manager.h"

// ROS STUFF //

Pose raytrace_starting_pose; // starting pose for ray tracing
std::vector<bool> lines_finished;
visualization_msgs::Marker ray_markers;
visualization_msgs::Marker bb_marker;
visualization_msgs::Marker tsdf_map; // marker for the tsdf map
loop_closure::LoopClosureConfig lc_config;
// loop_closure::RayTracerConfig rt_config;
float side_length_xy = 0;
float side_length_z = 0;
std::string h5_file_name_;
std::string poses_file_name_;
std::string file_base_path_;
bool has_update = true;

/// Map Stuff ///
std::shared_ptr<GlobalMap> global_map_ptr_;
std::shared_ptr<LocalMap> local_map_ptr_;

/// Ray Tracer ///
RayTracer *ray_tracer;

// Association Manager */
AssociationManager *manager;

/**
 * @brief Method, which generates the pose marker for the current ray trace position
 *
 * @return visualization_msgs::Marker (Sphere)
 */
visualization_msgs::Marker initPoseMarker()
{
  // raytrace pose
  visualization_msgs::Marker raytrace_starting_pose_marker;
  raytrace_starting_pose_marker.header.frame_id = "map";
  raytrace_starting_pose_marker.header.stamp = ros::Time();
  raytrace_starting_pose_marker.ns = "ray_pose";
  raytrace_starting_pose_marker.id = 0;
  raytrace_starting_pose_marker.type = visualization_msgs::Marker::SPHERE;
  raytrace_starting_pose_marker.action = visualization_msgs::Marker::ADD;
  raytrace_starting_pose_marker.pose.position.x = raytrace_starting_pose.pos.x();
  raytrace_starting_pose_marker.pose.position.y = raytrace_starting_pose.pos.y();
  raytrace_starting_pose_marker.pose.position.z = raytrace_starting_pose.pos.z();
  raytrace_starting_pose_marker.pose.orientation.x = 0.0;
  raytrace_starting_pose_marker.pose.orientation.y = 0.0;
  raytrace_starting_pose_marker.pose.orientation.z = 0.0;
  raytrace_starting_pose_marker.pose.orientation.w = 1.0;
  raytrace_starting_pose_marker.scale.x = 0.4;
  raytrace_starting_pose_marker.scale.y = 0.4;
  raytrace_starting_pose_marker.scale.z = 0.4;
  raytrace_starting_pose_marker.color.a = 0.5; // Don't forget to set the alpha!
  raytrace_starting_pose_marker.color.r = 1.0;
  raytrace_starting_pose_marker.color.g = 0.0;
  raytrace_starting_pose_marker.color.b = 0.0;

  return raytrace_starting_pose_marker;
}

/**
 * @brief Reads a tsdf global map and displays it in
 *
 * @todo use a tsdf-map datatype to read it and check intersection (separate class), marker should only be used as visualization,
 * only visualize chunks, which are in the bounds of the "local map", which can be seen from the raytracing position.
 *
 * @return visualization_msgs::Marker
 */
visualization_msgs::Marker initTSDFmarker()
{
  // create marker.
  visualization_msgs::Marker tsdf_markers;
  tsdf_markers.header.frame_id = "map";
  tsdf_markers.header.stamp = ros::Time();
  tsdf_markers.ns = "tsdf";
  tsdf_markers.id = 0;
  tsdf_markers.type = visualization_msgs::Marker::POINTS;
  tsdf_markers.action = visualization_msgs::Marker::ADD;
  tsdf_markers.scale.x = tsdf_markers.scale.y = MAP_RESOLUTION * 1.0 * 0.001; // why this? @julian -> 0.6 as parameter?
  std::vector<geometry_msgs::Point> points;                                   // 3 x 3 markers

  // display the current local map...

  // iterate through the whole localmap [3d]
  auto local_map = local_map_ptr_.get();
  auto &size = local_map->get_size();
  int num_intersects = 0;

  // define colors for the tsdf and intersections
  // intersection color might need to vary

  auto intersectColor = Colors::color_from_name(Colors::ColorNames::fuchsia);
  auto redTSDFColor = Colors::color_from_name(Colors::ColorNames::maroon);
  auto greenTSDFColor = Colors::color_from_name(Colors::ColorNames::green);

  ROS_INFO("Color: %f, %f, %f", intersectColor.r, intersectColor.g, intersectColor.b);

  Vector3i tmp_pos = (raytrace_starting_pose.pos * 1000.0f / MAP_RESOLUTION).cast<int>();

  // get values, ignore offset
  for (int x = tmp_pos.x() + (-1 * (size.x() - 1) / 2); x < tmp_pos.x() + ((size.x() - 1) / 2); x++)
  {
    for (int y = tmp_pos.y() + (-1 * (size.y() - 1) / 2); y < tmp_pos.y() + ((size.y() - 1) / 2); y++)
    {
      for (int z = tmp_pos.z() + (-1 * (size.z() - 1) / 2); z < tmp_pos.z() + ((size.z() - 1) / 2); z++)
      {
        auto tsdf = local_map->value(x, y, z);
        auto value = tsdf.value();
        auto weight = tsdf.weight();
        auto intersect = tsdf.getIntersect();
        if (intersect)
        {
          num_intersects++;
        }

        // just the cells, which actually have a weight and a value other than (and including) the default one
        if (weight > 0 && value < 600)
        {
          geometry_msgs::Point point;
          point.x = x * 0.001 * MAP_RESOLUTION;
          point.y = y * 0.001 * MAP_RESOLUTION;
          point.z = z * 0.001 * MAP_RESOLUTION;

          std_msgs::ColorRGBA color;

          if (intersect)
          {
            color = intersectColor;
          }
          else if (value < 0)
          {
            color = redTSDFColor;
          }
          else
          {
            color = greenTSDFColor;
          }

          tsdf_markers.points.push_back(point);
          tsdf_markers.colors.push_back(color);
        }
      }
    }
  }

  ROS_INFO("NUM_INTERSECTS: %d", num_intersects);

  return tsdf_markers;
}

/**
 * @brief Method, which generates a bounding box for to show the current size of the local map.
 *
 * @return visualization_msgs::Marker
 */
visualization_msgs::Marker initBoundingBox()
{

  ROS_INFO("[BOUNDING-BOX]: side_length: %f, %f, %f", side_length_xy, side_length_xy, side_length_z);
  ROS_INFO("[BOUNDING-BOX]: starting_pose: %f, %f, %f", raytrace_starting_pose.pos.x(), raytrace_starting_pose.pos.y(), raytrace_starting_pose.pos.z());
  visualization_msgs::Marker cube;
  cube.header.frame_id = "map";
  cube.header.stamp = ros::Time();
  cube.ns = "bounding_box";
  cube.id = 0;
  cube.type = visualization_msgs::Marker::CUBE;
  cube.action = visualization_msgs::Marker::ADD;
  cube.pose.position.x = raytrace_starting_pose.pos.x();
  cube.pose.position.y = raytrace_starting_pose.pos.y();
  cube.pose.position.z = raytrace_starting_pose.pos.z();
  cube.pose.orientation.x = 0.0;
  cube.pose.orientation.y = 0.0;
  cube.pose.orientation.z = 0.0;
  cube.pose.orientation.w = 1.0;
  cube.scale.x = side_length_xy;
  cube.scale.y = side_length_xy;
  cube.scale.z = side_length_z;
  cube.color.a = 0.2; // Don't forget to set the alpha!
  cube.color.r = 1.0;
  cube.color.g = 1.0;
  cube.color.b = 0.0;
  cube.lifetime.fromSec(30);

  return cube;
}

/**
 * @brief initializes the local and global map
 * @todo don't harcode this
 *
 */
void initMaps()
{
  global_map_ptr_ = std::make_shared<GlobalMap>(h5_file_name_, 0.0, 0.0);
  // todo: this is currently hardcoded. is there a way to retrieve the local map size from the hdf5?
  local_map_ptr_ = std::make_shared<LocalMap>(201, 201, 95, global_map_ptr_, true); // still hardcoded af

  auto &size = local_map_ptr_.get()->get_size();
  side_length_xy = size.x() * MAP_RESOLUTION / 1000.0f;
  side_length_z = size.z() * MAP_RESOLUTION / 1000.0f;
}

/**
 * @brief Set the Starting Position object, gets rotation from lc config, if not specified
 *
 * @param x
 * @param y
 * @param z
 * @param roll
 * @param pitch
 * @param yaw
 */
void set_starting_position(float x, float y, float z, float roll = std::numeric_limits<float>::infinity(),
                           float pitch = std::numeric_limits<float>::infinity(), float yaw = std::numeric_limits<float>::infinity())
{
  // starting position (6D)
  raytrace_starting_pose.pos.x() = 0;
  raytrace_starting_pose.pos.y() = 0;
  raytrace_starting_pose.pos.z() = 0;

  float infinity = std::numeric_limits<float>::infinity();

  // get roll pitch yaw
  float my_roll = roll == infinity ? lc_config.roll_start : roll;
  float my_pitch = pitch == infinity ? lc_config.pitch_start : pitch;
  float my_yaw = yaw == infinity ? lc_config.yaw_start : yaw;

  Eigen::Quaternionf q;
  auto rollAngle = Eigen::AngleAxisf(my_roll * M_PI / 180, Eigen::Vector3f::UnitX()); // todo: create helper function for degree <-> radiants
  auto pitchAngle = Eigen::AngleAxisf(my_pitch * M_PI / 180, Eigen::Vector3f::UnitY());
  auto yawAngle = Eigen::AngleAxisf(my_yaw * M_PI / 180, Eigen::Vector3f::UnitZ());

  q = yawAngle * pitchAngle * rollAngle;

  raytrace_starting_pose.quat = q;
}

/**
 * @brief handles the dynamic reconfiguration, restarts scan simulation with new configuration
 *
 * @param config
 * @param level
 */
void dynamic_reconfigure_callback(loop_closure::LoopClosureConfig &config, uint32_t level)
{
  ROS_INFO("-------------------------------------------------------------------------");
  ROS_INFO("Reconfigure Request for Ray Tracer. Overwriting config..."); // todo: add new params to info call
  lc_config = config;
  ROS_INFO("Done Overwriting!");
  ROS_INFO("-------------------------------------------------------------------------");
  ROS_INFO("\n");

  // reinitialize the map, as the intersections are now invalid TODO: where and how! this is important shit alla
  initMaps();
  ray_tracer->update_map_pointer(local_map_ptr_); // after the maps are reinitialized, we need to update the pointers

  //  re-init rotation component (todo: outsource)
  //set_starting_position(0, 0, 0);

  manager->greedy_associations();

  // restart the ray tracer
  // and do some time measuring
  /*auto start_time = ros::Time::now();

  ray_tracer->start();

  // more time measuring
  auto end_time = ros::Time::now();

  // calc duration
  auto duration = end_time - start_time;

  ROS_INFO("[RayTracer] Time Measurement: %.2f ms", duration.toNSec() / 1000000.0f); // display time in ms, with two decimal points*/

  ROS_INFO("Obtaining new ray marker from ray tracer");

  // update the ray marker
  ray_markers = ray_tracer->get_ros_marker();

  has_update = true;
  // re-init the markers
  bb_marker = initBoundingBox();
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

  // if no map was set, we won't go on from here.
  if (!nh.getParam("map", h5_file_name_))
  {
    ROS_WARN("No Global Map file delivered, use _map:=[Path] shutting down...");
    exit(EXIT_FAILURE);
  }
  else {
    ROS_INFO("[DEBUG] Global Map file delivered: %s", h5_file_name_.c_str());
  }

  if (!nh.getParam("poses", poses_file_name_))
  {
    ROS_WARN("No Poses file delivered, use _poses:=[Path], shutting down...");
    exit(EXIT_FAILURE);
  }
  else {
    ROS_INFO("[DEBUG] Poses file delivered: %s", poses_file_name_.c_str());
  }

  if (!nh.getParam("basepath", file_base_path_))
  {
    ROS_WARN("No File Base Path file delivered, use _basepath:=[Path], shutting down...");
    exit(EXIT_FAILURE);
  }
  else {
    ROS_INFO("[DEBUG] Base file path delivered: %s", file_base_path_.c_str());
  }

  // init path and read from json
  Path path;
  path.fromJSON(poses_file_name_);


  // init local and global maps
  initMaps();

  // feberate publishers
  ros::Publisher cube_publisher = n.advertise<visualization_msgs::Marker>("cubes", 1, true);
  ros::Publisher pose_publisher = n.advertise<visualization_msgs::Marker>("ray_trace_pose", 1, true);
  ros::Publisher ray_publisher = n.advertise<visualization_msgs::Marker>("rays", 100);
  ros::Publisher bb_publisher = n.advertise<visualization_msgs::Marker>("bounding_box", 1, true);

  // specify ros loop rate
  ros::Rate loop_rate(10);

  set_starting_position(0, 0, 0);

  // define stuff for raytracer
  ray_tracer = new RayTracer(&lc_config, local_map_ptr_, &raytrace_starting_pose);

  // create associationmanager
  manager = new AssociationManager(&path, file_base_path_, ray_tracer, local_map_ptr_);

  // and lastly: reconfigure callbacks
  dynamic_reconfigure::Server<loop_closure::LoopClosureConfig> server;
  dynamic_reconfigure::Server<loop_closure::LoopClosureConfig>::CallbackType f;

  f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
  server.setCallback(f);

  // get markers
  auto pose_marker = initPoseMarker();

  // initialize the bounding box
  bb_marker = initBoundingBox();

  // init tsdf
  tsdf_map = initTSDFmarker();

  // some stuff doesnt need to be published every iteration...
  bb_publisher.publish(bb_marker);
  pose_publisher.publish(pose_marker);
  cube_publisher.publish(tsdf_map);

  ros::spinOnce();

  // ros loop
  while (ros::ok())
  {
    // if there is an update, aka there has been some dynamic reconfiguration, we need to reinit the tsdf map
    if (has_update)
    {
      // ROS_INFO("WE GOT AN UPDATE FOR YOU..");
      tsdf_map = initTSDFmarker();
      has_update = false;
      // CudaTracing::helloWorld(); // test cuda
    }

    // publish the individual messages
    bb_publisher.publish(bb_marker);
    ray_publisher.publish(ray_markers);
    cube_publisher.publish(tsdf_map);

    // more ros related stuff
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}