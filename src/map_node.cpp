/* Create a map node, that publishes multiple cells of a specific size, trying to find bases for algorithm parts */

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
#include "util/path.h"
#include "util/point.h"
#include "ray_tracer/ray_tracer.h"

// ROS STUFF //

Pose raytrace_starting_pose; // starting pose for ray tracing
std::vector<bool> lines_finished;
visualization_msgs::Marker ray_markers;
visualization_msgs::Marker bb_marker;
visualization_msgs::Marker tsdf_map;         // marker for the tsdf map
loop_closure::LoopClosureConfig lc_config;
//loop_closure::RayTracerConfig rt_config;
float side_length_xy = 0;
float side_length_z = 0;
std::string h5_file_name_;
bool has_update = true;

/// Map Stuff ///
std::shared_ptr<GlobalMap> global_map_ptr_;
std::shared_ptr<LocalMap> local_map_ptr_;

/// Ray Tracer ///
RayTracer *ray_tracer;

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

  // get values, ignore offset
  for (int x = -1 * (size.x() - 1) / 2; x < (size.x() - 1) / 2; x++)
  {
    for (int y = -1 * (size.y() - 1) / 2; y < (size.y() - 1) / 2; y++)
    {
      for (int z = -1 * (size.z() - 1) / 2; z < (size.z() - 1) / 2; z++)
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
  //initMaps();

  //  re-init rotation component (todo: outsource)
  Eigen::Quaternionf q;
  auto rollAngle = Eigen::AngleAxisf(lc_config.roll_start * M_PI / 180, Eigen::Vector3f::UnitX()); // todo: create helper function for degree <-> radiants
  auto pitchAngle = Eigen::AngleAxisf(lc_config.pitch_start * M_PI / 180, Eigen::Vector3f::UnitY());
  auto yawAngle = Eigen::AngleAxisf(lc_config.yaw_start * M_PI / 180, Eigen::Vector3f::UnitZ());

  q = yawAngle * pitchAngle * rollAngle;

  raytrace_starting_pose.quat = q;

  // restart the ray tracer
  ray_tracer->start();


  ROS_INFO("Obtaining new ray marker from ray tracer");

  // update the ray marker
  ray_markers = ray_tracer->get_ros_marker();

  has_update = true;
  // re-init the markers
  bb_marker = initBoundingBox();
}

/**
 * @brief handles the reconfigure request for the ray tracer config
 * 
 * @param config 
 * @param level 
 */
void ray_tracer_reconfigure_callback(loop_closure::RayTracerConfig &config, uint32_t level)
{
  ROS_INFO("-------------------------------------------------------------------------");
  ROS_INFO("Reconfigure Request for Ray Tracer. Overwriting config..."); // todo: add new params to info call
  //rt_config = config; // overwrite config
  ROS_INFO("Done Overwriting!");
  ROS_INFO("-------------------------------------------------------------------------");
  ROS_INFO("\n");
  
  // restart the ray tracer
  ray_tracer->start();

  // update the ray marker
  ray_markers = ray_tracer->get_ros_marker();

  has_update = true;
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
  ros::init(argc, argv, "map_node");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  // if no map was set, we won't go on from here.
  if (!nh.getParam("map", h5_file_name_))
  {
    ROS_WARN("No Global Map file delivered, shutting down...");
    exit(EXIT_FAILURE);
  }

  // init local and global maps
  initMaps();

  ros::Publisher cube_publisher = n.advertise<visualization_msgs::Marker>("cubes", 1, true);
  ros::Publisher pose_publisher = n.advertise<visualization_msgs::Marker>("ray_trace_pose", 1, true);
  ros::Publisher ray_publisher = n.advertise<visualization_msgs::Marker>("rays", 100);
  ros::Publisher bb_publisher = n.advertise<visualization_msgs::Marker>("bounding_box", 1, true);


  ros::Rate loop_rate(10);


  // starting position
  raytrace_starting_pose.pos.x() = 0;
  raytrace_starting_pose.pos.y() = 0;
  raytrace_starting_pose.pos.z() = 0;

  Eigen::Quaternionf q;
  auto rollAngle = Eigen::AngleAxisf(lc_config.roll_start * M_PI / 180, Eigen::Vector3f::UnitX()); // todo: create helper function for degree <-> radiants
  auto pitchAngle = Eigen::AngleAxisf(lc_config.pitch_start * M_PI / 180, Eigen::Vector3f::UnitY());
  auto yawAngle = Eigen::AngleAxisf(lc_config.yaw_start * M_PI / 180, Eigen::Vector3f::UnitZ());

  q = yawAngle * pitchAngle * rollAngle;

  raytrace_starting_pose.quat = q;

  // define stuff for raytracer
  ray_tracer = new RayTracer(&lc_config, local_map_ptr_, &raytrace_starting_pose);

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

    bb_publisher.publish(bb_marker);

    if (has_update)
    {
      ROS_INFO("WE GOT AN UPDATE FOR YOU..");

      // publish the individual messages
      ray_publisher.publish(ray_markers);
      tsdf_map = initTSDFmarker();
      cube_publisher.publish(tsdf_map);

      has_update = false;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}