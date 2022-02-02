/* Create a map node, that publishes multiple cells of a specific size, trying to find bases for algorithm parts */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <loop_closure/LoopClosureConfig.h>
#include <iostream>
#include <highfive/H5File.hpp>
#include "map/global_map.h"
#include "map/local_map.h"

// ROS STUFF //

geometry_msgs::Point raytrace_starting_pose;        // starting pose for ray tracing
geometry_msgs::Vector3 raytrace_starting_direction; // aka, yaw, pitch, yaw
std::vector<bool> lines_finished;
visualization_msgs::Marker ray_markers;
visualization_msgs::Marker bb_marker;
visualization_msgs::Marker tsdf_map; // marker for the tsdf map
visualization_msgs::Marker cube_marker_list; // marker for the tsdf map
loop_closure::LoopClosureConfig lc_config;
float side_length_xy = 0;
float side_length_z = 0;
std::string h5_file_name_;

bool done_tracing = false;
bool done_iteration = false;

/// Map Stuff ///
std::shared_ptr<GlobalMap> global_map_ptr_;
std::shared_ptr<LocalMap> local_map_ptr_;

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
  raytrace_starting_pose_marker.pose.position.x = raytrace_starting_pose.x;
  raytrace_starting_pose_marker.pose.position.y = raytrace_starting_pose.y;
  raytrace_starting_pose_marker.pose.position.z = raytrace_starting_pose.z;
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
  std::vector<geometry_msgs::Point> points; // 3 x 3 markers

  // display the current local map...

  // iterate through the whole localmap [3d]
  auto local_map = local_map_ptr_.get();
  auto &size = local_map->get_size();
  int num_intersects = 0;

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
        if(intersect)
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
          
          if(intersect)
          {
            color.r = 1;
            color.g = 0;
            color.b = 1;
            color.a = 1;
          }
          else if(value < 0)
          {
            color.r = 1;
            color.g = 0;
            color.b = 0;
            color.a = 1;
          }
          else{
            color.r = 0;
            color.g = 1;
            color.b = 0;
            color.a = 1;
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
 * @brief Method, which generates a tsdf sim volume.
 * 
 * @return visualization_msgs::Marker (Cube List)
 * @deprecated deprecated due to the existance of initTSDFmarker() - Method 
 */
visualization_msgs::Marker initTSDFsimMarker()
{
  // tsdf sim
  visualization_msgs::Marker cube_marker_list;
  cube_marker_list.header.frame_id = "map";
  cube_marker_list.header.stamp = ros::Time();
  cube_marker_list.ns = "cube_list";
  cube_marker_list.id = 0;
  cube_marker_list.type = visualization_msgs::Marker::CUBE_LIST;
  cube_marker_list.action = visualization_msgs::Marker::ADD;
  cube_marker_list.pose.position.x = 0;
  cube_marker_list.pose.position.y = 0;
  cube_marker_list.pose.position.z = 0;
  cube_marker_list.pose.orientation.x = 0.0;
  cube_marker_list.pose.orientation.y = 0.0;
  cube_marker_list.pose.orientation.z = 0.0;
  cube_marker_list.pose.orientation.w = 1.0;
  cube_marker_list.scale.x = 1;
  cube_marker_list.scale.y = 1;
  cube_marker_list.scale.z = 1;
  cube_marker_list.color.a = 0.6; // Don't forget to set the alpha!
  cube_marker_list.color.r = 0.0;
  cube_marker_list.color.g = 1.0;
  cube_marker_list.color.b = 0.0;
  std::vector<geometry_msgs::Point> points; // 3 x 3 markers

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      for (int z = 0; z < 3; z++)
      {
        geometry_msgs::Point point;
        point.x = i;
        point.y = j;
        point.z = z;
        points.push_back(point);
      }
    }
  }

  cube_marker_list.points = points;

  return cube_marker_list;
}

/**
 * @brief Method, which generates a bounding box for to show the current size of the local map.
 * 
 * @return visualization_msgs::Marker 
 */
visualization_msgs::Marker initBoundingBox()
{

  ROS_INFO("[BOUNDING-BOX]: side_length: %f, %f, %f", side_length_xy, side_length_xy, side_length_z);
  ROS_INFO("[BOUNDING-BOX]: starting_pose: %f, %f, %f", raytrace_starting_pose.x, raytrace_starting_pose.y, raytrace_starting_pose.z);
  visualization_msgs::Marker cube;
  cube.header.frame_id = "map";
  cube.header.stamp = ros::Time();
  cube.ns = "bounding_box";
  cube.id = 0;
  cube.type = visualization_msgs::Marker::CUBE;
  cube.action = visualization_msgs::Marker::ADD;
  cube.pose.position.x = raytrace_starting_pose.x;
  cube.pose.position.y = raytrace_starting_pose.y;
  cube.pose.position.z = raytrace_starting_pose.z;
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

  return cube;
}

/**
 * @brief updates the rays of a specific marker, catch intersection with current local map, find intersections
 * @todo list global intersections in array. todo: what to do with those intersections?
 * 
 * @param rays (Line_List)
 */
void updateRays(visualization_msgs::Marker &rays)
{
  if (rays.points.size() == 0)
    return;

  int num_updates = 0;

  // iterate over every 'line'
  for (int i = 1; i < rays.points.size(); i += 2)
  {
    // if the current ray is finished ( reached bounding box or sign switch in tsdf), skip it
    if (lines_finished[(i - 1) / 2])
    {
      continue;
    }
    
    num_updates++;

    auto &p1 = rays.points[i];
    auto &p2 = rays.points[i - 1];
    float length = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
    //ROS_INFO("Cur Vector length: %f", length);
    float factor = (length + lc_config.step_size) / length; // vector enlargement

    // enlarge "vector"
    p1.x = (p1.x - raytrace_starting_pose.x) * factor + raytrace_starting_pose.x; // translate to (0,0,0), enlarge, translate back
    p1.y = (p1.y - raytrace_starting_pose.y) * factor + raytrace_starting_pose.y; // translate to (0,0,0), enlarge, translate back
    p1.z = (p1.z - raytrace_starting_pose.z) * factor + raytrace_starting_pose.z; // translate to (0,0,0), enlarge, translate back

    // if we are out of the bounds of the local map, we want to set the the point directly on the bounding box. (calc relative enlargement factor)
    float fac_x = 10.0f, fac_y = 10.0f, fac_z = 10.0f; // set to 10, as we calc the min of these
    bool needs_resize = false;

    // we need to do this 3 times and find the smalles factor (the biggest adatpion) needed to get the ray back to the bounding box.
    if (p1.x > raytrace_starting_pose.x + side_length_xy / 2.0f || p1.x < raytrace_starting_pose.x - side_length_xy / 2.0f)
    {
      fac_x = abs((side_length_xy / 2.0f) / (p1.x - raytrace_starting_pose.x));
      needs_resize = true;
    }

    if (p1.y > raytrace_starting_pose.y + side_length_xy / 2.0f || p1.y < raytrace_starting_pose.y - side_length_xy / 2.0f)
    {
      fac_y = abs((side_length_xy / 2.0f) / (p1.y - raytrace_starting_pose.y));
      needs_resize = true;
    }

    if (p1.z > raytrace_starting_pose.z + side_length_z / 2.0f || p1.z < raytrace_starting_pose.z - side_length_z / 2.0f)
    {
      fac_z = abs((side_length_z / 2.0f) / (p1.z - raytrace_starting_pose.z));
      needs_resize = true;
    } 

    if(!needs_resize && local_map_ptr_.get()->value(p1.x * 1000.0f / MAP_RESOLUTION, p1.y * 1000.0f / MAP_RESOLUTION, p1.z * 1000.0f / MAP_RESOLUTION).value() < 600)
    {
      auto& tsdf = local_map_ptr_.get()->value(p1.x * 1000.0f / MAP_RESOLUTION, p1.y * 1000.0f / MAP_RESOLUTION, p1.z * 1000.0f / MAP_RESOLUTION);
      ROS_INFO("Value: %d", tsdf.value());
      tsdf.setIntersect(true);
      // the line doesnt need any further updates
      lines_finished[(i - 1) / 2] = true;
    }
    else if (needs_resize)
    {
      //determine biggest adaption
      float min_xy = std::min(fac_x, fac_y);
      
      float min_xyz = std::min(min_xy, fac_z);

      // resize to bb size
      p1.x = (p1.x - raytrace_starting_pose.x) * min_xyz + raytrace_starting_pose.x;
      p1.y = (p1.y - raytrace_starting_pose.y) * min_xyz + raytrace_starting_pose.y;
      p1.z = (p1.z - raytrace_starting_pose.z) * min_xyz + raytrace_starting_pose.z;

      // the line doesnt need any further updates
      lines_finished[(i - 1) / 2] = true;
    }
  }

  // done updating
  if(num_updates == 0)
  {
    ROS_INFO("Done Updating...");
    ROS_INFO("Displaying intersections");
    done_tracing = true;
  }
}

/**
 * @brief Function, which initializes the rays of a simulated laserscan using polar coordinates
 * 
 * @return visualization_msgs::Marker -> a marker containing the rays (as lines)
 */
visualization_msgs::Marker initRayMarkers()
{
  // tsdf sim
  visualization_msgs::Marker ray_marker_list;
  ray_marker_list.header.frame_id = "map";
  ray_marker_list.header.stamp = ros::Time();
  ray_marker_list.ns = "ray_list";
  ray_marker_list.id = 0;
  ray_marker_list.type = visualization_msgs::Marker::LINE_LIST;
  ray_marker_list.action = visualization_msgs::Marker::ADD;
  ray_marker_list.pose.position.x = 0;
  ray_marker_list.pose.position.y = 0;
  ray_marker_list.pose.position.z = 0;
  ray_marker_list.pose.orientation.x = 0.0;
  ray_marker_list.pose.orientation.y = 0.0;
  ray_marker_list.pose.orientation.z = 0.0;
  ray_marker_list.pose.orientation.w = 1.0;
  ray_marker_list.scale.x = lc_config.ray_size;
  ray_marker_list.scale.y = lc_config.ray_size;
  ray_marker_list.scale.z = lc_config.ray_size;
  ray_marker_list.color.a = 0.6; // Don't forget to set the alpha!
  ray_marker_list.color.r = 0.0;
  ray_marker_list.color.g = 0.0;
  ray_marker_list.color.b = 1.0;
  std::vector<geometry_msgs::Point> points; // for line list

  // simulate sensor
  const float start_degree = -(float)lc_config.opening_degree / 2.0f;
  const float fin_degree = (float)lc_config.opening_degree / 2.0f;
  const float x_res = 360.0f / (float)(lc_config.hor_res);
  const float y_res = (float)lc_config.opening_degree / (float)(lc_config.vert_res - 1); // assuming, that the fin degree is positive and start degree negative.

  ROS_INFO("Hor-Resolution: %f, Vertical resolution: %f (in degree)", x_res, y_res);

  // double for loop iterating over the specified resolution of the scanner (360 degrees horizontally, predefines angle vertically)
  for (float i = -180.0f; i < 180.0f; i += x_res)
  {
    for (float j = start_degree; j <= fin_degree; j += y_res)
    {
      // every ray has the same starting position
      points.push_back(geometry_msgs::Point(raytrace_starting_pose));

      // no we need to calc the respective points for each of the rays. scary.
      // done with two angles in sphere coordinates
      // formulas from http://wiki.ros.org/ainstein_radar/Tutorials/Tracking%20object%20Cartesian%20pose
      // and https://math.libretexts.org/Bookshelves/Calculus/Book%3A_Calculus_(OpenStax)/12%3A_Vectors_in_Space/12.7%3A_Cylindrical_and_Spherical_Coordinates
      geometry_msgs::Point ray_point;
      ray_point.x = raytrace_starting_pose.x + cos(i * M_PI / 180) * cos(j * M_PI / 180);
      ray_point.y = raytrace_starting_pose.y + sin(i * M_PI / 180) * cos(j * M_PI / 180); // oppsite angle
      ray_point.z = raytrace_starting_pose.z + sin(j * M_PI / 180);

      // resize the vectors to the length defined by config.step_size
      auto &p1 = ray_point;
      auto &p2 = raytrace_starting_pose;
      float length = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
      float factor = lc_config.step_size / length; // vector enlargement

      // ROS_INFO("INIT_RAYS: start length: %f, length factor; %f", length, factor);

      // enlarge "vector"
      ray_point.x = (p1.x - raytrace_starting_pose.x) * factor + raytrace_starting_pose.x; // translate to (0,0,0), enlarge, translate back
      ray_point.y = (p1.y - raytrace_starting_pose.y) * factor + raytrace_starting_pose.y; // translate to (0,0,0), enlarge, translate back
      ray_point.z = (p1.z - raytrace_starting_pose.z) * factor + raytrace_starting_pose.z; // translate to (0,0,0), enlarge, translate back

      points.push_back(ray_point);
    }
  }

  ROS_INFO("There are %d rays in the simulated scan", (int)(points.size() / 2));

  // update the lines finished vector, as we need to track, if a line is already finished
  lines_finished = std::vector<bool>(points.size() / 2, false);

  // attach to marker
  ray_marker_list.points = points;

  return ray_marker_list;
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

  auto& size = local_map_ptr_.get()->get_size();
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
  ROS_INFO("Reconfigure Request: %d %d",
           config.hor_res, config.vert_res);

  lc_config = config;

  // re-init the markers
  ray_markers = initRayMarkers();
  bb_marker = initBoundingBox();
  
  // reinitialize the map, as the intersections are now invalid
  initMaps();

  done_tracing = false;
  done_iteration = false;
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

  dynamic_reconfigure::Server<loop_closure::LoopClosureConfig> server;
  dynamic_reconfigure::Server<loop_closure::LoopClosureConfig>::CallbackType f;

  f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
  server.setCallback(f);

  // define stuff for raytracer

  // starting position
  raytrace_starting_pose.x = 0;
  raytrace_starting_pose.y = 0;
  raytrace_starting_pose.z = 0;

  // get markers
  auto pose_marker = initPoseMarker();
  cube_marker_list = initTSDFsimMarker();
  ray_markers = initRayMarkers();
  bb_marker = initBoundingBox();

  // init tsdf
  tsdf_map = initTSDFmarker();


  bb_publisher.publish(bb_marker);
  pose_publisher.publish(pose_marker);
  cube_publisher.publish(tsdf_map);

  ros::spinOnce();
  
  //ros loop
  while (ros::ok())
  {
    // publish the individual messages
    ray_publisher.publish(ray_markers);
    bb_publisher.publish(bb_marker);

    if(done_tracing && !done_iteration){
        ROS_INFO("Done tracing, displaying intersections from main loop..");
        tsdf_map = initTSDFmarker();
        cube_publisher.publish(tsdf_map);
        done_iteration = true;
    }

    // every iteration, the rays length is enlarged, until every ray has either cut the bounding box, or the localmap
    if(!done_tracing && !done_iteration)
    {
      updateRays(ray_markers);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}