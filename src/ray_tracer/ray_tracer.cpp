#include "ray_tracer.h"

RayTracer::RayTracer()
{
  // default
}

RayTracer::RayTracer(loop_closure::LoopClosureConfig *new_config, std::shared_ptr<LocalMap> local_map_in, Pose *start_pose)
{
  lc_config = new_config;
  local_map_ptr_ = local_map_in;
  current_pose = start_pose;
}

void RayTracer::start()
{

  // we casually ignore calls to this function, when
  if (lc_config == NULL || current_pose == NULL)
  {
    return;
  }

  ROS_INFO("[RayTracer] Started Tracing...");

  // before doing anything, we need to cleanup the data from the last run
  cleanup();

  // first initialize the rays with the current pose and config data
  ROS_INFO("[RayTracer] Intitializing Rays...");
  initRays();
  ROS_INFO("[RayTracer] Intitializing Rays done...");

  // now we initialized the "lines finished" - array and know exactly, when to stop updating the rays.
  // exactly when all rays are finished :D
  ROS_INFO("[RayTracer] Updating Rays...");

  /*while(finished_counter < rays.size())
  {
    CudaTracing::updateRays(&rays, current_pose, lc_config);
  }*/

  // and do some time measuring
  auto start_time = ros::Time::now();

  while (finished_counter < rays.size())
  {
    updateRays();
  }

  // more time measuring
  auto end_time = ros::Time::now();

  // calc duration
  auto duration = end_time - start_time;

  ROS_INFO("[RayTracer] Time Measurement updates only: %.2f ms", duration.toNSec() / 1000000.0f); // display time in ms, with two decimal points

  ROS_INFO("[RayTracer] Updating Rays done...");

  ROS_INFO("[RayTracer] Done Tracing...");
}

void RayTracer::initRays()
{

  // simulate sensor
  const float start_degree = -(float)lc_config->opening_degree / 2.0f;
  const float fin_degree = (float)lc_config->opening_degree / 2.0f;
  const float x_res = 360.0f / (float)(lc_config->hor_res);
  const float y_res = (float)lc_config->opening_degree / (float)(lc_config->vert_res - 1); // assuming, that the fin degree is positive and start degree negative.

  ROS_INFO("Hor-Resolution: %f, Vertical resolution: %f (in degree)", x_res, y_res);

  // double for loop iterating over the specified resolution of the scanner (360 degrees horizontally, predefines angle vertically)
  for (float i = -180.0f; i < 180.0f; i += x_res)
  {
    for (float j = start_degree; j <= fin_degree; j += y_res)
    {
      // no we need to calc the respective points for each of the rays. scary.
      // done with two angles in sphere coordinates
      // formulas from http://wiki.ros.org/ainstein_radar/Tutorials/Tracking%20object%20Cartesian%20pose
      // and https://math.libretexts.org/Bookshelves/Calculus/Book%3A_Calculus_(OpenStax)/12%3A_Vectors_in_Space/12.7%3A_Cylindrical_and_Spherical_Coordinates
      Eigen::Vector3f ray_point(
          current_pose->pos.x() + cos(i * M_PI / 180) * cos(j * M_PI / 180),
          current_pose->pos.y() + sin(i * M_PI / 180) * cos(j * M_PI / 180), // oppsite angle
          current_pose->pos.z() + sin(j * M_PI / 180));

      // now rotate using the 3d rotation matrix, translate to origin first and afterwards translate back.
      ray_point = current_pose->rotationMatrixFromQuaternion() * (ray_point - current_pose->pos) + current_pose->pos;

      // resize the vectors to the length defined by config.step_size
      auto &p1 = ray_point;
      auto &p2 = current_pose->pos;

      float length = (p1 - p2).norm(); // get length

      float factor = lc_config->step_size / length; // vector enlargement

      // enlarge "vector" by translating to (0,0,0), rotating it in space and putting it back alla
      ray_point = (p1 - current_pose->pos) * factor + current_pose->pos;

      // add to points list for ray marker (line list)
      rays.push_back(ray_point);
    }
  }

  ROS_INFO("There are %d rays in the simulated scan", (int)(rays.size()));
  // update the lines finished vector, as we need to track, if a line is already finished
  lines_finished = std::vector<bool>(rays.size(), false);
}

void RayTracer::updateRays()
{

  float side_length_xy = local_map_ptr_->get_size().x() * MAP_RESOLUTION / 1000.0f;
  float side_length_z = local_map_ptr_->get_size().z() * MAP_RESOLUTION / 1000.0f;

  // why would we update, when there are no rays?
  if (rays.size() == 0)
    return;

  // iterate over every 'line'
  for (int i = 0; i < rays.size(); i++)
  {
    // if the current ray is finished ( reached bounding box or sign switch in tsdf), skip it
    if (lines_finished[i])
    {
      continue;
    }

    auto &p1 = rays[i];
    auto &p2 = current_pose->pos;
    float length = (p1 - p2).norm();
    // ROS_INFO("Cur Vector length: %f", length);
    float factor = (length + lc_config->step_size) / length; // vector enlargement

    // enlarge "vector"
    p1 = (p1 - current_pose->pos) * factor + current_pose->pos; // translate to (0,0,0), enlarge, translate back

    // if we are out of the bounds of the local map, we want to set the the point directly on the bounding box. (calc relative enlargement factor)
    float fac_x = 10.0f, fac_y = 10.0f, fac_z = 10.0f; // set to 10, as we calc the min of these
    bool needs_resize = false;

    // we need to do this 3 times and find the smalles factor (the biggest adatpion) needed to get the ray back to the bounding box.
    if (p1.x() > current_pose->pos.x() + side_length_xy / 2.0f || p1.x() < current_pose->pos.x() - side_length_xy / 2.0f)
    {
      fac_x = abs((side_length_xy / 2.0f) / (p1.x() - current_pose->pos.x()));
      needs_resize = true;
    }

    if (p1.y() > current_pose->pos.y() + side_length_xy / 2.0f || p1.y() < current_pose->pos.y() - side_length_xy / 2.0f)
    {
      fac_y = abs((side_length_xy / 2.0f) / (p1.y() - current_pose->pos.y()));
      needs_resize = true;
    }

    if (p1.z() > current_pose->pos.z() + side_length_z / 2.0f || p1.z() < current_pose->pos.z() - side_length_z / 2.0f)
    {
      fac_z = abs((side_length_z / 2.0f) / (p1.z() - current_pose->pos.z()));
      needs_resize = true;
    }

    if (!needs_resize && local_map_ptr_.get()->value(p1.x() * 1000.0f / MAP_RESOLUTION, p1.y() * 1000.0f / MAP_RESOLUTION, p1.z() * 1000.0f / MAP_RESOLUTION).value() < 600)
    {
      auto &tsdf = local_map_ptr_.get()->value(p1.x() * 1000.0f / MAP_RESOLUTION, p1.y() * 1000.0f / MAP_RESOLUTION, p1.z() * 1000.0f / MAP_RESOLUTION);
      // ROS_INFO("Value: %d", tsdf.value());
      tsdf.setIntersect(true);
      // the line doesnt need any further updates
      lines_finished[i] = true;
      finished_counter++;
    }
    else if (needs_resize)
    {
      // determine biggest adaption
      float min_xy = std::min(fac_x, fac_y);

      float min_xyz = std::min(min_xy, fac_z);

      // resize to bb size
      p1.x() = (p1.x() - current_pose->pos.x()) * min_xyz + current_pose->pos.x();
      p1.y() = (p1.y() - current_pose->pos.y()) * min_xyz + current_pose->pos.y();
      p1.z() = (p1.z() - current_pose->pos.z()) * min_xyz + current_pose->pos.z();

      // the line doesnt need any further updates
      lines_finished[i] = true;
      finished_counter++;
    }
  }
}

void RayTracer::cleanup()
{
  // clear the finished lines array
  lines_finished.clear();

  // reset counter
  finished_counter = 0;

  // clear last runs rays
  rays.clear();
}

void RayTracer::update_pose(Pose *new_pose)
{
  /*// calculate how much the local map needs to be shifted
  Vector3f diff = new_pose->pos - current_pose->pos;

  // calc real word to global map coordinates

  std::cout << "Vector before pose update: " << diff << std::endl;

  Vector3i diff_map = (diff * 1000.0f / MAP_RESOLUTION).cast<int>();

  std::cout << "Vector after pose update: " << diff_map << std::endl;*/

  Eigen::Vector3i new_pose_map = (new_pose->pos * 1000.0f / MAP_RESOLUTION).cast<int>();

  // shift the local map
  //local_map_ptr_->shift(diff_map);
  local_map_ptr_->shift(new_pose_map);

  // update the pose
  current_pose = new_pose;
}

visualization_msgs::Marker RayTracer::get_ros_marker()
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
  ray_marker_list.scale.x = lc_config->ray_size;
  ray_marker_list.scale.y = lc_config->ray_size;
  ray_marker_list.scale.z = lc_config->ray_size;
  ray_marker_list.color.a = 0.6; // Don't forget to set the alpha!
  ray_marker_list.color.r = 0.0;
  ray_marker_list.color.g = 0.0;
  ray_marker_list.color.b = 1.0;
  std::vector<geometry_msgs::Point> points; // for line list

  // add lines of the trace to ros marker
  geometry_msgs::Point start;
  start.x = current_pose->pos.x();
  start.y = current_pose->pos.y();
  start.z = current_pose->pos.z();

  for (auto &point : rays)
  {
    geometry_msgs::Point ros_point;
    ros_point.x = point.x();
    ros_point.y = point.y();
    ros_point.z = point.z();

    // push start and end of ray
    points.push_back(start);

    points.push_back(ros_point);
  }

  ray_marker_list.points = points;

  return ray_marker_list;
}
