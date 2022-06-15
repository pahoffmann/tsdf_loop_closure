#include "ray_tracer.h"

RayTracer::RayTracer(loop_closure::LoopClosureConfig *new_config, std::shared_ptr<LocalMap> local_map_in, std::shared_ptr<GlobalMap> global_map_in, Pose *start_pose)
{
  lc_config = new_config;
  local_map_ptr_ = local_map_in;
  global_map_ptr_ = global_map_in;
  current_pose = start_pose;
}

RayTracer::RayTracer(lc_options_reader *new_options, std::shared_ptr<LocalMap> local_map_in, std::shared_ptr<GlobalMap> global_map_in)
{
  options = new_options;
  local_map_ptr_ = local_map_in;
  global_map_ptr_ = global_map_in;
  current_pose = NULL;
}

float RayTracer::start(int mode)
{

  // when either of these is NULL, we might as well just throw a logic error.
  if ((lc_config == NULL && options == NULL) || current_pose == NULL)
  {
    throw std::logic_error("[RayTracer] - Major Error, no configuration data delivered to ray-tracer");
  }
  else if (current_pose == NULL)
  {
    throw std::invalid_argument("[RayTracer] The RayTracer needs a pose to be able to start tracing.");
  }

  opening_degree = options != NULL ? options->get_opening_degree() : lc_config->opening_degree;
  hor_res = options != NULL ? options->get_hor_res() : lc_config->hor_res;
  vert_res = options != NULL ? options->get_vert_res() : lc_config->vert_res;
  step_size = options != NULL ? options->get_step_size() : lc_config->step_size;

  side_length_xy = (local_map_ptr_->get_size().x() - 1) * MAP_RESOLUTION / 1000.0f;
  side_length_z = (local_map_ptr_->get_size().z() - 1) * MAP_RESOLUTION / 1000.0f;

  ray_size = options != NULL ? options->get_ray_size() : lc_config->ray_size;

  // just for better readablity
  std::cout << std::endl;

  std::cout << "[RayTracer] Started Tracing..." << std::endl;

  // before doing anything, we need to cleanup the data from the last run
  cleanup();

  // first initialize the rays with the current pose and config data
  initRays();

  // now we initialized the "lines finished" - array and know exactly, when to stop updating the rays.
  // exactly when all rays are finished :D

  // and do some time measuring
  auto start_time = ros::Time::now();

  int num_iterations = 0;

  while (finished_counter < rays.size())
  {
    updateRays(mode);
    num_iterations++;
    // std::cout << "Num_Iterations: " << num_iterations << " | Finished Counter: " << finished_counter << " | num_rays: " << rays.size() << std::endl;
  }

  // more time measuring
  auto end_time = ros::Time::now();

  // calc duration
  auto duration = end_time - start_time;

  std::cout << std::fixed;
  std::cout << std::setprecision(2);
  std::cout << "[RayTracer] Time Measurement updates only: " << duration.toNSec() / 1000000.0f << " ms" << std::endl; // display time in ms, with two decimal points

  std::cout << "[RayTracer] Done Tracing..." << std::endl;

  std::cout << std::endl;

  return (float)num_good / (float)num_not_good;
}

void RayTracer::initRays()
{
  // simulate sensor
  const float start_degree = -(float)opening_degree / 2.0f;
  const float fin_degree = (float)opening_degree / 2.0f;
  const float x_res = 360.0f / (float)(hor_res);
  const float y_res = (float)opening_degree / (float)(vert_res - 1); // assuming, that the fin degree is positive and start degree negative.

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

      float factor = step_size / length; // vector enlargement

      // enlarge "vector" by translating to (0,0,0), rotating it in space and putting it back alla
      ray_point = (p1 - current_pose->pos) * factor + current_pose->pos;

      // add to points list for ray marker (line list)
      rays.push_back(ray_point);
    }
  }

  // update the lines finished vector, as we need to track, if a line is already finished
  // initially, all rays have a status of OK, meaning they neither passed a zero crossing, nor are already finished.
  lines_finished = std::vector<RayStatus>(rays.size(), RayStatus::INIT);
}

void RayTracer::updateRays(int mode)
{
  // why would we update, when there are no rays? This is more of a fatal thing here.
  if (rays.size() == 0)
  {
    throw std::logic_error("[RayTracer] Logic Error: when trying to update the rays, there were none. this should never happen.");
  }

  // iterate over every 'line'
  for (int i = 0; i < rays.size(); i++)
  {
    // if the current ray is finished ( reached bounding box or sign switch in tsdf), skip it
    if (lines_finished[i] == RayStatus::FINISHED)
    {
      continue;
    }

    // get the two ray-points and calculate the current rays length
    auto &p1 = rays[i];
    auto &p2 = current_pose->pos;
    float length = (p1 - p2).norm();
    float factor = (length + step_size) / length; // vector enlargement

    // enlarge "vector"
    p1 = (p1 - current_pose->pos) * factor + current_pose->pos; // translate to (0,0,0), enlarge, translate back

    // if we are out of the bounds of the local map, we want to set the the point directly on the bounding box. (calc relative enlargement factor)
    float fac_x = 10.0f, fac_y = 10.0f, fac_z = 10.0f; // set to 10, as we calc the min of these
    bool needs_resize = false;

    // we need to do this 3 times and find the smalles factor (the biggest adatpion) needed to get the ray back to the bounding box.
    // this is useful for visualization, but might not be useful for a production environment, at least in terms of the resizing which is done.
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

    try
    {
      // if the current ray surpasses the border of the bounding box, the ray is finished
      if (needs_resize)
      {
        // determine biggest adaption
        float min_xy = std::min(fac_x, fac_y);

        float min_xyz = std::min(min_xy, fac_z);

        // resize to bb size
        p1 = (p1 - current_pose->pos) * min_xyz + current_pose->pos;

        lines_finished[i] = RayStatus::FINISHED;
        finished_counter++;

        if (mode == 1)
        {
          num_not_good++;
        }

        // no need to proceed further
        continue;
      }

      // now we can obtain the current tsdf value, as we are sure the ray is inbounds the bounding box (local map)
      auto &tsdf = local_map_ptr_.get()->value(real_to_map(p1));

      // now check if a status change is necessary.
      if (lines_finished[i] == RayStatus::INIT)
      {
        // interesting case: if the ray directly hits a negative valued tsdf cell, the ray is finished, as
        // this cell cannot have been seen from the current position.
        if (tsdf.value() < 0 && tsdf.weight() > 0)
        {
          lines_finished[i] = RayStatus::FINISHED;
          finished_counter++;

          if (mode == 1)
          {
            num_not_good++;
          }
        }
        else if (tsdf.value() < 600 && tsdf.weight() > 0) // FIXME: how is the "600" calculated? maybe also an attribute for the local map req?
        {
          // now that the ray hit it's first positvely valued cell, we update it's status

          lines_finished[i] = RayStatus::HIT;

          if (mode == 0)
          {
            cur_association->addAssociation(real_to_map(p1), tsdf);
            tsdf.setIntersect(TSDFEntry::IntersectStatus::INT);
          }
        }
      }
      else if (lines_finished[i] == RayStatus::HIT)
      {
        if (tsdf.value() < 0 && tsdf.weight() > 0)
        {
          // if we detect a negative valued cell with a weight in this mode, we switch the mode
          lines_finished[i] = RayStatus::ZERO_CROSSED;

          if (mode == 0)
          {
            cur_association->addAssociation(real_to_map(p1), tsdf);
            tsdf.setIntersect(TSDFEntry::IntersectStatus::INT_ZERO);
          }
        }
        else if (!(tsdf.value() < 600 && tsdf.weight() > 0))
        {
          // if we are currently in hit mode, but the ray suddenly hits a meaningless cell, the tracing is also finished.
          lines_finished[i] = RayStatus::FINISHED;
          finished_counter++;

          if (mode == 1)
          {
            num_not_good++;
          }
        }
        else
        {
          if (mode == 0)
          {
            cur_association->addAssociation(real_to_map(p1), tsdf);
            tsdf.setIntersect(TSDFEntry::IntersectStatus::INT);
          }
        }
      }
      else if (lines_finished[i] == RayStatus::ZERO_CROSSED)
      {
        // if we have already had a sign change in tsdf and the value gets positive again, we are done.
        // else, we are still in the negative value range and therefore, we add the association and mark the intersection in the local map
        if (!(tsdf.value() < 0 && tsdf.weight() > 0))
        {
          lines_finished[i] = RayStatus::FINISHED;
          finished_counter++;

          if (mode == 1)
          {
            num_good++;
          }
        }
        else
        {
          if (mode == 0)
          {
            cur_association->addAssociation(real_to_map(p1), tsdf);
            tsdf.setIntersect(TSDFEntry::IntersectStatus::INT_NEG);
          }
        }
      }
    }
    catch (...)
    {
      Eigen::Vector3f globalMapPos = local_map_ptr_->get_pos().cast<float>();
      globalMapPos *= MAP_RESOLUTION / 1000.0f;
      std::cout << "[RayTracer] Error while checking the local map values for local-map with global pose " << globalMapPos << std::endl
                << "Pos checked: " << p1 << std::endl
                << "Distanze between center and checked pos: " << (p1 - globalMapPos).norm() << std::endl;

      throw std::logic_error("[RAY_TRACER] If this fires, there is a huge bug in your code m8.");
    }
  }
}

void RayTracer::cleanup()
{
  // clear the finished lines array
  lines_finished.clear();

  // reset counter
  finished_counter = 0;
  num_good = 0;
  num_not_good = 0;

  // clear last runs rays
  rays.clear();
}

void RayTracer::update_pose(Pose *new_pose)
{
  // calc the new localmap pose
  Eigen::Vector3i new_pose_map = real_to_map(new_pose->pos);

  // shift the local map
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
  ray_marker_list.scale.x = ray_size;
  ray_marker_list.scale.y = ray_size;
  ray_marker_list.scale.z = ray_size;
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

/**
 * @todo TODO: test this code piece
 */
bool RayTracer::is_visible(Pose &a, Pose &b)
{

  // easiest case: the tsdf cell values for Pose a and b have different signs. -> the result of a multiplication of both values should not be negative
  if (global_map_ptr_->get_value(real_to_map(a.pos)).value() * global_map_ptr_->get_value(real_to_map(b.pos)).value() < 0)
  {
    std::cout << "[RayTracer - is_visible]: Different signs of the tsdf values in both poses" << std::endl;
    return false;
  }

  // else, we actually need to cast a ray and check if there has been a zero crossing, which basically means that the poses are not visible, because there is a wall
  // in the way.

  float distance = (b.pos - a.pos).norm();

  // calculate the number of steps aka, how many steps of size "step_size" need to be taken in order to get from point a to point b
  int num_steps = std::floor(distance / step_size);

  // check if the value from a is positive or negative to check visibility later
  bool is_neg;

  if (global_map_ptr_->get_value(real_to_map(a.pos)).value() < 0)
  {
    is_neg = true;
  }
  else
  {
    is_neg = false;
  }

  Vector3f cur = a.pos;

  for (int i = 0; i < num_steps; i++)
  {
    float length = (cur - a.pos).norm(); // get length

    std::cout << "current length: " << length << std::endl;

    float factor = step_size / length; // vector enlargement

    // enlarge "vector" by translating to (0,0,0), enlarging it from there and translating it back
    cur = (cur - a.pos) * factor + a.pos;

    auto value = global_map_ptr_->get_value(real_to_map(cur)).value();

    if (is_neg && value > 0)
    {
      std::cout << "[RayTracer: is_visible] - There has been a sign switch from negative to positive.." << std::endl;
      return false;
    }
    else if (!is_neg && value < 0)
    {
      std::cout << "[RayTracer: is_visible] - There has been a sign switch from positive to negative.." << std::endl;
      return false;
    }
  }

  // if the casted ray from a hits position b without encountering a sign switch, pose b is visible from pose a and vice versa
  return true;
}
