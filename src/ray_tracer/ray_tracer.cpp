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
    updateRaysNew(mode);
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

  // create a array for the current ray associations, with the size of the current rays
  current_ray_associations = std::vector<std::vector<Vector3i>>(rays.size());
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

void RayTracer::updateRaysNew(int mode)
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
      // this logic is kindof complicated, as many factors are involved. it is necassary to save (positive valued) tsdf cells
      // which might be associated to the current pose (with the current ray)
      // this logic should be able to let the ray tracer see through tight passages
      if (lines_finished[i] == RayStatus::INIT)
      {
        // interesting case: if the ray directly hits a negative valued tsdf cell, the ray is finished, as
        // this cell cannot have been seen from the current position.
        if (tsdf.value() < 0 && tsdf.weight() > 0)
        {
          // we just skip here, cause this case is not interesting to us
          continue;
        }
        else if (tsdf.value() < 600 && tsdf.weight() > 0) // 600 : tau * 1000 -> is TAU
        {
          // now that the ray hit it's first positvely valued cell, we update it's status
          lines_finished[i] = RayStatus::HIT;

          // save the tsdf cell, as we are not sure, if the cell is to be associated with the current pose
          current_ray_associations[i].push_back(real_to_map(p1));

          continue;
        }
      }
      else if (lines_finished[i] == RayStatus::HIT)
      {
        if (tsdf.value() < 0 && tsdf.weight() > 0)
        {
          // if we detect a negative valued cell with a weight in this mode, we switch the mode
          lines_finished[i] = RayStatus::ZERO_CROSSED;

          // associations and visualization
          if (mode == 0)
          {
            // add association and visualization for every saved cell of the current ray.
            for (Vector3i saved_cell : current_ray_associations[i])
            {
              auto &tsdf_tmp = local_map_ptr_->value(saved_cell);

              // set intersection
              tsdf_tmp.setIntersect(TSDFEntry::IntersectStatus::INT);

              // set association
              cur_association->addAssociation(saved_cell, tsdf_tmp);
            }

            // now clear
            current_ray_associations[i].clear();

            // add zero crossing visualization and association
            cur_association->addAssociation(real_to_map(p1), tsdf);
            tsdf.setIntersect(TSDFEntry::IntersectStatus::INT_ZERO);
          }
        }
        else if (!(tsdf.value() < 600 && tsdf.weight() > 0))
        {
          // if we are currently in hit mode, but the ray suddenly hits a meaningless cell, the status is
          // set back to the init state, as we just passed through empty space, but not hit any kind of zero crossing doing it
          // meaning: the data should ne be associated with the current pose
          lines_finished[i] = RayStatus::INIT;

          // clear the stored tsdf cells, as we now know those aren't supposed to be referenced to this scan (at least for now)
          current_ray_associations[i].clear();

          // continue with next ray
          continue;
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

void RayTracer::start_bresenham()
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

  // init bresenham
  init3DBresenham();
}

void RayTracer::init3DBresenham()
{
  // init localmap surfaces

  // simulate sensor
  const float start_degree = -(float)opening_degree / 2.0f;
  const float fin_degree = (float)opening_degree / 2.0f;
  const float x_res = 360.0f / (float)(hor_res);
  const float y_res = (float)opening_degree / (float)(vert_res - 1); // assuming, that the fin degree is positive and start degree negative.

  int zero_counter = 0;
  int global_match_count = 0;

  auto size = local_map_ptr_->get_size();

  
  Vector3f normal_bottom(0, 0, 1);
  Vector3f normal_top(0, 0, -1);

  Vector3f normal_left(0, 1, 0);
  Vector3f normal_right(0, -1, 0);

  Vector3f normal_front(-1, 0, 0);
  Vector3f normal_back(1, 0, 0);

  // -1 as the size of the local map is odd
  Vector3f s_bottom = current_pose->pos + map_to_real(Vector3i(0, 0, (size.z() - 1) / 2));
  Vector3f s_top = current_pose->pos + map_to_real(Vector3i(0, 0, -1 * (size.z() - 1) / 2));

  Vector3f s_left = current_pose->pos + map_to_real(Vector3i(0, (size.y() - 1) / 2, 0));
  Vector3f s_right = current_pose->pos + map_to_real(Vector3i(0, -1 * (size.y() - 1) / 2, 0));

  Vector3f s_front = current_pose->pos + map_to_real(Vector3i((size.x() - 1) / 2, 0, 0));
  Vector3f s_back = current_pose->pos + map_to_real(Vector3i(-1 * (size.x() - 1) / 2, 0, 0));

  Eigen::Vector3f bottom_int, top_int, left_int, right_int, front_int, back_int;

  // double for loop iterating over the specified resolution of the scanner (360 degrees horizontally, predefines angle vertically)
  // TODO: PARRALELIZE
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

      // calc the vector between the ray base and ray point
      Vector3f ray_vector = ray_point - current_pose->pos;

      // std::cout << "Ray Vector: " << ray_vector.x() << " | " << ray_vector.y() << " | " << ray_vector.z() << std::endl;

      // normalize the vector
      ray_vector.normalize();

      // std::cout << "Ray Vector Normalized: " << ray_vector.x() << " | " << ray_vector.y() << " | " << ray_vector.z() << std::endl;
      
      // now calculate the belonging cell at the outer most part of the localmap to start bresenham

      // bottom layer:
      bool bottom_intersected = linePlaneIntersection(bottom_int, ray_vector, current_pose->pos, normal_bottom, s_bottom);

      // top layer:
      bool top_intersected = linePlaneIntersection(top_int, ray_vector, current_pose->pos, normal_top, s_top);

      // left layer:
      bool left_intersected = linePlaneIntersection(left_int, ray_vector, current_pose->pos, normal_left, s_left);

      // right layer:
      bool right_intersected = linePlaneIntersection(right_int, ray_vector, current_pose->pos, normal_right, s_right);

      // front layer:
      bool front_intersected = linePlaneIntersection(front_int, ray_vector, current_pose->pos, normal_front, s_front);

      // back layer:
      bool back_intersected = linePlaneIntersection(back_int, ray_vector, current_pose->pos, normal_back, s_back);

      // that we have all the intersections, we need to find the closest postive one
      // TODO: instead of the following code, just use the inbounds method of the localmap
      std::vector<std::pair<Vector3f, bool>> intersections =
          {std::make_pair(bottom_int, bottom_intersected),
           std::make_pair(top_int, top_intersected),
           std::make_pair(left_int, left_intersected),
           std::make_pair(right_int, right_intersected),
           std::make_pair(front_int, front_intersected),
           std::make_pair(back_int, back_intersected)};

      Vector3f bresenham_vertex;

      // min length of the array is what we are looking for
      // set this to infinity at first
      float min_length = std::numeric_limits<float>::max();

      for (auto intersection : intersections)
      {
        // if the ray was parallel continue with the next one or: t was negative
        if (!intersection.second)
        {
          continue;
        }

        global_match_count++;

        // the length is the length of the vector between the found intersection and the position of the artifical ray tracer
        float length = (intersection.first - current_pose->pos).norm();

        if (length < min_length)
        {
          min_length = length;
          bresenham_vertex = intersection.first;
        }
      }

      bresenham_cells.push_back(real_to_map(bresenham_vertex));

      if (real_to_map(bresenham_vertex) == Vector3i(0, 0, 0))
      {
        zero_counter++;
      }
    }
  }

  std::cout << "Vectors: " << std::endl << s_back << std::endl;


  std::cout << "Global Match count avg: " << (float)global_match_count / bresenham_cells.size() << std::endl;
  std::cout << "got " << zero_counter << " cells with zero vector" << std::endl;

  std::cout << "Found " << bresenham_cells.size() << " cells" << std::endl;
  // update the lines finished vector, as we need to track, if a line is already finished
  // initially, all rays have a status of OK, meaning they neither passed a zero crossing, nor are already finished.
  lines_finished = std::vector<RayStatus>(bresenham_cells.size(), RayStatus::INIT);

  // create a array for the current ray associations, with the size of the current rays
  current_ray_associations = std::vector<std::vector<Vector3i>>(bresenham_cells.size());
}

bool RayTracer::linePlaneIntersection(Vector3f &intersection, Vector3f ray_vector,
                                      Vector3f ray_origin, Vector3f plane_normal, Vector3f plane_coord)
{
  // get d value
  float d = plane_normal.dot(plane_coord);

  std::cout << "D: " << d << std::endl;

  // check if the plane is parallel to the the ray, if so return false
  if (plane_normal.dot(ray_vector) == 0)
  {
    return false;
  }

  // Compute the X value for the directed line ray intersecting the plane
  float t = (d - plane_normal.dot(ray_origin)) / plane_normal.dot(ray_vector);

  if (t == 0)
  {
    std::cout << "T is NULL" << std::endl;
  }

  // this is special: we only want to look in the direction of the vector, a negative t suggests,
  // that the intersection is indeed the opposite way, which is an unwanted scenario handled similar to a parallel ray
  if (t <= 0)
  {
    return false;
  }

  intersection = ray_origin + ray_vector * t; // Make sure your ray vector is normalized

  return true;
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

  // clear bresenham data
  bresenham_cells.clear();
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

visualization_msgs::Marker RayTracer::get_bresenham_intersection_marker()
{
  // bresenham marker
  visualization_msgs::Marker bresenham_intersections;
  bresenham_intersections.header.frame_id = "map";
  bresenham_intersections.header.stamp = ros::Time();
  bresenham_intersections.ns = "bresenham_intersections";
  bresenham_intersections.id = 0;
  bresenham_intersections.type = visualization_msgs::Marker::POINTS;
  bresenham_intersections.action = visualization_msgs::Marker::ADD;
  bresenham_intersections.scale.x = MAP_RESOLUTION * 1.0 * 0.001;
  bresenham_intersections.scale.y = MAP_RESOLUTION * 1.0 * 0.001;
  bresenham_intersections.scale.z = MAP_RESOLUTION * 1.0 * 0.001;
  bresenham_intersections.color.a = 0.6; // Don't forget to set the alpha!
  bresenham_intersections.color.r = 0.0;
  bresenham_intersections.color.g = 0.0;
  bresenham_intersections.color.b = 1.0;

  for (Vector3i bresenham_cell : bresenham_cells)
  {
    // get the exact position of the cell (in localmap coordinates -> rounding)
    bresenham_intersections.points.push_back(type_transform::eigen_point_to_ros_point(map_to_real(bresenham_cell)));
  }

  return bresenham_intersections;
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
