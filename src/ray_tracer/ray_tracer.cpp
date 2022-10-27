#include <loop_closure/ray_tracer/ray_tracer.h>

RayTracer::RayTracer(LoopClosureParams &lc_params, std::shared_ptr<LocalMap> local_map_in, std::shared_ptr<GlobalMap> global_map_in)
{
  params = lc_params;
  local_map_ptr_ = local_map_in;
  global_map_ptr_ = global_map_in;
  current_pose = NULL;

  // define the side lengths of the localmap
  side_length_x = (local_map_ptr_->get_size().x() - 1) * MAP_RESOLUTION / 1000.0f;
  side_length_y = (local_map_ptr_->get_size().x() - 1) * MAP_RESOLUTION / 1000.0f;
  side_length_z = (local_map_ptr_->get_size().z() - 1) * MAP_RESOLUTION / 1000.0f;
}

float RayTracer::start(int mode)
{
  // and do some time measuring
  auto start_time = ros::Time::now();

  // when no pose has been set before tracing, we throw an error
  if (current_pose == NULL)
  {
    throw std::invalid_argument("[RayTracer] The RayTracer needs a pose to be able to start tracing.");
  }

  // just for better readablity
  std::cout << std::endl;

  std::cout << "[RayTracer] Started Tracing..." << std::endl;

  // before doing anything, we need to cleanup the data from the last run
  cleanup();

  // first initialize the rays with the current pose and config data
  initRays();

  // now we initialized the "lines finished" - array and know exactly, when to stop updating the rays.
  // exactly when all rays are finished :D

  int num_iterations = 0;

  while (finished_counter < rays.size())
  {
    updateRaysNew(mode);
    num_iterations++;
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

void RayTracer::initRays(bool use_icp_params)
{
  // simulate sensor

  const float start_degree = use_icp_params
                                 ? -(float)params.ray_tracer.opening_degree_icp / 2.0f
                                 : -(float)params.ray_tracer.opening_degree / 2.0f;
  const float fin_degree = use_icp_params
                               ? (float)params.ray_tracer.opening_degree_icp / 2.0f
                               : (float)params.ray_tracer.opening_degree / 2.0f;

  const float x_res = use_icp_params
                          ? 360.0f / (float)(params.ray_tracer.hor_res_icp)
                          : 360.0f / (float)(params.ray_tracer.hor_res);

  // assuming, that the fin degree is positive and start degree negative.
  const float y_res = use_icp_params
                          ? (float)params.ray_tracer.opening_degree_icp / (float)(params.ray_tracer.vert_res_icp - 1)
                          : (float)params.ray_tracer.opening_degree / (float)(params.ray_tracer.vert_res - 1);

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

      float factor = params.ray_tracer.step_size / length; // vector enlargement

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
    float factor = (length + params.ray_tracer.step_size) / length; // vector enlargement

    // enlarge "vector"
    p1 = (p1 - current_pose->pos) * factor + current_pose->pos; // translate to (0,0,0), enlarge, translate back

    // if we are out of the bounds of the local map, we want to set the the point directly on the bounding box. (calc relative enlargement factor)
    float fac_x = 10.0f, fac_y = 10.0f, fac_z = 10.0f; // set to 10, as we calc the min of these
    bool needs_resize = false;

    // we need to do this 3 times and find the smalles factor (the biggest adatpion) needed to get the ray back to the bounding box.
    // this is useful for visualization, but might not be useful for a production environment, at least in terms of the resizing which is done.
    if (p1.x() > current_pose->pos.x() + side_length_x / 2.0f || p1.x() < current_pose->pos.x() - side_length_x / 2.0f)
    {
      fac_x = abs((side_length_x / 2.0f) / (p1.x() - current_pose->pos.x()));
      needs_resize = true;
    }

    if (p1.y() > current_pose->pos.y() + side_length_y / 2.0f || p1.y() < current_pose->pos.y() - side_length_y / 2.0f)
    {
      fac_y = abs((side_length_y / 2.0f) / (p1.y() - current_pose->pos.y()));
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
        else if (tsdf.value() < params.map.tau && tsdf.weight() > 0)
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
        else if (!(tsdf.value() < params.map.tau && tsdf.weight() > 0))
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
            current_ray_associations[i].push_back(real_to_map(p1));
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
  auto start = ros::Time::now();

  // if no pose has been set, we throw an error
  if (current_pose == NULL)
  {
    throw std::invalid_argument("[RayTracer] The RayTracer needs a pose to be able to start tracing.");
  }

  // just for better readablity
  std::cout << std::endl;

  std::cout << "[RayTracer] Started Tracing..." << std::endl;

  // before doing anything, we need to cleanup the data from the last run
  cleanup();

  // init bresenham
  init3DBresenham();

  // now perform the bresenham algorithm based on the initialization
  perform3DBresenham();

  auto end = ros::Time::now();

  auto duration = (end - start);

  std::cout << std::fixed;
  std::cout << std::setprecision(2);
  std::cout << "[RayTracer] Time Measurement Bresenham: " << duration.toNSec() / 1000000.0f << " ms" << std::endl; // display time in ms, with two decimal points
}

void RayTracer::init3DBresenham()
{
  // init localmap surfaces

  // simulate sensor
  const float start_degree = -(float)params.ray_tracer.opening_degree / 2.0f;
  const float fin_degree = (float)params.ray_tracer.opening_degree / 2.0f;
  const float x_res = 360.0f / (float)(params.ray_tracer.hor_res);
  const float y_res = (float)params.ray_tracer.opening_degree / (float)(params.ray_tracer.vert_res - 1); // assuming, that the fin degree is positive and start degree negative.

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
  //#pragma omp parallel for private()
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

      // normalize the vector
      ray_vector.normalize();

      // now calculate the belonging cell at the outer most part of the localmap to start bresenham
      // TODO: do this differently - just use direction vector, don't initialize.

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

      // now we push the vertex to array, here we need to transform the real world vertex to a vertex
      // in local map coordinates - this needs to be done with the real_to_map_relative function, which
      // makes sure, that the point is rounded towards the center of the local map
      // by transforming into the origin
      // hereby, no indexing errors occur - without this, due to c++ nature, the values are rounded towards zero, which can
      // lead to cells out of bounds of the local map
      bresenham_cells.push_back(real_to_map_relative(bresenham_vertex, current_pose->pos));

      if (real_to_map_relative(bresenham_vertex, current_pose->pos) == Vector3i(0, 0, 0))
      {
        zero_counter++;
      }
    }
  }

  // this may vary for the same localmap and parametrization, because of the pose of the laserscanner.
  // if it is slightly rotated, this may actually be 3, meaning there were no lines parallel to any of the planes
  std::cout << "Global Match count avg: " << (float)global_match_count / bresenham_cells.size() << std::endl;

  // update the lines finished vector, as we need to track, if a line is already finished
  // initially, all rays have a status of OK, meaning they neither passed a zero crossing, nor are already finished.
  lines_finished = std::vector<RayStatus>(bresenham_cells.size(), RayStatus::INIT);

  // create a array for the current ray associations, with the size of the current rays
  current_ray_associations = std::vector<std::vector<Vector3i>>(bresenham_cells.size());
}

void RayTracer::perform3DBresenham()
{

  // if trying to use openmp, this should be marked as private. as well as more properties below
  std::vector<Vector3i> tmp_cells;

  int dx, dy, dz;

  // do bresenham for every ray
  for (int i = 0; i < bresenham_cells.size(); i++)
  {
    auto &cell = bresenham_cells[i];
    Vector3i bresenham_start = real_to_map(current_pose->pos);

    //  from here each ray is updated independently

    // calculate differences between each axis, calc direction
    int dx = std::abs(cell.x() - bresenham_start.x()), sx = bresenham_start.x() < cell.x() ? 1 : -1;
    int dy = std::abs(cell.y() - bresenham_start.y()), sy = bresenham_start.y() < cell.y() ? 1 : -1;
    int dz = std::abs(cell.z() - bresenham_start.z()), sz = bresenham_start.z() < cell.z() ? 1 : -1;

    // calculate maximum difference
    int max_diff = std::max(dx, std::max(dy, dz));
    int index = max_diff;

    Vector3i cell_tmp(max_diff / 2, max_diff / 2, max_diff / 2);

    // while the line is not finished
    while (lines_finished[i] != RayStatus::FINISHED)
    {
      auto &tsdf = local_map_ptr_->value(bresenham_start);

      // now do switch case

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
          lines_finished[i] = RayStatus::FINISHED;
          continue;
        }
        else if (tsdf.value() < params.map.tau && tsdf.weight() > 0)
        {
          // now that the ray hit it's first positvely valued cell, we update it's status
          lines_finished[i] = RayStatus::HIT;

          // save the tsdf cell, as we are not sure, if the cell is to be associated with the current pose
          tmp_cells.push_back(bresenham_start);
        }
      }
      else if (lines_finished[i] == RayStatus::HIT)
      {
        if (tsdf.value() < 0 && tsdf.weight() > 0)
        {
          // if we detect a negative valued cell with a weight in this mode, we switch the mode
          lines_finished[i] = RayStatus::ZERO_CROSSED;

          // associations and visualization

          // add association and visualization for every saved cell of the current ray.
          for (Vector3i saved_cell : tmp_cells)
          {
            auto &tsdf_tmp = local_map_ptr_->value(saved_cell);

            // set intersection
            tsdf_tmp.setIntersect(TSDFEntry::IntersectStatus::INT);

            // set association
            cur_association->addAssociation(saved_cell, tsdf_tmp);
          }

          // now clear
          tmp_cells.clear();

          // add zero crossing visualization and association
          cur_association->addAssociation(bresenham_start, tsdf);
          tsdf.setIntersect(TSDFEntry::IntersectStatus::INT_ZERO);
        }
        else if (!(tsdf.value() < params.map.tau && tsdf.weight() > 0))
        {
          // if we are currently in hit mode, but the ray suddenly hits a meaningless cell, the status is
          // set back to the init state, as we just passed through empty space, but not hit any kind of zero crossing doing it
          // meaning: the data should ne be associated with the current pose
          lines_finished[i] = RayStatus::INIT;

          // clear the stored tsdf cells, as we now know those aren't supposed to be referenced to this scan (at least for now)
          tmp_cells.clear();
        }
        else
        {
          tmp_cells.push_back(bresenham_start);
        }
      }
      else if (lines_finished[i] == RayStatus::ZERO_CROSSED)
      {
        // if we have already had a sign change in tsdf and the value gets positive again, we are done.
        // else, we are still in the negative value range and therefore, we add the association and mark the intersection in the local map
        // [TODO] we may also need to stop here, if the distance between the current cell and the cell of the zero crossing is larger
        //        than the distance defined by tau (as the values further out do not belong to this pose)
        if (!(tsdf.value() < 0 && tsdf.weight() > 0))
        {
          lines_finished[i] = RayStatus::FINISHED;
          finished_counter++;

          continue;
        }
        else
        {
          cur_association->addAssociation(bresenham_start, tsdf);
          tsdf.setIntersect(TSDFEntry::IntersectStatus::INT_NEG);
        }
      }

      // end switch case

      // anchor
      if (index-- == 0)
      {
        break;
      }

      cell_tmp -= Vector3i(dx, dy, dz);

      // update vectors
      if (cell_tmp.x() < 0)
      {
        cell_tmp.x() += max_diff;
        bresenham_start.x() += sx;
      }
      if (cell_tmp.y() < 0)
      {
        cell_tmp.y() += max_diff;
        bresenham_start.y() += sy;
      }
      if (cell_tmp.z() < 0)
      {
        cell_tmp.z() += max_diff;
        bresenham_start.z() += sz;
      }
    }

    tmp_cells.clear();
  }
}

void RayTracer::local_removal(Pose *pose)
{
  std::cout << "Updating the pose to " << std::endl
            << *pose << std::endl;
  update_pose(pose);

  std::cout << "Cleaning data from previous runs" << std::endl;
  // cleanup everything from previous runs
  cleanup();

  std::cout << "initializing rays... " << std::endl;
  // initialize the rays
  initRays();

  std::cout << "Tracing over " << rays.size() << " rays" << std::endl;

  std::cout << "Default entry: [" << params.map.tau << " | " << 0 << "]" << std::endl;
  TSDFEntry default_entry(params.map.tau, 0); // default tsdf entry used to "reset" old cell locations

  // why would we update, when there are no rays? This is more of a fatal thing here.
  if (rays.size() == 0)
  {
    throw std::logic_error("[RayTracer] Logic Error: when trying to update the rays, there were none. this should never happen.");
  }

  // iterate over every 'line'

  bool first_iteration = true;
  int delete_counter = 0;

  while (finished_counter != lines_finished.size())
  {
    if (!first_iteration)
    {
      std::cout << "\033[A\33[2K\r";
    }
    else
    {
      first_iteration = false;
    }

    float percent = ((float)finished_counter / (float)lines_finished.size()) * 100.0f;

    std::cout << std::fixed;
    std::cout << std::setprecision(2);
    std::cout << "[Finished]: " << percent << " \% of the rays in this iteration" << std::endl;
    //#pragma omp parallel for schedule(static) default(shared)
    for (int i = 0; i < rays.size(); i++)
    {
      // if the current ray is finished ( reached bounding box or sign switch in tsdf), skip it
      if (lines_finished[i] == RayStatus::FINISHED)
      {
        continue;
      }

      // get the two ray-points and calculate the current rays length
      auto &p1 = rays[i];
      auto p2 = current_pose->pos;
      float length = (p1 - p2).norm();
      float factor = (length + params.ray_tracer.step_size) / length; // vector enlargement

      // enlarge "vector"
      p1 = (p1 - current_pose->pos) * factor + current_pose->pos; // translate to (0,0,0), enlarge, translate back

      // if we are out of the bounds of the local map, we want to set the the point directly on the bounding box. (calc relative enlargement factor)
      float fac_x = 10.0f, fac_y = 10.0f, fac_z = 10.0f; // set to 10, as we calc the min of these
      bool needs_resize = false;

      // we need to do this 3 times and find the smalles factor (the biggest adatpion) needed to get the ray back to the bounding box.
      // this is useful for visualization, but might not be useful for a production environment, at least in terms of the resizing which is done.
      if (p1.x() > current_pose->pos.x() + side_length_x / 2.0f || p1.x() < current_pose->pos.x() - side_length_x / 2.0f)
      {
        fac_x = abs((side_length_x / 2.0f) / (p1.x() - current_pose->pos.x()));
        needs_resize = true;
      }

      if (p1.y() > current_pose->pos.y() + side_length_y / 2.0f || p1.y() < current_pose->pos.y() - side_length_y / 2.0f)
      {
        fac_y = abs((side_length_y / 2.0f) / (p1.y() - current_pose->pos.y()));
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
          else if (tsdf.value() < params.map.tau && tsdf.weight() > 0)
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

            // add association and visualization for every saved cell of the current ray.
            for (Vector3i saved_cell : current_ray_associations[i])
            {
              TSDFEntry &tsdf_tmp = local_map_ptr_->value(saved_cell);
              // std::cout << "Before1: " << tsdf_tmp.value() << " | " << tsdf_tmp.weight() << std::endl;
              // tsdf_tmp.value(default_entry.value());
              // tsdf_tmp.weight(default_entry.weight());
              // std::cout << "After1: " << tsdf_tmp.value() << " | " << tsdf_tmp.weight() << std::endl;

              // local_map_ptr_->value(saved_cell) = default_entry;
            }

            // tsdf.value(default_entry.value());
            // tsdf.weight(default_entry.weight());

            // now clear
            current_ray_associations[i].clear();
          }
          else if (!(tsdf.value() < params.map.tau && tsdf.weight() > 0))
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
            current_ray_associations[i].push_back(real_to_map(p1));
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
          }
          else
          {
            // local_map_ptr_->value(real_to_map(p1)) = default_entry;
            // std::cout << "Before: " << tsdf.value() << " | " << tsdf.weight() << std::endl;
            // tsdf.value(default_entry.value());
            // tsdf.weight(default_entry.weight());
            // std::cout << "After: " << tsdf.value() << " | " << tsdf.weight() << std::endl;
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

  local_map_ptr_->write_back();

  std::cout << "After small update size: " << global_map_ptr_->get_full_data().size() << std::endl;
}

pcl::PointCloud<PointType>::Ptr RayTracer::approximate_pointcloud(Pose *pose)
{
  update_pose(pose);

  // and do some time measuring
  auto start_time = ros::Time::now();

  // when either of these is NULL, we might as well just throw a logic error.
  if (current_pose == NULL)
  {
    throw std::invalid_argument("[RayTracer] The RayTracer needs a pose to be able to start tracing.");
  }

  // just for better readablity
  std::cout << std::endl;

  std::cout << "[RayTracer] Started approximating a pointcloud..." << std::endl;

  // before doing anything, we need to cleanup the data from the last run
  cleanup();

  // first initialize the rays with the current pose and config data
  initRays(true);

  // now we initialized the "lines finished" - array and know exactly, when to stop updating the rays.
  // exactly when all rays are finished :D

  // std::vector<Vector3f> surface_points;
  pcl::PointCloud<PointType>::Ptr pointcloud;
  pointcloud.reset(new pcl::PointCloud<PointType>());

  while (finished_counter < lines_finished.size())
  {
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
      float factor = (length + params.ray_tracer.step_size) / length; // vector enlargement

      // save old pos, which is later used to check for zero crossings
      Vector3i old_pos = real_to_map(p1);

      // enlarge "vector"
      p1 = (p1 - current_pose->pos) * factor + current_pose->pos; // translate to (0,0,0), enlarge, translate back

      Vector3i new_pos = real_to_map(p1);

      if (old_pos == new_pos)
      {
        continue;
      }

      // check if out of bounds of the local map, if so:
      if (!local_map_ptr_->in_bounds(real_to_map(p1)))
      {
        finished_counter++;
        lines_finished[i] = RayStatus::FINISHED;

        continue;
      }

      // check if old tsdf is positive and (0 <= value < 600) and new tsdf is (< 0)
      auto old_tsdf = local_map_ptr_->value(old_pos);
      auto new_tsdf = local_map_ptr_->value(new_pos);

      auto old_val = old_tsdf.value();
      auto new_val = new_tsdf.value();

      // zero crossing detected
      if (old_val < 600 && old_val >= 0 && new_val < 0)
      {
        // calculate surface point
        Vector3i old_mm = old_pos * MAP_RESOLUTION;
        Vector3i new_mm = new_pos * MAP_RESOLUTION;

        Vector3i diff = (new_pos - old_pos).cwiseAbs();
        old_mm += diff * old_val;
        new_mm += diff * new_val;

        Vector3i avg_vec_mm = floor_divide((new_mm + old_mm), 2);

        // project point onto ray (by intersecting the ray with the plane cutting through the cell)

        // the normal is already the diff vector

        Vector3f intersection_point;

        auto is_ok = linePlaneIntersection(intersection_point, p1 - current_pose->pos, current_pose->pos, diff.cast<float>().normalized(), avg_vec_mm.cast<float>() / 1000.0f);

        // intersection point coordinates are currently in the coordinate system spanned by (0,0,0)
        // transform into the current pose coordinate system:
        auto trans_mat = pose->getTransformationMatrix();
        auto rot_part = trans_mat.block<3, 3>(0, 0);
        auto transl_part = trans_mat.block<3, 1>(0, 3);

        // rotate and translate
        intersection_point = rot_part * intersection_point + transl_part;

        if (!is_ok)
        {
          // std::cout << "No intersection detected" << std::endl;
        }
        else
        {
          PointType point;
          point.x = intersection_point.x();
          point.y = intersection_point.y();
          point.z = intersection_point.z();

          pointcloud->push_back(point);
          // surface_points.push_back(intersection_point);
        }
      }
    }
  }

  // return surface_points;
  return pointcloud;
}

bool RayTracer::linePlaneIntersection(Vector3f &intersection, Vector3f ray_vector,
                                      Vector3f ray_origin, Vector3f plane_normal, Vector3f plane_coord)
{
  // get d value
  float d = plane_normal.dot(plane_coord);

  // check if the plane is parallel to the the ray, if so return false
  if (plane_normal.dot(ray_vector) == 0)
  {
    return false;
  }

  // Compute the X value for the directed line ray intersecting the plane
  float t = (d - plane_normal.dot(ray_origin)) / plane_normal.dot(ray_vector);

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
  ray_marker_list.scale.x = params.ray_tracer.ray_size;
  ray_marker_list.scale.y = params.ray_tracer.ray_size;
  ray_marker_list.scale.z = params.ray_tracer.ray_size;
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
  int num_steps = std::floor(distance / params.ray_tracer.step_size);

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

    float factor = params.ray_tracer.step_size / length; // vector enlargement

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
