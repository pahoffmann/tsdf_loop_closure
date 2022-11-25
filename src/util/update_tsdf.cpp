#include <loop_closure/util/update_tsdf.h>
#include <set>
#include <unordered_map>
#include <omp.h>

void update_tsdf(const std::vector<Eigen::Vector3i> &scan_points, const Eigen::Vector3i &scanner_pos, const Eigen::Vector3i &up,
                 LocalMap &buffer, int tau, int max_weight, int map_resolution, int pose_index)
{
  float angle = 45.f / 128.f; // TODO: Scanner FoV as Parameter
  int dz_per_distance = std::tan(angle / 180 * M_PI) / 2.0 * MATRIX_RESOLUTION;

  int weight_epsilon = tau / 10;

  int thread_count = omp_get_max_threads();

  std::vector<std::unordered_map<Eigen::Vector3i, TSDFEntry>> values(thread_count);

  auto pos = scanner_pos * map_resolution;

  int distance_zero_cnt = 0;
  int interpolation_zero_cnt = 0;
#pragma omp parallel num_threads(thread_count)
  {
    int current_thread = omp_get_thread_num();
    auto &local_values = values[current_thread];

#pragma omp for schedule(static)
    for (const auto &point : scan_points)
    {
      Eigen::Vector3i direction_vector = point - pos;
      long distance = direction_vector.norm();
      if (distance == 0)
      {
        distance_zero_cnt++;
        continue;
      }

      auto normed_direction_vector = (direction_vector.cast<long>() * MATRIX_RESOLUTION) / distance;
      auto interpolation_vector = (normed_direction_vector.cross(normed_direction_vector.cross(up.cast<long>()) / MATRIX_RESOLUTION));
      auto interpolation_norm = interpolation_vector.norm();
      if (interpolation_norm == 0)
      {
        interpolation_zero_cnt++;
        continue;
      }
      interpolation_vector = (interpolation_vector * MATRIX_RESOLUTION) / interpolation_norm;

      Eigen::Vector3i prev;

      for (int len = 1; len <= distance + tau; len += map_resolution / 2)
      {
        Eigen::Vector3i proj = pos + direction_vector * len / distance;
        Eigen::Vector3i index = proj / map_resolution;
        if (index.x() == prev.x() && index.y() == prev.y())
        {
          continue;
        }

        prev = index;

        if (!buffer.in_bounds(index.x(), index.y(), index.z()))
        {
          continue;
        }

        // use the distance to the center of the cell, since 'proj' can be anywhere in the cell
        Eigen::Vector3i target_center = index * map_resolution + Eigen::Vector3i::Constant(map_resolution / 2);
        long value_tmp = (point - target_center).norm();
        value_tmp = std::min(value_tmp, (long)tau);
        int value = static_cast<int>(value_tmp);

        if (len > distance)
        {
          value = -value;
        }

        // Calculate the corresponding weight for every TSDF value
        int weight = WEIGHT_RESOLUTION;
        if (value < -weight_epsilon)
        {
          weight = WEIGHT_RESOLUTION * (tau + value) / (tau - weight_epsilon);
        }
        
        if (weight == 0)
        {
          continue;
        }

        auto object = TSDFEntry(value, weight);
        int delta_z = dz_per_distance * len / MATRIX_RESOLUTION;
        auto iter_steps = (delta_z * 2) / map_resolution + 1;
        auto mid = delta_z / map_resolution;
        auto lowest = (proj - ((delta_z * interpolation_vector) / MATRIX_RESOLUTION).cast<int>());
        auto mid_index = index;

        for (auto step = 0; step < iter_steps; ++step)
        {
          index = (lowest + ((step * map_resolution * interpolation_vector) / MATRIX_RESOLUTION).cast<int>()) / map_resolution;

          if (!buffer.in_bounds(index.x(), index.y(), index.z()))
          {
            continue;
          }

          auto tmp = object;

          // if (mid_index != index)
          if (step != mid)
          {
            tmp.weight(tmp.weight() * -1);
          }

          auto existing = local_values.try_emplace(index, tmp);
          if (!existing.second && (abs(value) < abs(existing.first->second.value()) || existing.first->second.weight() < 0))
          {
            existing.first->second = tmp;
          }
        }
      }
    }

    // wait for all threads to fill their local_values
#pragma omp barrier
    for (auto &map_entry : local_values)
    {

      bool skip = false;
      for (int i = 0; i < thread_count; i++)
      {
        if (i == current_thread)
        {
          continue;
        }

        auto iter = values[i].find(map_entry.first);
        if (iter != values[i].end() && fabsf(iter->second.value()) < fabsf(map_entry.second.value()))
        {
          skip = true;
          break;
        }
      }
      if (skip)
      {
        continue;
      }

      auto &index = map_entry.first;
      auto value = map_entry.second.value();
      auto weight = map_entry.second.weight();

      auto &entry = buffer.value(index.x(), index.y(), index.z());

      // if both cell already exited, and new value calculated for it -> average da ting
      if (weight > 0 && entry.weight() > 0)
      {
        entry.value((entry.value() * entry.weight() + value * weight) / (entry.weight() + weight));
        entry.weight(std::min(max_weight, entry.weight() +
                                              weight)); // This variable (max_weight) ensures that later changes can still have an influence to the map
        entry.intersect(static_cast<TSDFEntryHW::IntersectionType>(TSDFEntry::IntersectStatus::NO_INT));
        entry.pose_index(static_cast<TSDFEntryHW::IndexType>(pose_index));
      }
      // if this is the first time writing to cell, overwrite with newest values
      else if (weight != 0 && entry.weight() <= 0)
      {
        entry.value(value);
        entry.weight(weight);
        entry.intersect(static_cast<TSDFEntryHW::IntersectionType>(TSDFEntry::IntersectStatus::NO_INT));
        entry.pose_index(static_cast<TSDFEntryHW::IndexType>(pose_index));
      }
    }
  }
}

void reverse_update_tsdf(const std::vector<Eigen::Vector3i> &scan_points, const Eigen::Vector3i &scanner_pos, const Eigen::Vector3i &up,
                         LocalMap &buffer, int tau, int max_weight, int map_resolution, int pose_index)
{
  float angle = 45.f / 128.f; // TODO: Scanner FoV as Parameter
  int dz_per_distance = std::tan(angle / 180 * M_PI) / 2.0 * MATRIX_RESOLUTION;

  int weight_epsilon = tau / 10;

  int thread_count = omp_get_max_threads();

  std::vector<std::unordered_map<Eigen::Vector3i, TSDFEntry>> values(thread_count);

  auto pos = scanner_pos * map_resolution;

  int distance_zero_cnt = 0;
  int interpolation_zero_cnt = 0;
#pragma omp parallel num_threads(thread_count)
  {
    int current_thread = omp_get_thread_num();
    auto &local_values = values[current_thread];

#pragma omp for schedule(static)
    for (const auto &point : scan_points)
    {
      Eigen::Vector3i direction_vector = point - pos;
      int distance = direction_vector.norm();
      if (distance == 0)
      {
        distance_zero_cnt++;
        continue;
      }

      auto normed_direction_vector = (direction_vector.cast<long>() * MATRIX_RESOLUTION) / distance;
      auto interpolation_vector = (normed_direction_vector.cross(normed_direction_vector.cross(up.cast<long>()) / MATRIX_RESOLUTION));
      auto interpolation_norm = interpolation_vector.norm();
      if (interpolation_norm == 0)
      {
        interpolation_zero_cnt++;
        continue;
      }
      interpolation_vector = (interpolation_vector * MATRIX_RESOLUTION) / interpolation_norm;

      Eigen::Vector3i prev;

      for (int len = 1; len <= distance + tau; len += map_resolution / 2)
      {
        Eigen::Vector3i proj = pos + direction_vector * len / distance;
        Eigen::Vector3i index = proj / map_resolution;
        if (index.x() == prev.x() && index.y() == prev.y())
        {
          continue;
        }
        prev = index;
        if (!buffer.in_bounds(index.x(), index.y(), index.z()))
        {
          continue;
        }

        auto object = TSDFEntry(tau, 0);
        int delta_z = dz_per_distance * len / MATRIX_RESOLUTION;
        auto iter_steps = (delta_z * 2) / map_resolution + 1;
        auto mid = delta_z / map_resolution;
        auto lowest = (proj - ((delta_z * interpolation_vector) / MATRIX_RESOLUTION).cast<int>());
        auto mid_index = index;

        for (auto step = 0; step < iter_steps; ++step)
        {
          index = (lowest + ((step * map_resolution * interpolation_vector) / MATRIX_RESOLUTION).cast<int>()) / map_resolution;

          if (!buffer.in_bounds(index.x(), index.y(), index.z()))
          {
            continue;
          }

          auto tmp = object;
          
          auto existing = local_values.try_emplace(index, tmp);
          if (!existing.second)
          {
            existing.first->second = tmp;
          }
        }
      }
    }

    // wait for all threads to fill their local_values
#pragma omp barrier
    for (auto &map_entry : local_values)
    {
      // bool skip = false;
      // for (int i = 0; i < thread_count; i++)
      // {
      //   if (i == current_thread)
      //   {
      //     continue;
      //   }

      //   auto iter = values[i].find(map_entry.first);
      //   if (iter != values[i].end() && fabsf(iter->second.value()) < fabsf(map_entry.second.value()))
      //   {
      //     skip = true;
      //     break;
      //   }
      // }
      // if (skip)
      // {
      //   continue;
      // }

      auto &index = map_entry.first;

      auto &entry = buffer.value(index.x(), index.y(), index.z());

      // in the reverse update, just reset the found cell, whatever it may be
      if (entry.pose_index() == static_cast<TSDFEntryHW::IndexType>(pose_index))
      {
        entry.value(tau);
        entry.weight(0);
        entry.intersect(static_cast<TSDFEntryHW::IntersectionType>(TSDFEntry::IntersectStatus::NO_INT));
        entry.pose_index(-1);
      }
    }
  }
}
