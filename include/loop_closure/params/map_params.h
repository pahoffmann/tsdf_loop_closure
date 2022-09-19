#pragma once
#include <iomanip>
#include <sstream>
#include <string>
#include <boost/filesystem.hpp>
#include <Eigen/Dense>
#include <ros/node_handle.h>
#include <loop_closure/util/point.h>

namespace fs = boost::filesystem;

inline bool replace(std::string& str, const std::string& from, const std::string& to)
{
  size_t start_pos = str.find(from);
  if(start_pos == std::string::npos)
    return false;
  str.replace(start_pos, from.length(), to);
  return true;
}

inline std::string max_distance_identifier(float d)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << d;
  std::string max_distance = ss.str();
  replace(max_distance, ".", "dot");
  return max_distance;
}

struct MapParams
{
  MapParams() = default;
  MapParams(const ros::NodeHandle& nh)
  {
    load(nh);
  }

  void load(const ros::NodeHandle& nh)
  {
    size.setOnes();

    nh.param<int>("map/resolution", resolution, MAP_RESOLUTION);
    nh.param<std::string>("map/comment", comment, "");
    nh.param<float>("map/max_distance", max_distance, 0.6);
    nh.param<int>("map/max_weight", max_weight, 10.f);
    float size1d;
    nh.param<float>("map/size/x", size1d, 20);
    size.x() = size1d;
    nh.param<float>("map/size/y", size1d, 20);
    size.y() = size1d;
    nh.param<float>("map/size/z", size1d, 5);
    size.z() = size1d;
    nh.param<int>("map/initial_weight", initial_weight, 0);
    nh.param<bool>("map/refinement", refinement, true);

    std::string temp;
    nh.param<std::string>("map/filename", temp, "");
    if (temp.empty())
    {
      nh.param<std::string>("map/dir", temp, "/home/julian/dev");
      dir = fs::path(temp);
      if (!fs::exists(dir))
      {
        throw std::runtime_error("Save directory doesn't exist");
      }
      create_identifier();
      filename = dir / (id + ".h5");
    }
    else
    {
      filename = fs::path(temp);
      filename.replace_extension("h5");
    }

    scale_tau(max_distance, tau);
    scale_max_weight(max_weight);
    scale_map_size(size, resolution);
  }

  static void scale_tau(const float max_distance, int& tau)
  {
    tau = static_cast<int>(max_distance * 1000.f);
  }

  static void scale_max_weight(int& max_weight)
  {
    max_weight *= WEIGHT_RESOLUTION;
  }

  static void scale_map_size(Eigen::Vector3i& size, int resolution)
  {
    size *= 1000; // scale to int
    size /= resolution; // scale to map
  }

  void create_identifier()
  {
    std::string max_distance_str = max_distance_identifier(max_distance);

    id = std::string("warpsense-int") \
      + std::string(comment.empty() ? "_" : "_" + std::string(comment) + "_") \
      + "res-" + std::to_string(resolution) + "_"
         + "max_d-" + max_distance_str + "_" \
      + "max_w-" + std::to_string(max_weight) + "_" \
      + "map-" + std::to_string(size.x()) + "x" + std::to_string(size.y()) + "x" + std::to_string(size.z());
  }

  std::string identifier() const
  {
    return id;
  }


  ~MapParams() = default;

  fs::path dir;
  fs::path filename;
  std::string comment;
  float max_distance;
  int resolution;
  int tau;
  int initial_weight;
  int max_weight;
  Eigen::Vector3i size;
  bool refinement;
  std::string id;
};
