#pragma once
#include "map_params.h"
#include "ray_tracer_params.h"

/**
 * @file dataset_params.h
 * @author patrick hoffmann
 * @date 10/17/21
 **/

struct DatasetParams
{
  DatasetParams() = default;

  explicit DatasetParams(const ros::NodeHandle &nh)
  {
    load(nh);
  }

  void load(const ros::NodeHandle &nh)
  {
    nh.param<std::string>("data_set/h5_file_name", data_set.h5_file_name, "");
    nh.param<std::string>("data_set/pcd_cloud_location", data_set.pcd_cloud_location, "");
    nh.param<std::string>("data_set/hannover1_location", data_set.hannover1_location, "");
  }

  struct Dataset
  {
    std::string h5_file_name;
    std::string pcd_cloud_location;

    std::string hannover1_location;
  } data_set;
};
