#pragma once
#include "map_params.h"
#include "ray_tracer_params.h"

/**
 * @file params.h
 * @author julian gaal
 * @author patrick hoffmann
 * @date 10/17/21
 **/

struct LoopClosureParams
{
  LoopClosureParams() = default;

  explicit LoopClosureParams(const ros::NodeHandle &nh)
  {
    load(nh);
  }

  void load(const ros::NodeHandle &nh)
  {
    // load map params
    map.load(nh);
    ray_tracer.load(nh);

    // load lc related params

    // max euclidean distance between poses to be viable for a lc
    nh.param<float>("map/max_dist_lc", loop_closure.max_dist_lc, 1.0f);
    // minimum distance traveled in m (sum of euclidean distances when follwing the pose grap)
    nh.param<float>("map/min_traveled_lc", loop_closure.min_traveled_lc, 10.0f);
    // when performing icp between lc poses, this is the max number of iterations
    nh.param<int>("map/max_icp_iterations", loop_closure.max_icp_iterations, 100);
    // method on how to generate the path: 0 = from global map, 1 = path exploration
    nh.param<int>("map/path_method", loop_closure.path_method, 0);
    // json filename
    nh.param<std::string>("map/json_dirname", loop_closure.json_dirname, "/home/patrick/maps/generated/json");
  }

  struct LoopClosure
  {
    int max_icp_iterations;
    float max_dist_lc;
    float min_traveled_lc;
    int path_method;
    std::string json_dirname;
  } loop_closure;

  MapParams map;
  RayTracerParams ray_tracer;
};
