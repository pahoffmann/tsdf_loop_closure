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
    nh.param<float>("loop_closure/max_dist_lc", loop_closure.max_dist_lc, 1.0f);
    // minimum distance traveled in m (sum of euclidean distances when follwing the pose grap)
    nh.param<float>("loop_closure/min_traveled_lc", loop_closure.min_traveled_lc, 10.0f);
    // defines the minimum distance between two loop closures
    nh.param<float>("loop_closure/dist_between_lcs", loop_closure.dist_between_lcs, 10.0f);
      // defines the minimum number of lc's per pose
    nh.param<int>("loop_closure/max_closures_per_pose", loop_closure.max_closures_per_pose, 2);
    // when performing icp between lc poses, this is the max number of iterations
    nh.param<int>("loop_closure/max_icp_iterations", loop_closure.max_icp_iterations, 100);
    // method on how to generate the path: 0 = from global map, 1 = path exploration
    nh.param<int>("loop_closure/path_method", loop_closure.path_method, 0);
    // max icp fitness score for preregistration
    nh.param<float>("loop_closure/max_prereg_icp_fitness", loop_closure.max_prereg_icp_fitness, 0.15f);
    // max icp fitness score for loop closure
    nh.param<float>("loop_closure/max_lc_icp_fitness", loop_closure.max_lc_icp_fitness, 0.4f);

    // indicates whether a preregistration is done or not
    nh.param<bool>("loop_closure/do_preregistration", loop_closure.do_preregistration, true);

    // json filename
    nh.param<std::string>("loop_closure/json_dirname", loop_closure.json_dirname, "/home/patrick/maps/generated/json");

    // NOISES (actual, not the variance)
    // prior
    nh.param<float>("loop_closure/prior_translation_noise_x", loop_closure.prior_translation_noise_x, 0.4f);
    nh.param<float>("loop_closure/prior_translation_noise_y", loop_closure.prior_translation_noise_y, 0.4f);
    nh.param<float>("loop_closure/prior_translation_noise_z", loop_closure.prior_translation_noise_z, 0.4f);

    // rotation (in radiants)
    nh.param<float>("loop_closure/prior_rotation_noise_x", loop_closure.prior_rotation_noise_x, 0.1f);
    nh.param<float>("loop_closure/prior_rotation_noise_y", loop_closure.prior_rotation_noise_y, 0.1f);
    nh.param<float>("loop_closure/prior_rotation_noise_z", loop_closure.prior_rotation_noise_z, M_PI);

    // between
    nh.param<float>("loop_closure/between_translation_noise_x", loop_closure.between_translation_noise_x, 0.1f);
    nh.param<float>("loop_closure/between_translation_noise_y", loop_closure.between_translation_noise_y, 0.1f);
    nh.param<float>("loop_closure/between_translation_noise_z", loop_closure.between_translation_noise_z, 0.1f);

    // rotation (in radiants)
    nh.param<float>("loop_closure/between_rotation_noise_x", loop_closure.between_rotation_noise_x, 0.01f);
    nh.param<float>("loop_closure/between_rotation_noise_y", loop_closure.between_rotation_noise_y, 0.01f);
    nh.param<float>("loop_closure/between_rotation_noise_z", loop_closure.between_rotation_noise_z, 0.01f);
  }

  struct LoopClosure
  {
    int max_icp_iterations;
    float max_dist_lc;
    float min_traveled_lc;
    float dist_between_lcs;
    int max_closures_per_pose;

    int path_method;
    std::string json_dirname;

    float max_prereg_icp_fitness;
    float max_lc_icp_fitness;
    bool do_preregistration;

    // gtsam noise params
    float prior_rotation_noise_x;
    float prior_rotation_noise_y;
    float prior_rotation_noise_z;
    float prior_translation_noise_x;
    float prior_translation_noise_y;
    float prior_translation_noise_z;

    float between_rotation_noise_x;
    float between_rotation_noise_y;
    float between_rotation_noise_z;
    float between_translation_noise_x;
    float between_translation_noise_y;
    float between_translation_noise_z;
  } loop_closure;

  MapParams map;
  RayTracerParams ray_tracer;
};
