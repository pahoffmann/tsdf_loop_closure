#pragma once
#include "map_params.h"

/**
  * @file params.h
  * @author julian gaal
  * @author patrick hoffmann
  * @date 10/17/21
 **/


struct LoopClosureParams
{
    LoopClosureParams() = default;

    explicit LoopClosureParams(const ros::NodeHandle& nh)
    {
      load(nh);
    }

    void load(const ros::NodeHandle& nh)
    {
    
      map.load(nh);
    }

    struct LoopClosure
    {
      int max_icp_iterations;
    } loop_closure;

    MapParams map;
};


