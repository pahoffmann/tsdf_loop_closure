#pragma once
#include <iomanip>
#include <sstream>
#include <string>
#include <boost/filesystem.hpp>
#include <Eigen/Dense>
#include <ros/node_handle.h>
#include <loop_closure/util/point.h>

namespace fs = boost::filesystem;

struct RayTracerParams
{
    RayTracerParams() = default;
    RayTracerParams(const ros::NodeHandle &nh)
    {
        load(nh);
    }

    void load(const ros::NodeHandle &nh)
    {
        nh.param<int>("ray_tracer/hor_res", hor_res, 1024);
        nh.param<int>("ray_tracer/vert_res", vert_res, 128);
        nh.param<int>("ray_tracer/opening_degree", opening_degree, 45);
        nh.param<float>("ray_tracer/step_size", step_size, 0.032);
        nh.param<float>("ray_tracer/ray_size", ray_size, 0.01);

        nh.param<int>("ray_tracer/hor_res_icp", hor_res_icp, 128);
        nh.param<int>("ray_tracer/vert_res_icp", vert_res_icp, 64);
        nh.param<int>("ray_tracer/opening_degree_icp", opening_degree_icp, 45);
    }

    ~RayTracerParams() = default;

    // basic raytracer stuff
    int hor_res;
    int vert_res;
    int opening_degree;
    float step_size;
    float ray_size;

    // icp related stuff
    int hor_res_icp;
    int vert_res_icp;
    int opening_degree_icp;
};
