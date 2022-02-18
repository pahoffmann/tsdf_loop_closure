#pragma once

#include "../util/point.h"
#include "../util/colors.h"
#include "../map/local_map.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <loop_closure/RayTracerConfig.h>

/**
 * @brief Class used to emulate a laserscan, by tracing artifical rays in space based on a 6D pose.
 *        Is able to check intersections with a localmap and generate markers for the Robot Operating System (ROS) for display purposes.
 */
class RayTracer {
private:

    // configuration for the ray tracer
    loop_closure::RayTracerConfig* rt_config; 
    
    // current pose used for tracing
    Pose* current_pose;

    // current tsdf marker
    visualization_msgs::Marker marker;

    // pointer to the current local map
    std::shared_ptr<LocalMap> local_map_ptr_;

    // used to store the rays, which have been finished, will be cleared after each tracing
    std::vector<bool> lines_finished;

    // the actual rays
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> rays;

    void initRays();
    void updateRays();
public:

    RayTracer();
    /**
     * @brief Construct a new Raytracer object, inserts the respective config, including a shared pointer to the current local map.
     * 
     * @param new_config 
     * @param local_map_in 
     */
    RayTracer(loop_closure::RayTracerConfig* new_config, std::shared_ptr<LocalMap> local_map_in, Pose* start_pose);

    /**
     * @brief Get the ros marker object, might be better set up inside the localmap code.
     * 
     * @return visualization_msgs::Marker* 
     */
    visualization_msgs::Marker* get_ros_marker();

    /**
     * @brief Starts the tracing process
     * 
     */
    void start();
};