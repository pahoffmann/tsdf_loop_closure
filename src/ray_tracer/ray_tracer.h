#pragma once

#include "../util/point.h"
#include "../util/colors.h"
#include "../map/local_map.h"
//#include "tracer.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <loop_closure/LoopClosureConfig.h>
#include <visualization_msgs/Marker.h>

/**
 * @brief Class used to emulate a laserscan, by tracing artifical rays in space based on a 6D pose.
 *        Is able to check intersections with a localmap and generate markers for the Robot Operating System (ROS) for display purposes.
 */
class RayTracer {
private:

    // configuration for the ray tracer
    loop_closure::LoopClosureConfig* lc_config; 
    
    // current pose used for tracing
    Pose* current_pose;

    // pointer to the current local map
    std::shared_ptr<LocalMap> local_map_ptr_;

    // used to store the rays, which have been finished, will be cleared after each tracing
    std::vector<bool> lines_finished;

    // counter, which tracks, how many lines have been finished, works hand in hand with above array.
    int finished_counter = 0;

    // the actual rays, every ray starts at the current position of the tracer
    std::vector<Eigen::Vector3f> rays;

    /**
     * @brief initializes the rays for the current position
     * 
     */
    void initRays();

    /**
     * @brief updates the rays, until every single one is finished
     * 
     */
    void updateRays();

    /**
     * @brief does some cleanup work in between runs.
     * 
     */
    void cleanup();
public:

    RayTracer();
    /**
     * @brief Construct a new Raytracer object, inserts the respective config, including a shared pointer to the current local map.
     * 
     * @param new_config 
     * @param local_map_in 
     */
    RayTracer(loop_closure::LoopClosureConfig* new_config, std::shared_ptr<LocalMap> local_map_in, Pose* start_pose);

    /**
     * @brief Starts the tracing process
     * 
     */
    void start();

    /**
     * @brief Get the ros marker for the current ray trace
     * 
     * @return visualization_msgs::Marker 
     */
    visualization_msgs::Marker get_ros_marker();

    /**
     * @brief updates the local map pointer, due to a dynamic reconfigure
     * 
     * @param local_map_in 
     */
    inline void update_map_pointer(std::shared_ptr<LocalMap> local_map_in) {
        local_map_ptr_ = local_map_in;
    };
};