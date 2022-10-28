#pragma once

/**
 * @file ray_tracer.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <loop_closure/params/loop_closure_params.h>

#include <loop_closure/util/point.h>
#include <loop_closure/util/colors.h>
#include <loop_closure/map/local_map.h>
#include <loop_closure/data_association/association.h>
#include <loop_closure/util/eigen_vs_ros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <omp.h>

#include <pcl/point_cloud.h>


/**
 * @brief Class used to emulate a laserscan, by tracing artifical rays in space based on a 6D pose.
 *        Is able to check intersections with a localmap and generate markers for the Robot Operating System (ROS) for display purposes.
 */
class RayTracer {
private:

    enum RayStatus {
        INIT,
        HIT,
        ZERO_CROSSED,
        FINISHED
    };

    // general parametrization
    LoopClosureParams params;
    
    // current pose used for tracing
    Pose* current_pose;

    // pointer to the current local map
    std::shared_ptr<LocalMap> local_map_ptr_;

    //pointer to global map
    std::shared_ptr<GlobalMap> global_map_ptr_;

    // used to store the rays, which have been finished, will be cleared after each tracing
    // a ray is finished when
    // 1. it crossed the boundaries of the local map
    // 2. it encountered a.) a sign change and b.)  if it has crossed tsdf which actually has a meaning, went through a (value change)
    //    -> this means, that the ray actually hit a 'wall', or to be exact, the tsdf, which describes a surface implicitly
    // each of these states is covered in the 'RayStatus' enum, which will be used.
    std::vector<RayStatus> lines_finished;

    // counter, which tracks, how many lines have been finished, works hand in hand with above array.
    // [DEBUG]
    int finished_counter = 0;
    int num_good = 0;
    int num_not_good = 0;

    float side_length_x;
    float side_length_y;
    float side_length_z;

    // the actual rays, every ray starts at the current position of the tracer
    std::vector<Eigen::Vector3f> rays;

    // a 3D array representing colors from individual scans, currently unused
    std::vector<std::vector<std::vector<std_msgs::ColorRGBA>>> colors;

    std::vector<std::vector<Vector3i>> current_ray_associations;

    // the current data association we are working with
    Association *cur_association;

    // each pair contains the start and end vertex of bresenham
    std::vector<Vector3i> bresenham_cells;

    /**
     * @brief initializes the rays for the current position
     * 
     * @param use_icp_params this flag controls, which parameters are used to init the rays (normal or the ones for icp)
     */
    void initRays(bool use_icp_params = false);

    /**
     * @brief updates the rays, until every single one is finished
     * 
     */
    void updateRays(int mode = 0);

    /**
     * @brief new method to update rays. new logic for checking, if a ray is done, new logic for adding associations
     * 
     * @param mode 
     */
    void updateRaysNew(int mode = 0);

    /**
     * @brief do whatever necessary to initialize the 3D Bresenham structure to ensure a higher hit percentage.
     * 
     */
    void init3DBresenham();

    /**
     * @brief uses the data initialized by above function and performs a bresenham approximation being considerate of the TSDF data structure
     * 
     */
    void perform3DBresenham();

    /**
     * @brief start 3D bresenham
     * 
     */
    void start3DBresenham();

    /**
     * @brief calculates the intersection between a ray and and a plane, returns true if successful, false if the plane is parallel to the ray
     * 
     * calc the intersection between the surfaces and the ray
     * math behind here:
     * All points X of a plane follow the equation
     * Dot(N, X) = d
     * where d can be calculated by putting an arbitrary point X into the equation
     * wheres the ray is described by
     * s = p + t*D
     * with s being all the points the ray might generate by the location vec p and direction vec D
     * this gives us:
     * t = (d - Dot(N, p)) / Dot(N, D)
     * by inserting the resulting t in the line equation, we get the intersection.
     * see: https://stackoverflow.com/questions/7168484/3d-line-segment-and-plane-intersection
     * 
     * @param intersection 
     * @param ray_vector 
     * @param ray_origin 
     * @param plane_normal 
     * @param plane_coord 
     * @return true 
     * @return false 
     */
    bool linePlaneIntersection(Vector3f& intersection, Vector3f ray_vector, Vector3f ray_origin, Vector3f plane_normal, Vector3f plane_coord);

    /**
     * @brief does some cleanup work in between runs.
     * 
     */
    void cleanup();
public:

    /**
     * @brief Delete default constructor
     * 
     */
    RayTracer() = delete;

    /*
     * @brief Construct a new Raytracer object, inserts the respective config, including a shared pointer to the current local map.
     * 
     * @param new_config 
     * @param local_map_in 
     */
    RayTracer(LoopClosureParams &lc_params, std::shared_ptr<LocalMap> local_map_in, std::shared_ptr<GlobalMap> global_map_in);

    /**
     * @brief function used to update the association the ray tracer is working with.
     * 
     * @param association 
     */
    inline void update_association(Association *association)
    {
        cur_association = association;
    }

    /**
     * @brief Starts the tracing process
     * 
     * @param mode 0: normal, 1: no association data being written, just plain scan while keeping track of hit vs non hit
     * 
     * @return float stating num_ok / num_error  value
     */
    float start(int mode = 0);

    /**
     * @brief starts the bresenham tracing
     * 
     */
    void start_bresenham();
    
    /**
     * @brief will remove all associated cells around the passed pose from the localmap
     * 
     * @param pose 
     */
    void local_removal(Pose *pose, int pose_index);

    /**
     * @brief Get the ros marker for the current ray trace
     * 
     * @return visualization_msgs::Marker 
     */
    visualization_msgs::Marker get_ros_marker();

    /**
     * @brief Get the bresenham intersections as a ros marker
     * 
     * @return visualization_msgs::Marker 
     */
    visualization_msgs::Marker get_bresenham_intersection_marker();

    /**
     * @brief updates the local map pointer, due to a dynamic reconfigure
     * 
     * @param local_map_in 
     */
    inline void update_map_pointer(std::shared_ptr<LocalMap> local_map_in) {
        local_map_ptr_ = local_map_in;
    };

    /**
     * @brief Updates the pose used for tracing, performs a shift in the local map
     *        so that the new pose is the center of the local map
     * 
     * @param new_pose 
     */
    void update_pose(Pose *new_pose);

    /**
     * @brief checks, wether pose a is visble from Pose b and vice versa, this means, there cannot be a sign change in tsdf, when casting a ray between a and b
     * 
     * @param a 
     * @param b 
     */
    bool is_visible(Pose &a, Pose &b) ;

    /**
     * @brief approximates a pointcloud for a given pose
     * 
     * @return std::vector<Eigen::Vector3f> 
     */
    pcl::PointCloud<PointType>::Ptr approximate_pointcloud(Pose *pose);
};