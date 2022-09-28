/**
 * @file path_listener.cpp
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */

// ros related includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// c++ standard related includes
#include <sstream>
#include <iostream>

// external bibs
#include <highfive/H5File.hpp>

// own includes
#include <loop_closure/params/loop_closure_params.h>
#include <loop_closure/map/global_map.h>
#include <loop_closure/map/local_map.h>
#include <loop_closure/ray_tracer/ray_tracer.h>
#include <loop_closure/path/path.h>
#include <loop_closure/visualization/ros_viewhelper.h>
#include <loop_closure/util/point.h>
#include <loop_closure/util/eigen_vs_ros.h>
#include <loop_closure/util/update_tsdf.h>
#include <loop_closure/gtsam/gtsam_wrapper.h>

// transform between ros and eigen
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// pcl includes
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace message_filters;

// parametrization
LoopClosureParams params;

// own stuff
RayTracer *tracer;
std::shared_ptr<LocalMap> local_map_ptr;
std::shared_ptr<GlobalMap> global_map_ptr;
std::unique_ptr<GTSAMWrapper> gtsam_wrapper_ptr;
Path *path;
Path *optimized_path;
int side_length_xy;
int side_length_z;

// vector to store found lc index pairs
std::vector<std::pair<int, int>> lc_index_pairs;

// save a vector of pointclouds (which should be preprocessed)
std::vector<pcl::PointCloud<PointType>::Ptr> dataset_clouds;

// loop closure information ((cur) pose of the last lc before transformation and the transformation difference of the last pose due to the lc)
Pose last_loop_old_pose;
Matrix4f last_loop_transform_diff;

// ros

// subscribe to lsma6d cloud and poses
ros::Publisher path_pub;
ros::Publisher optimized_path_pub;
ros::Publisher loop_pub;
ros::Publisher tsdf_pub;

// publisher used to signal to it's subscribers, that the node is back in idle mode
ros::Publisher ready_flag_pub;

// publish approximated cloud for current pose of lc (cur_index > prev_index)
ros::Publisher approx_pcl_pub_cur;
// publish approximated cloud for previous pose of lc (cur_index > prev_index)
ros::Publisher approx_pcl_pub_prev;
//
ros::Publisher approx_pcl_pub_icp;

ros::Publisher input_cloud_pub;
ros::Publisher filtered_cloud_pub;

nav_msgs::Path ros_path;
sensor_msgs::PointCloud2 cur_pcl_msg;
sensor_msgs::PointCloud2 prev_pcl_msg;
sensor_msgs::PointCloud2 icp_pcl_msg;
sensor_msgs::PointCloud2 cur_pcl_filtered_msg;
sensor_msgs::PointCloud2 prev_pcl_filtered_msg;

/**
 * @brief method used to initiate all the objects associated with this test case
 *
 */
void init_obj()
{
    // create maps
    global_map_ptr.reset(new GlobalMap(params.map));
    local_map_ptr.reset(new LocalMap(params.map.size.x(), params.map.size.y(), params.map.size.y(), global_map_ptr));
    gtsam_wrapper_ptr.reset(new GTSAMWrapper(params));

    // init ray tracer
    tracer = new RayTracer(params, local_map_ptr, global_map_ptr);

    // init path
    path = new Path();
    path->attach_raytracer(tracer);

    ros_path.header.frame_id = "map";
    ros_path.header.stamp = ros::Time::now();

    // initialize these values so that transformations have no effect
    last_loop_old_pose.pos = Vector3f::Zero();
    last_loop_old_pose.quat = Eigen::Quaternionf::Identity();
    last_loop_transform_diff = Matrix4f::Identity();
}

void fill_optimized_path(gtsam::Values values)
{
    optimized_path = new Path();

    for (auto value : values)
    {
        auto pose = value.value.cast<gtsam::Pose3>();

        gtsam::Rot3 rotation = pose.rotation();
        Vector3f position;
        position << pose.x(), pose.y(), pose.z();
        Eigen::Matrix3f rotation_mat = rotation.matrix().cast<float>();

        Quaternionf quat(rotation_mat);
        Pose new_pose;
        new_pose.pos = position;
        new_pose.quat = quat;
        optimized_path->add_pose(new_pose);
    }
}

/**
 * @brief will check the currently found lc pair against the ones already found
 *
 * @param pair
 * @return true
 * @return false
 */
bool is_viable_lc(std::pair<int, int> candidate_pair)
{
    if (candidate_pair.first == -1 || candidate_pair.second == -1)
    {
        return false;
    }

    if (lc_index_pairs.size() == 0)
    {
        return true;
    }

    // the first index of the candidate pair needs to be greater than the already found pair
    // [TODO] eval this solution
    for (auto lc_pair : lc_index_pairs)
    {
        // if (candidate_pair.first < lc_pair.second || path->get_distance_between_path_poses(lc_pair.first, lc_pair.second) < params.loop_closure.min_traveled_lc)
        // {
        //     return false;
        // }

        // testing purpose, using different indices
        if (candidate_pair.second < lc_pair.second || path->get_distance_between_path_poses(lc_pair.second, candidate_pair.second) < params.loop_closure.min_traveled_lc)
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief handles incoming pointcloud2
 *
 * @param cloud_ptr
 */
void handle_slam6d_cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr, const geometry_msgs::PoseStampedConstPtr &pose_ptr)
{
    // std::cout << "Received Cloud timestamp: " << cloud_ptr->header.stamp << std::endl;
    // std::cout << "Received Pose timestamp: " << pose_ptr->header.stamp << std::endl;

    // message used to show, that the processing of the last message pair is done and the node awaits new data
    std_msgs::String ready_msg;
    ready_msg.data = std::string("ready");

#pragma region PREPROCESSING

    // filter input cloud:
    pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*cloud_ptr.get(), *input_cloud.get());
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(input_cloud);
    sor.setLeafSize(params.map.resolution / 1000.0f, params.map.resolution / 1000.0f, params.map.resolution / 1000.0f);
    sor.filter(*filtered_cloud.get());

    // save
    // dataset_clouds.push_back(filtered_cloud);
    dataset_clouds.push_back(input_cloud);

    // create Pose from ros pose
    Pose input_pose;
    Eigen::Vector3d tmp_point;
    Eigen::Quaterniond tmp_quat;
    tf::pointMsgToEigen(pose_ptr->pose.position, tmp_point);
    tf::quaternionMsgToEigen(pose_ptr->pose.orientation, tmp_quat);
    input_pose.quat = tmp_quat.cast<float>();
    input_pose.pos = tmp_point.cast<float>();

    // transform input pose according to the last loop closure
    auto input_pose_origin = last_loop_old_pose.getTransformationMatrix().inverse() * input_pose.getTransformationMatrix();
    auto input_pose_origin_transformed = last_loop_transform_diff * input_pose_origin;
    auto input_pose_transformed = last_loop_old_pose.getTransformationMatrix() * input_pose_origin_transformed;

    input_pose = Pose(input_pose_transformed);

    // add the new pose to the path
    path->add_pose(input_pose);

    Matrix4f pose = input_pose.getTransformationMatrix();

#pragma endregion

#pragma region TSDF_UPDATE
    std::vector<Eigen::Vector3i> points_original(filtered_cloud->size());

    // transform points to map coordinates
#pragma omp parallel for schedule(static) default(shared)
    for (int i = 0; i < filtered_cloud->size(); ++i)
    {
        const auto &cp = (*filtered_cloud)[i];
        points_original[i] = Eigen::Vector3i(cp.x * 1000.f, cp.y * 1000.f, cp.z * 1000.f);
    }

    // Shift
    Vector3i pos = real_to_map(pose.block<3, 1>(0, 3));
    local_map_ptr->shift(pos);

    Eigen::Matrix4i rot = Eigen::Matrix4i::Identity();
    rot.block<3, 3>(0, 0) = to_int_mat(pose).block<3, 3>(0, 0);
    Eigen::Vector3i up = transform_point(Eigen::Vector3i(0, 0, MATRIX_RESOLUTION), rot);

    // create TSDF Volume
    update_tsdf(points_original, pos, up, *local_map_ptr, params.map.tau, params.map.max_weight, params.map.resolution);

#pragma endregion

#pragma region GTSAM_EDGE_CONSTRAINTS
    // gtsam optimizations
    // add gtsam constraints
    if (path->get_length() == 1)
    {
        Matrix4f pose_mat = path->at(0)->getTransformationMatrix();
        gtsam_wrapper_ptr->add_prior_constraint(pose_mat);
    }
    else
    {
        int from_idx = path->get_length() - 2;
        int to_idx = path->get_length() - 1;
        auto transform_diff = getTransformationMatrixDiff(path->at(from_idx)->getTransformationMatrix(), path->at(to_idx)->getTransformationMatrix());
        gtsam_wrapper_ptr->add_in_between_contraint(transform_diff, from_idx, to_idx);
    }
#pragma endregion

    local_map_ptr->write_back();

#pragma region CLOUD_GM_VIS
    // auto gm_data = global_map_ptr->get_full_data();
    // auto marker = ROSViewhelper::marker_from_gm_read(gm_data);
    // auto marker = ROSViewhelper::initTSDFmarkerPose(local_map_ptr, new Pose(pose));
    // tsdf_pub.publish(marker);

    input_cloud_pub.publish(*cloud_ptr);
    sensor_msgs::PointCloud2 filtered_ros_cloud;
    pcl::toROSMsg(*filtered_cloud, filtered_ros_cloud);
    filtered_cloud_pub.publish(filtered_ros_cloud);

    path_pub.publish(ROSViewhelper::initPathMarker(path, Colors::ColorNames::lime));

#pragma endregion

#pragma region LOOP_IDENTIFICATION
    // check for loops
    auto lc_pair = path->find_loop_kd_min_dist_backwards(path->get_length() - 1, params.loop_closure.max_dist_lc, params.loop_closure.min_traveled_lc, false);

    // check the return of the loop detection method
    if (lc_pair.first != -1 && lc_pair.second != -1)
    {
        std::cout << "Found LC Candidate between pose " << lc_pair.first << " and " << lc_pair.second << std::endl;

        // check if the detected loop might be viable considering already found loops
        if (!is_viable_lc(lc_pair))
        {
            std::cout << "LC pair not viable!" << std::endl;
            ready_flag_pub.publish(ready_msg);

            return;
        }
        else
        {
            std::cout << "Possible LC found between " << lc_pair.first << " and " << lc_pair.second << std::endl;
        }
    }
    else
    {
        std::cout << "No loop found when inserting Pose" << path->get_length() << std::endl;
        ready_flag_pub.publish(ready_msg);

        return;
    }
#pragma endregion

#pragma region LOOP_EXAMINATION

    pcl::PointCloud<PointType>::Ptr icp_cloud;
    icp_cloud.reset(new pcl::PointCloud<PointType>());

    auto cur_cloud_transform = path->at(lc_pair.second)->getTransformationMatrix();
    auto prev_cloud_transform = path->at(lc_pair.first)->getTransformationMatrix();

    pcl::PointCloud<PointType>::Ptr source_cloud;
    pcl::PointCloud<PointType>::Ptr target_cloud;
    source_cloud.reset(new pcl::PointCloud<PointType>());
    target_cloud.reset(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*dataset_clouds[lc_pair.second], *source_cloud, cur_cloud_transform);
    pcl::transformPointCloud(*dataset_clouds[lc_pair.first], *target_cloud, prev_cloud_transform);

    bool converged = gtsam_wrapper_ptr->add_loop_closure_constraint(lc_pair, source_cloud, target_cloud,
                                                                    icp_cloud, cur_cloud_transform, prev_cloud_transform);

    // if the lc found did not converge, we skip everything else
    if (!converged)
    {
        ready_flag_pub.publish(ready_msg);

        return;
    }

#pragma endregion

    // THE FOLLOWING IS ONLY EXECUTED, WHEN A LOOP CLOSURE WAS IDENTIFIED AND SCAN MATCHING CONVERGED IN A NICE MANNER //

#pragma region LOOP_PROCESSING_AND_VISUALIZATION
    lc_index_pairs.push_back(lc_pair);
    auto lc_marker = ROSViewhelper::init_loop_detected_marker_multiple(path, lc_index_pairs);
    loop_pub.publish(lc_marker);

    gtsam::Values initial;

    // fill initial values with path positions
    for (int i = 0; i < path->get_length(); i++)
    {
        auto current_pose = path->at(i);
        gtsam::Rot3 rot3(path->at(i)->rotationMatrixFromQuaternion().cast<double>());
        gtsam::Point3 point3(current_pose->pos.x(), current_pose->pos.y(), current_pose->pos.z());

        initial.insert(i, gtsam::Pose3(rot3, point3));
    }

    auto new_values = gtsam_wrapper_ptr->perform_optimization(initial);

    fill_optimized_path(new_values);

    optimized_path_pub.publish(ROSViewhelper::initPathMarker(optimized_path, Colors::ColorNames::fuchsia));

    std::cout << "Points in msg: " << cur_pcl_msg.data.size() << std::endl;

    cur_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(source_cloud);
    prev_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(target_cloud);
    icp_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(icp_cloud);

    approx_pcl_pub_cur.publish(cur_pcl_msg);
    approx_pcl_pub_prev.publish(prev_pcl_msg);
    approx_pcl_pub_icp.publish(icp_pcl_msg);

    // update path and save transform and diffs
    last_loop_old_pose = *path->at(path->get_length() - 1);
    last_loop_transform_diff =
        getTransformationMatrixDiff(last_loop_old_pose.getTransformationMatrix(), optimized_path->at(optimized_path->get_length() - 1)->getTransformationMatrix());

    // copy values
    path = new Path(*optimized_path);

#pragma endregion

    // fix map

    // publish

    ready_flag_pub.publish(ready_msg);
}

/**
 * @brief Main method
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam6d_listener");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // specify ros loop rate
    ros::Rate loop_rate(1.0f);

    // load params from nodehandle
    params = LoopClosureParams(nh);
    init_obj();

    message_filters::Subscriber<sensor_msgs::PointCloud2> slam6d_cloud_sub(n, "/slam6d_cloud", 100);
    message_filters::Subscriber<geometry_msgs::PoseStamped> slam6d_pose_sub(n, "/slam6d_pose", 100);
    message_filters::Subscriber<std_msgs::String> slam6d_filename_sub(n, "/slam6d_filename", 100);

    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(40), slam6d_cloud_sub, slam6d_pose_sub);
    sync.registerCallback(boost::bind(&handle_slam6d_cloud_callback, _1, _2));

    optimized_path_pub = n.advertise<visualization_msgs::Marker>("/optimized_path", 1);
    path_pub = n.advertise<visualization_msgs::Marker>("/path", 1);
    loop_pub = n.advertise<visualization_msgs::Marker>("/loop", 1);
    approx_pcl_pub_cur = n.advertise<sensor_msgs::PointCloud2>("/approx_pcl_cur", 1);
    approx_pcl_pub_prev = n.advertise<sensor_msgs::PointCloud2>("/approx_pcl_prev", 1);
    approx_pcl_pub_icp = n.advertise<sensor_msgs::PointCloud2>("/approx_pcl_icp", 1);
    input_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/input_cloud", 1);
    filtered_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
    tsdf_pub = n.advertise<visualization_msgs::Marker>("/tsdf", 1);
    ready_flag_pub = n.advertise<std_msgs::String>("/slam6d_listener_ready", 1);

    std_msgs::String ready_msg;
    ready_msg.data = std::string("ready");

    ready_flag_pub.publish(ready_msg);

    // ros loop
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
