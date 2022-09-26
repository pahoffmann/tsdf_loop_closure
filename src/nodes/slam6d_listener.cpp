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

#include <loop_closure/params/loop_closure_params.h>

#include <loop_closure/map/global_map.h>
#include <loop_closure/map/local_map.h>
#include <loop_closure/ray_tracer/ray_tracer.h>
#include <loop_closure/path/path.h>
#include <loop_closure/visualization/ros_viewhelper.h>
#include <loop_closure/util/point.h>
#include <loop_closure/util/eigen_vs_ros.h>
#include <loop_closure/util/update_tsdf.h>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

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
Path *path;
Path *optimized_path;
int side_length_xy;
int side_length_z;

// vector to store found lc index pairs
std::vector<std::pair<int, int>> lc_index_pairs;

// save a vector of pointclouds (which should be preprocessed)
std::vector<pcl::PointCloud<PointType>::Ptr> dataset_clouds;

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

// gtsam
// factor graph used to store information about the path of the slam6d inital approximation
gtsam::NonlinearFactorGraph graph;

/**
 * @brief method used to initiate all the objects associated with this test case
 *
 */
void init_obj()
{
    // create maps
    global_map_ptr.reset(new GlobalMap(params.map));
    local_map_ptr.reset(new LocalMap(params.map.size.x(), params.map.size.y(), params.map.size.y(), global_map_ptr));

    // init ray tracer
    tracer = new RayTracer(params, local_map_ptr, global_map_ptr);

    // init path
    path = new Path();
    path->attach_raytracer(tracer);

    ros_path.header.frame_id = "map";
    ros_path.header.stamp = ros::Time::now();
}

gtsam::BetweenFactor<gtsam::Pose3> estimate_loop_closure_between_factor(std::pair<int, int> lc_indices)
{
    int loop_key_cur = lc_indices.second;
    int loop_key_pre = lc_indices.first;

    // estimate point-clouds
    // this is pretty runtime intensive right now
    // it might be clever to reduce the number of rays here
    auto pointcloud_prev = tracer->approximate_pointcloud(path->at(loop_key_pre));
    auto pointcloud_cur = tracer->approximate_pointcloud(path->at(loop_key_cur));

    std::cout << "Previous pose: " << std::endl << *path->at(loop_key_pre) << std::endl;
    std::cout << "Current pose: " << std::endl << *path->at(loop_key_cur) << std::endl;

    // todo: instead of approximating the pcl's, use the actual clouds

    // as the estimated clouds will be very far from each other most of the time, we will
    // pretransform the current pcl

    Eigen::Vector4d centroid_prev, centroid_cur;
    pcl::compute3DCentroid(*pointcloud_cur.get(), centroid_cur);
    pcl::compute3DCentroid(*pointcloud_prev.get(), centroid_prev);

    std::cout << "Computed centroid cur: " << std::endl
              << centroid_cur << std::endl;
    std::cout << "Computed centroid prev: " << std::endl
              << centroid_prev << std::endl;

    // calculate centoid diff from cur to prev
    Eigen::Vector3f centroid_diff(centroid_prev.x() - centroid_cur.x(), centroid_prev.y() - centroid_cur.y(), centroid_prev.z() - centroid_cur.z());

    std::cout << "Computed centroid diff: " << std::endl
              << centroid_diff << std::endl;

    // filter outliers
    // Create the filtering object

    std::cout << "Before filter size: " << pointcloud_cur->size() << std::endl;

    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud(pointcloud_cur);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*pointcloud_cur);

    std::cout << "After filter size: " << pointcloud_cur->size() << std::endl;

    std::cout << "Before filter size: " << pointcloud_prev->size() << std::endl;

    sor.setInputCloud(pointcloud_prev);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*pointcloud_prev);

    std::cout << "After filter size: " << pointcloud_prev->size() << std::endl;

    // pretransform cur with centroid diff

    // get pretransform rotation from pose diff
    // auto pose_pretransform = getTransformationMatrixDiff(path->at(loop_key_cur)->getTransformationMatrix(), path->at(loop_key_pre)->getTransformationMatrix());

    // pretransform cur cloud with rotation pose diff and translation diff from centroids
    // as icp cannot work with changes this big.
    // Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // transform.block<3, 1>(0, 3) = centroid_diff;
    // transform.block<3, 3>(0, 0) = pose_pretransform.block<3, 3>(0, 0);
    // pcl::PointCloud<PointType>::Ptr pointcloud_cur_pretransformed;
    // pointcloud_cur_pretransformed.reset(new pcl::PointCloud<PointType>());
    // pcl::transformPointCloud(*pointcloud_cur.get(), *pointcloud_cur_pretransformed.get(), transform);
    //  pcl::transformPointCloud(*pointcloud_cur.get(), *pointcloud_cur_pretransformed.get(), pose_pretransform);

    // fill ros markers
    prev_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(pointcloud_prev);
    // cur_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(pointcloud_cur_pretransformed);
    cur_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(pointcloud_cur);

    // ICP Settings
    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    // icp.setMaxCorrespondenceDistance(0.2f); // hardcoded for now
    icp.setMaximumIterations(params.loop_closure.max_icp_iterations);
    // icp.setTransformationEpsilon(1e-6);
    // icp.setEuclideanFitnessEpsilon(1e-6);
    // icp.setRANSACIterations(0);

    // Align clouds
    // icp.setInputSource(pointcloud_cur_pretransformed);
    icp.setInputSource(pointcloud_cur);
    icp.setInputTarget(pointcloud_prev);
    pcl::PointCloud<PointType>::Ptr icp_result(new pcl::PointCloud<PointType>());
    icp.align(*icp_result);

    // general icp
    /*static pcl::GeneralizedIterativeClosestPoint<PointType, PointType> g_icp;
    g_icp.setMaximumIterations(params.loop_closure.max_icp_iterations);

    g_icp.setInputSource(pointcloud_cur);
    g_icp.setInputTarget(pointcloud_prev);
    pcl::PointCloud<PointType>::Ptr g_icp_result(new pcl::PointCloud<PointType>());
    g_icp.align(*g_icp_result);*/

    // get a aligned cloud marker
    icp_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(icp_result);
    // icp_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(g_icp_result);

    std::cout << "ICP Fitness Score: " << icp.getFitnessScore() << std::endl;
    std::cout << "ICP converged? " << icp.hasConverged() << std::endl;
    std::cout << "Final tranformation: " << std::endl
              << icp.getFinalTransformation() << std::endl;

    // std::cout << "ICP Fitness Score: " << g_icp.getFitnessScore() << std::endl;
    // std::cout << "ICP converged? " << g_icp.hasConverged() << std::endl;
    // std::cout << "Final tranformation: " << std::endl
    //           << g_icp.getFinalTransformation() << std::endl;

    // 0.3 from liosam:
    // https://github.com/TixiaoShan/LIO-SAM/blob/e0b77a9654d32350338b7b9246934cff6271bec1/config/params.yaml#L88
    if (icp.hasConverged() == false || icp.getFitnessScore() > 0.3)
    {
        std::cout << "ICP has not converged.." << std::endl;

        // throw std::logic_error("ICP not converged, look into this");
    }

    // if (g_icp.hasConverged() == false || g_icp.getFitnessScore() > 0.3)
    // {
    //     std::cout << "ICP has not converged.." << std::endl;

    //     // throw std::logic_error("ICP not converged, look into this");
    // }

    // corrected pointcloud might be published here...

    // Get pose transformation

    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    // float noiseScore = g_icp.getFitnessScore();
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    // correctionLidarFrame = g_icp.getFinalTransformation();

    // LIOSAM
    // transform from world origin to wrong pose
    // Eigen::Affine3f tWrong(path->at(loop_key_cur)->getTransformationMatrix());

    // // transform from world origin to corrected pose
    // Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
    // pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

    // gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    // gtsam::Pose3 poseTo(gtsam::Rot3(path->at(lc_indices.first)->quat.cast<double>()),
    //                     gtsam::Point3(path->at(lc_indices.first)->pos.cast<double>()));

    // auto between_fac = gtsam::BetweenFactor<gtsam::Pose3>(lc_indices.second, lc_indices.first, poseFrom.between(poseTo), constraintNoise);

    // ME
    Eigen::Affine3f tCorrect = correctionLidarFrame; // * tWrong; // pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 between_trans = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    // create between factor
    auto between_fac = gtsam::BetweenFactor<gtsam::Pose3>(lc_indices.second, lc_indices.first, between_trans, constraintNoise);

    return between_fac;
}

/**
 * @brief Create a factor graph from path object,
 *
 */
void create_factor_graph_from_path(Path *path, std::pair<int, int> lc_pair_indices)
{
    // SIEHE:
    // https://gtsam.org/tutorials/intro.html#magicparlabel-65728
    // 20cm noise in x, y und z Richtung, 0.1 radiants fehler in z richtung
    // gtsam::Vector6 noise_vec;
    // noise_vec << 0.5, 0.5, 0.5, 0.3, 0.3, 0.3;

    // SIEHE:
    // https://github.com/TixiaoShan/LIO-SAM/blob/6665aa0a4fcb5a9bb3af7d3923ae4a035b489d47/src/mapOptmization.cpp#L1385
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
    gtsam::noiseModel::Diagonal::shared_ptr in_between_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    // gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
    //     gtsam::noiseModel::Diagonal::Sigmas(noise_vec);

    auto current_pose = path->at(0);
    gtsam::Rot3 rot3_prior(current_pose->rotationMatrixFromQuaternion().cast<double>());
    gtsam::Point3 point3_prior(current_pose->pos.x(), current_pose->pos.y(), current_pose->pos.z());

    gtsam::PriorFactor<gtsam::Pose3> factor(0, gtsam::Pose3(rot3_prior, point3_prior), prior_noise);

    // add prior factor to every pos of the graph
    graph.add(factor);

    for (int i = 0; i < path->get_length(); i++)
    {
        // // auto current_pose = path->at(i);
        gtsam::Rot3 rot3(current_pose->rotationMatrixFromQuaternion().cast<double>());
        gtsam::Point3 point3(current_pose->pos.x(), current_pose->pos.y(), current_pose->pos.z());

        gtsam::PriorFactor<gtsam::Pose3> factor(i, gtsam::Pose3(rot3, point3), prior_noise);

        // add prior factor to every pos of the graph
        graph.add(factor);

        // add between factor
        // if (i < path->get_length() - 1)
        // {
        //     auto pose_diff = getTransformationMatrixDiff(path->at(i)->getTransformationMatrix(), path->at(i + 1)->getTransformationMatrix());

        //     gtsam::Rot3 rot3(pose_diff.block<3, 3>(0, 0).cast<double>());
        //     Vector3f pos_diff = pose_diff.block<3, 1>(0, 3);
        //     gtsam::Point3 point3(pos_diff.x(), pos_diff.y(), pos_diff.z());

        //     graph.add(gtsam::BetweenFactor<gtsam::Pose3>(i, i + 1, gtsam::Pose3(rot3, point3), in_between_noise));
        // }
    }

    // add lc constraint
    auto lc_pos_1 = path->at(lc_pair_indices.first);
    auto lc_pos_2 = path->at(lc_pair_indices.second);

    // estimate the lc in between factor
    auto lc_between_fac = estimate_loop_closure_between_factor(lc_pair_indices);

    // calculate the pose differences between the poses participating in the loop closure
    auto lc_pose_diff = getTransformationMatrixDiff(lc_pos_2->getTransformationMatrix(), lc_pos_1->getTransformationMatrix());

    Vector3f pos_diff = lc_pose_diff.block<3, 1>(0, 3);
    gtsam::Rot3 rot3(lc_pose_diff.block<3, 3>(0, 0).cast<double>());
    gtsam::Point3 point3(pos_diff.x(), pos_diff.y(), pos_diff.z());

    // lc constraint added here
    // siehe auch
    // https://github.com/TixiaoShan/LIO-SAM/blob/e0b77a9654d32350338b7b9246934cff6271bec1/src/imuPreintegration.cpp
    // Zeile 1488
    // graph.add(gtsam::BetweenFactor<gtsam::Pose3>(lc_pair_indices.second + 1, lc_pair_indices.first + 1, gtsam::Pose3(rot3, point3)));
    graph.add(lc_between_fac);
}

/**
 * @brief will solve a previously filled factor graph using initial values
 *
 * @return gtsam::Values  the resulting position of the initial values
 */
gtsam::Values solve_factor_graph(Path *path)
{
    gtsam::Values initial;

    // fill initial values with path positions
    for (int i = 0; i < path->get_length(); i++)
    {
        auto current_pose = path->at(i);
        gtsam::Rot3 rot3(path->at(i)->rotationMatrixFromQuaternion().cast<double>());
        gtsam::Point3 point3(current_pose->pos.x(), current_pose->pos.y(), current_pose->pos.z());

        initial.insert(i, gtsam::Pose3(rot3, point3));
    }

    double error_prev = graph.error(initial);

    std::cout << "Error (previous): " << error_prev << std::endl;

    // optimize intial values using levenberg marquardt
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    auto values = optimizer.optimize();

    std::cout << "Error (after): " << graph.error(values) << std::endl;

    return values;
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

inline Eigen::Matrix4i to_int_mat(const Eigen::Matrix4f &mat)
{
    return (mat * MATRIX_RESOLUTION).cast<int>();
}

inline Eigen::Vector3i transform_point(const Eigen::Vector3i &input, const Eigen::Matrix4i &mat)
{
    Eigen::Vector4i v;
    v << input, 1;
    return (mat * v).block<3, 1>(0, 0) / MATRIX_RESOLUTION;
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

    std_msgs::String ready_msg;
    ready_msg.data = std::string("ready");

    // filter input cloud:
    pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*cloud_ptr.get(), *input_cloud.get());
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(input_cloud);
    sor.setLeafSize(params.map.resolution / 1000.0f, params.map.resolution / 1000.0f, params.map.resolution / 1000.0f);
    sor.filter(*filtered_cloud.get());

    // save 
    dataset_clouds.push_back(filtered_cloud);

    // create Pose from ros pose
    Pose input_pose;
    Eigen::Vector3d tmp_point;
    Eigen::Quaterniond tmp_quat;
    tf::pointMsgToEigen(pose_ptr->pose.position, tmp_point);
    tf::quaternionMsgToEigen(pose_ptr->pose.orientation, tmp_quat);
    input_pose.quat = tmp_quat.cast<float>();
    input_pose.pos = tmp_point.cast<float>();

    Matrix4f pose = input_pose.getTransformationMatrix();

    // update tsdf
    std::vector<Eigen::Vector3i> points_original(filtered_cloud->size());
    // std::vector<Eigen::Vector3i> points_original(input_cloud->size());

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

    path->add_pose(input_pose);

    local_map_ptr->write_back();

    // auto gm_data = global_map_ptr->get_full_data();
    // auto marker = ROSViewhelper::marker_from_gm_read(gm_data);
    //auto marker = ROSViewhelper::initTSDFmarkerPose(local_map_ptr, new Pose(pose));

    // tsdf_pub.publish(marker);
    input_cloud_pub.publish(*cloud_ptr);

    sensor_msgs::PointCloud2 filtered_ros_cloud;
    pcl::toROSMsg(*filtered_cloud, filtered_ros_cloud);
    filtered_cloud_pub.publish(filtered_ros_cloud);

    path_pub.publish(ROSViewhelper::initPathMarker(path));

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
            std::cout << "LC found between " << lc_pair.first << " and " << lc_pair.second << std::endl;

            lc_index_pairs.push_back(lc_pair);
        }
    }
    else
    {
        std::cout << "No loop found when inserting Pose" << path->get_length() << std::endl;
        ready_flag_pub.publish(ready_msg);

        return;
    }

    // auto lc_marker = ROSViewhelper::init_loop_detected_marker(path->at(lc_pair.first)->pos, path->at(lc_pair.second)->pos);
    auto lc_marker = ROSViewhelper::init_loop_detected_marker_multiple(path, lc_index_pairs);

    loop_pub.publish(lc_marker);

    // gtsam optimizations

    // create a factor graph using the current path and the found loop closure
    create_factor_graph_from_path(path, lc_pair);

    // solve the factor graph using levenberg marquardt and get the new Positions of the optimized path
    auto values = solve_factor_graph(path);

    // fill the optimized path with the newly found positions
    fill_optimized_path(values);

    // std::cout << std::endl << "Values in optimized path: " << optimized_path->get_length() << std::endl;

    optimized_path_pub.publish(ROSViewhelper::initPathMarker(optimized_path, Colors::ColorNames::purple));

    std::cout << "Points in msg: " << cur_pcl_msg.data.size() << std::endl;

    approx_pcl_pub_cur.publish(cur_pcl_msg);
    approx_pcl_pub_prev.publish(prev_pcl_msg);
    approx_pcl_pub_icp.publish(icp_pcl_msg);

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

    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped, std_msgs::String> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(40), slam6d_cloud_sub, slam6d_pose_sub, slam6d_filename_sub);
    sync.registerCallback(boost::bind(&handle_slam6d_cloud_callback, _1, _2, _3));

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
