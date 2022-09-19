/**
 * @file path_listener.cpp
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief
 * @version 0.1
 * @date 2022-05-29
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

// c++ standard related includes
#include <sstream>
#include <iostream>

// external bibs
#include <highfive/H5File.hpp>

// relative includes
#include <loop_closure/map/global_map.h>
#include <loop_closure/map/local_map.h>
#include <loop_closure/ray_tracer/ray_tracer.h>
#include <loop_closure/path/path.h>
#include <loop_closure/options/options_reader.h>
#include <loop_closure/visualization/ros_viewhelper.h>
#include <loop_closure/util/point.h>
#include <loop_closure/util/eigen_vs_ros.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>

RayTracer *tracer;
std::shared_ptr<LocalMap> local_map_ptr;
std::shared_ptr<GlobalMap> global_map_ptr;
Path *path;
Path *optimized_path;
int side_length_xy;
int side_length_z;
lc_options_reader *options;

// distance between poses (min) in path
float PATH_DIST = 0.2f;
float MAX_DIST_LC = 1.0f;
float MIN_TRAVELED_LC = 10.0f;

float cur_length = 0.0f;
Eigen::Vector3f last_vec = Vector3f::Zero();
float last_path_idx = 0;

// ros
ros::Subscriber pose_sub;
ros::Publisher path_pub;
ros::Publisher optimized_path_pub;
ros::Publisher path_marker_pub;
ros::Publisher loop_pub;

// publish approximated cloud for current pose of lc (cur_index > prev_index)
ros::Publisher approx_pcl_pub_cur;
// publish approximated cloud for previous pose of lc (cur_index > prev_index)
ros::Publisher approx_pcl_pub_prev;
//
ros::Publisher approx_pcl_pub_icp;

nav_msgs::Path ros_path;
sensor_msgs::PointCloud2 cur_pcl_msg;
sensor_msgs::PointCloud2 prev_pcl_msg;
sensor_msgs::PointCloud2 icp_pcl_msg;
sensor_msgs::PointCloud2 cur_pcl_filtered_msg;
sensor_msgs::PointCloud2 prev_pcl_filtered_msg;

// gtsam

gtsam::NonlinearFactorGraph graph;

/**
 * @brief initializes the local and global map
 * @todo don't harcode this
 *
 */
void init_maps()
{
    global_map_ptr = std::make_shared<GlobalMap>(options->get_map_file_name());

    // extract attribute data from global map
    auto attribute_data = global_map_ptr->get_attribute_data();

    // init local map using attribute data
    local_map_ptr = std::make_shared<LocalMap>(attribute_data.get_map_size_x(),
                                               attribute_data.get_map_size_y(),
                                               attribute_data.get_map_size_z(),
                                               global_map_ptr, true); // still hardcoded af

    auto &size = local_map_ptr.get()->get_size();
    side_length_xy = size.x() * MAP_RESOLUTION / 1000.0f;
    side_length_z = size.z() * MAP_RESOLUTION / 1000.0f;

    std::cout << "Finished init the maps" << std::endl;
}

void initOptions(int argc, char **argv)
{
    // read options from cmdline
    options = new lc_options_reader();
    int status = options->read_options(argc, argv);

    if (status == 1)
    {
        std::cout << "[CLI] Terminate node, because there were some errors while reading from cmd-line" << std::endl;

        exit(EXIT_FAILURE);
    }
    else if (status == 2)
    {
        std::cout << "[CLI] Terminate node, because help was requested" << std::endl;

        return exit(EXIT_FAILURE);
    }
}

/**
 * @brief method used to initiate all the objects associated with this test case
 *
 */
void init_obj()
{
    // initialize the map based ob the passed arguments first
    init_maps();

    // init ray tracer
    tracer = new RayTracer(options, local_map_ptr, global_map_ptr);

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
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 1>(0, 3) = centroid_diff;
    // transform.block<3, 3>(0, 0) = pose_pretransform.block<3, 3>(0, 0);
    pcl::PointCloud<PointType>::Ptr pointcloud_cur_pretransformed;
    pointcloud_cur_pretransformed.reset(new pcl::PointCloud<PointType>());
    // pcl::transformPointCloud(*pointcloud_cur.get(), *pointcloud_cur_pretransformed.get(), transform);
    //  pcl::transformPointCloud(*pointcloud_cur.get(), *pointcloud_cur_pretransformed.get(), pose_pretransform);

    // fill ros markers
    prev_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(pointcloud_prev);
    // cur_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(pointcloud_cur_pretransformed);
    cur_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(pointcloud_cur);

    // ICP Settings
    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    // icp.setMaxCorrespondenceDistance(0.2f); // hardcoded for now
    icp.setMaximumIterations(200);
    // icp.setTransformationEpsilon(1e-6);
    // icp.setEuclideanFitnessEpsilon(1e-6);
    // icp.setRANSACIterations(0);

    // Align clouds
    // icp.setInputSource(pointcloud_cur_pretransformed);
    icp.setInputSource(pointcloud_cur);
    icp.setInputTarget(pointcloud_prev);
    pcl::PointCloud<PointType>::Ptr icp_result(new pcl::PointCloud<PointType>());
    icp.align(*icp_result);

    // get a aligned cloud marker
    icp_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(icp_result);

    std::cout << "ICP Fitness Score: " << icp.getFitnessScore() << std::endl;
    std::cout << "ICP converged? " << icp.hasConverged() << std::endl;
    std::cout << "Final tranformation: " << std::endl
              << icp.getFinalTransformation() << std::endl;

    // 0.3 from liosam:
    // https://github.com/TixiaoShan/LIO-SAM/blob/e0b77a9654d32350338b7b9246934cff6271bec1/config/params.yaml#L88
    if (icp.hasConverged() == false || icp.getFitnessScore() > 0.3)
    {
        std::cout << "ICP has not converged.." << std::endl;

        throw std::logic_error("ICP not converged, look into this");
    }

    // corrected pointcloud might be published here...

    // Get pose transformation

    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    // LIOSAM
    // transform from world origin to wrong pose
    Eigen::Affine3f tWrong(path->at(loop_key_cur)->getTransformationMatrix());

    // transform from world origin to corrected pose
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    gtsam::Pose3 poseTo(gtsam::Rot3(path->at(lc_indices.first)->quat.cast<double>()),
                        gtsam::Point3(path->at(lc_indices.first)->pos.cast<double>()));

    auto between_fac = gtsam::BetweenFactor<gtsam::Pose3>(lc_indices.second, lc_indices.first, poseFrom.between(poseTo), constraintNoise);

    // ME
    // Eigen::Affine3f tCorrect = correctionLidarFrame;// * tWrong; // pre-multiplying -> successive rotation about a fixed frame
    // pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    // gtsam::Pose3 between_trans = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    // //create between factor
    // auto between_fac = gtsam::BetweenFactor<gtsam::Pose3>(lc_indices.second, lc_indices.first, between_trans, constraintNoise);

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

    for (int i = 1; i < path->get_length(); i++)
    {
        // auto current_pose = path->at(i);
        gtsam::Rot3 rot3(current_pose->rotationMatrixFromQuaternion().cast<double>());
        gtsam::Point3 point3(current_pose->pos.x(), current_pose->pos.y(), current_pose->pos.z());

        gtsam::PriorFactor<gtsam::Pose3> factor(i, gtsam::Pose3(rot3, point3), prior_noise);

        // add prior factor to every pos of the graph
        graph.add(factor);

        // add between factor
        // if (i == 5)//i < path->get_length() - 1)
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

/**
 * @brief Main method
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    initOptions(argc, argv);

    ros::init(argc, argv, "test_lc_detection_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    init_obj();

    // specify ros loop rate
    ros::Rate loop_rate(10);

    path_pub = n.advertise<nav_msgs::Path>("/test_path", 1);
    optimized_path_pub = n.advertise<visualization_msgs::Marker>("/optimized_path", 1);
    path_marker_pub = n.advertise<visualization_msgs::Marker>("/path", 1);
    loop_pub = n.advertise<visualization_msgs::Marker>("/loop", 1);
    approx_pcl_pub_cur = n.advertise<sensor_msgs::PointCloud2>("/approx_pcl_cur", 1);
    approx_pcl_pub_prev = n.advertise<sensor_msgs::PointCloud2>("/approx_pcl_prev", 1);
    approx_pcl_pub_icp = n.advertise<sensor_msgs::PointCloud2>("/approx_pcl_icp", 1);

    // extract poses from global map and add to path
    auto poses = global_map_ptr->get_path();

    for (auto pose : poses)
    {
        path->add_pose(pose);
    }

    // crate path marker
    auto path_marker = ROSViewhelper::initPathMarker(path);

    // variables for the lc detection
    int start_idx = 0;
    int end_idx = path->get_length() - 1;
    bool is_ok = true;
    std::vector<visualization_msgs::Marker> loop_visualizations;

    while (is_ok)
    {
        // find path, including visibility check
        // auto res = path->find_loop_greedy(start_idx, 3.0f, 10.0f, true);
        auto res = path->find_loop_kd_min_dist(start_idx, 3.0, 10.0f, true);

        // found
        if (res.first != -1 && res.second != -1)
        {
            // update start index for possible second closure
            start_idx = res.second;

            std::cout << "Found a closed loop! Between index " << res.first << " and index " << res.second << std::endl;

            loop_visualizations.push_back(ROSViewhelper::init_loop_detected_marker(path->at(res.first)->pos, path->at(res.second)->pos));

            // create a factor graph using the current path and the found loop closure
            create_factor_graph_from_path(path, res);

            // solve the factor graph using levenberg marquardt and get the new Positions of the optimized path
            auto values = solve_factor_graph(path);

            // fill the optimized path with the newly found positions
            fill_optimized_path(values);
        }
        else
        {
            is_ok = false;
        }
    }

    // no loop found = success, exit program
    if (loop_visualizations.size() == 0)
    {
        std::cout << "No Loops found!" << std::endl;
        exit(EXIT_SUCCESS);
    }

    // generate a marker for the optimized path
    auto optimized_path_marker = ROSViewhelper::initPathMarker(optimized_path);

    std::cout << "Found " << loop_visualizations.size() << " loop(s)" << std::endl;

    // ros loop
    while (ros::ok())
    {
        // publish path and loop detects
        path_marker_pub.publish(path_marker);
        optimized_path_pub.publish(optimized_path_marker);
        approx_pcl_pub_cur.publish(cur_pcl_msg);
        approx_pcl_pub_prev.publish(prev_pcl_msg);
        approx_pcl_pub_icp.publish(icp_pcl_msg);

        for (auto marker : loop_visualizations)
        {
            loop_pub.publish(marker);
        }

        // more ros related stuff
        ros::spinOnce();

        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
