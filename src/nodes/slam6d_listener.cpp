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

#pragma region INCLUDES

// ros related includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <geometry_msgs/PoseStamped.h>

#include <bondcpp/bond.h>

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
#include <loop_closure/util/lc_evaluator.h>

// transform between ros and eigen
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

// pcl includes
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#pragma endregion

using namespace message_filters;

#pragma region GLOBAL_VARIABLES

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

// evaluation
std::unique_ptr<Evaluator> evaluator;

// vector to store found lc index pairs
std::vector<std::pair<Matrix4f, std::pair<int, int>>> lc_index_pairs;
std::vector<float> lc_fitness_scores;

// save a vector of pointclouds (which should be preprocessed)
std::vector<pcl::PointCloud<PointType>::Ptr> dataset_clouds;

// saves the last initial estimate coming from the slam6d dataset
Matrix4f last_initial_estimate;

// loop closure information ((cur) pose of the last lc before transformation and the transformation difference of the last pose due to the lc)
Pose last_loop_old_pose;
Matrix4f last_loop_transform_diff;

// ROS //
ros::Publisher path_pub;
ros::Publisher optimized_path_pub;
ros::Publisher loop_pub;
ros::Publisher tsdf_pub;
ros::Publisher ready_flag_pub;      // publisher used to signal to it's subscribers, that the node is back in idle mode
ros::Publisher approx_pcl_pub_cur;  // publish approximated cloud for current pose of lc (cur_index > prev_index)
ros::Publisher approx_pcl_pub_prev; // publish approximated cloud for previous pose of lc (cur_index > prev_index)
ros::Publisher approx_pcl_pub_icp;  // result of icp
ros::Publisher input_cloud_pub;     // publishes the input cloud
ros::Publisher tsdf_cloud_pub;
ros::Publisher lc_candidate_publisher; // publishes loop closure candidates
ros::Publisher normal_publisher;       // publishes loop closure candidates

// Ros messages
nav_msgs::Path ros_path;
sensor_msgs::PointCloud2 cur_pcl_msg;
sensor_msgs::PointCloud2 prev_pcl_msg;
sensor_msgs::PointCloud2 icp_pcl_msg;

// Boncpp
std::unique_ptr<bond::Bond> bond_ptr;

#pragma endregion

/**
 * @brief method used to initiate all the objects associated with this node
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

    // create evaluator
    evaluator.reset(new Evaluator(path));

    ros_path.header.frame_id = "map";
    ros_path.header.stamp = ros::Time::now();

    // initialize these values so that transformations have no effect
    last_loop_old_pose.pos = Vector3f::Zero();
    last_loop_old_pose.quat = Eigen::Quaternionf::Identity();
    last_loop_transform_diff = Matrix4f::Identity();
    last_initial_estimate = Matrix4f::Identity();
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
        // if (candidate_pair.first < lc_pair.second.second || path->get_distance_between_path_poses(lc_pair.second.first, lc_pair.second.second) < params.loop_closure.min_traveled_lc)
        // {
        //     return false;
        // }

        // check validity
        if (candidate_pair.second < lc_pair.second.second || path->get_distance_between_path_poses(lc_pair.second.second, candidate_pair.second) < params.loop_closure.dist_between_lcs)
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief will refill the gtsam graph after finding a lc
 *
 */
void refill_graph()
{
    gtsam_wrapper_ptr->reset();

    Matrix4f pose_mat = path->at(0)->getTransformationMatrix();
    gtsam_wrapper_ptr->add_prior_constraint(pose_mat);

    for (int i = 0; i < path->get_length() - 1; i++)
    {
        Matrix4f diff_mat = getTransformationMatrixBetween(path->at(i)->getTransformationMatrix(), path->at(i + 1)->getTransformationMatrix());
        gtsam_wrapper_ptr->add_in_between_contraint(diff_mat, i, i + 1);
    }

    // readd a lc constraints?
    for (int i = 0; i < lc_index_pairs.size(); i++)
    {
        // gtsam_wrapper_ptr->add_in_between_contraint(lc_index_pairs[i].first, lc_index_pairs[i].second.second, lc_index_pairs[i].second.first, lc_fitness_scores[i]);
        gtsam_wrapper_ptr->add_in_between_contraint(lc_index_pairs[i].first, lc_index_pairs[i].second.second, lc_index_pairs[i].second.first);
    }
}

void broadcast_robot_pose(Pose &pose)
{
    // std::cout << "Broadcasting transform.. " << std::endl;

    static tf2_ros::TransformBroadcaster br;

    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "robot";
    transformStamped.transform.translation.x = pose.pos.x();
    transformStamped.transform.translation.y = pose.pos.y();
    transformStamped.transform.translation.z = pose.pos.z();

    auto euler = pose.quat.toRotationMatrix().eulerAngles(0, 1, 2);

    tf2::Quaternion q;
    q.setRPY(euler.x(), euler.y(), euler.z());
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}

void broadcast_robot_path(Path *path)
{
    // std::cout << "Broadcasting transform.. " << std::endl;

    static tf2_ros::StaticTransformBroadcaster path_br;

    for (int idx = 0; idx < path->get_length(); idx++)
    {
        Pose *current = path->at(idx);
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "pose_" + std::to_string(idx);
        transformStamped.transform.translation.x = current->pos.x();
        transformStamped.transform.translation.y = current->pos.y();
        transformStamped.transform.translation.z = current->pos.z();

        auto euler = current->quat.toRotationMatrix().eulerAngles(0, 1, 2);

        tf2::Quaternion q;
        q.setRPY(euler.x(), euler.y(), euler.z());
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        path_br.sendTransform(transformStamped);
    }
}

/**
 * @brief callback if the bond from data publisher to the current node is broken
 *
 */
void clear_and_update_tsdf()
{
    std::cout << "[Slam6D_Listener] Cleanup and write new tsdf" << std::endl;

    // write data
    local_map_ptr->write_back();
    std::cout << "Old map has been written to: " << params.map.filename.string() << std::endl;

    // update filename
    auto previous_filename_path = params.map.filename;
    params.map.filename = previous_filename_path.parent_path() / (boost::filesystem::path(previous_filename_path.stem().string() + "_" + "updated").string() + previous_filename_path.extension().string());

    std::cout << "Start generating the updated map as: " << params.map.filename.string() << std::endl;

    // reset global and localmap
    global_map_ptr.reset(new GlobalMap(params.map));
    local_map_ptr.reset(new LocalMap(params.map.size.x(), params.map.size.y(), params.map.size.y(), global_map_ptr));

    // refill tsdf map
    for (int i = 0; i < path->get_length(); i++)
    {
        auto current_pose_ptr = path->at(i);
        auto pose_mat = current_pose_ptr->getTransformationMatrix();
        auto cloud_ptr = dataset_clouds.at(i);

        // CREATE POINTCLOUD USED FOR TSDF UPDATE
        pcl::PointCloud<PointType>::Ptr tsdf_cloud;

        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(cloud_ptr);
        sor.setLeafSize(params.map.resolution / 1000.0f, params.map.resolution / 1000.0f, params.map.resolution / 1000.0f);
        sor.filter(*tsdf_cloud.get());

        std::vector<Eigen::Vector3i> points_original(tsdf_cloud->size());

        // transform points to map coordinates
#pragma omp parallel for schedule(static) default(shared)
        for (int i = 0; i < tsdf_cloud->size(); ++i)
        {
            const auto &cp = (*tsdf_cloud)[i];
            points_original[i] = Eigen::Vector3i(cp.x * 1000.f, cp.y * 1000.f, cp.z * 1000.f);
        }

        // Shift
        Vector3i input_3d_pos = real_to_map(pose_mat.block<3, 1>(0, 3));
        local_map_ptr->shift(input_3d_pos);

        Eigen::Matrix4i rot = Eigen::Matrix4i::Identity();
        rot.block<3, 3>(0, 0) = to_int_mat(pose_mat).block<3, 3>(0, 0);
        Eigen::Vector3i up = transform_point(Eigen::Vector3i(0, 0, MATRIX_RESOLUTION), rot);

        // create TSDF Volume
        update_tsdf(points_original, input_3d_pos, up, *local_map_ptr, params.map.tau, params.map.max_weight, params.map.resolution);
    }

    local_map_ptr->write_back();

    if (bond_ptr->isBroken())
    {
        bond_ptr.reset();
        exit(EXIT_SUCCESS);
    }
}

/**
 * @brief handles incoming pointcloud2
 *
 * @param cloud_ptr
 */
void handle_slam6d_cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr, const geometry_msgs::PoseStampedConstPtr &pose_ptr)
{

#pragma region FUNCTION_VARIABLES
    // message used to show, that the processing of the last message pair is done and the node awaits new data
    std_msgs::String ready_msg;
    ready_msg.data = std::string("ready");

    Pose input_pose;
    Matrix4f input_pose_mat;
    Matrix4f input_pose_transformed;

#pragma endregion

#pragma region PREPROCESSING

    pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr tsdf_cloud(new pcl::PointCloud<PointType>);

    pcl::fromROSMsg(*cloud_ptr.get(), *input_cloud.get());

    // filter input cloud with a statistical outlier filter:
    std::cout << "[SLAM6D_LISTENER] Start input cloud filtering: " << std::endl;
    std::cout << "[SLAM6D_LISTENER] Size before filtering: " << input_cloud->size() << std::endl;
    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*input_cloud);
    std::cout << "[SLAM6D_LISTENER] Size after filtering: " << input_cloud->size() << std::endl;

    // save cloud
    dataset_clouds.push_back(input_cloud);

    // create Pose from ros pose
    Eigen::Vector3d tmp_point;
    Eigen::Quaterniond tmp_quat;
    Eigen::Affine3d tmp_pose;
    tf::pointMsgToEigen(pose_ptr->pose.position, tmp_point);
    tf::quaternionMsgToEigen(pose_ptr->pose.orientation, tmp_quat);
    tf::poseMsgToEigen(pose_ptr->pose, tmp_pose);

    input_pose = Pose(tmp_pose.matrix().cast<float>());
    // input_pose.quat = tmp_quat.cast<float>();
    // input_pose.pos = tmp_point.cast<float>();

    std::cout << "New Pose with index: " << path->get_length() << ":" << std::endl
              << input_pose << std::endl;

    // add the delta between the initial poses to the last pose of the (maybe already updated) path
    auto initial_pose_delta = getTransformationMatrixBetween(last_initial_estimate, input_pose.getTransformationMatrix());
    auto gtsam_pose_delta = initial_pose_delta;
    last_initial_estimate = input_pose.getTransformationMatrix();

    // fitness score from preregistration (-1.0f = default)
    float prereg_fitness_score = -1.0f;

    if (path->get_length() > 0)
    {
        input_pose_transformed = path->at(path->get_length() - 1)->getTransformationMatrix() * initial_pose_delta;

        // PREREGISTRATION
        // ------------------
        // This section is used to preregister poses following each other in order to get a better initial estimate
        // This is achieved by using GICP/ICP and reevaluating the respective fitness scores
        if (path->get_length() > 1)
        {
            // pose after (possible) preregistration
            Pose preregistered_input_pose;

            pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr scan_cloud(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*dataset_clouds.at(path->get_length() - 1), *model_cloud);
            pcl::copyPointCloud(*input_cloud, *scan_cloud);
            pcl::PointCloud<PointType>::Ptr result_cloud(new pcl::PointCloud<PointType>());

            // gicp variables
            bool converged = false;
            Matrix4f final_transformation = Matrix4f::Identity();

            auto last_pose = path->at(path->get_length() - 1); // obtain last pose

            // this is the transformation from the model coordinate system to the input poses.
            // we transform the model cloud into the coordinate system of the scan cloud (the cloud of the incoming pose)
            // afterwards we perform icp and combine the resulting transformations
            auto model_to_scan_initial = getTransformationMatrixBetween(input_pose_transformed, last_pose->getTransformationMatrix());


            // pretransform model cloud into the system of the scan
            pcl::transformPointCloud(*model_cloud, *model_cloud, model_to_scan_initial);



            // try matching the clouds in the scan system -> we obtain P_scan' -> P_scan (final transformation of ICP)
            // with P_scan' = actual position of the robot when capturing the current scan (according to icp)
            gtsam_wrapper_ptr->perform_pcl_gicp(model_cloud, scan_cloud, result_cloud, converged, final_transformation, prereg_fitness_score);
            // gtsam_wrapper_ptr->perform_vgicp(model_cloud, scan_cloud, result_cloud, converged, final_transformation, prereg_fitness_score);
            // gtsam_wrapper_ptr->perform_pcl_icp(model_cloud, scan_cloud, result_cloud, converged, final_transformation, prereg_fitness_score);
            // gtsam_wrapper_ptr->perform_vgicp(model_cloud, scan_cloud, result_cloud, converged, final_transformation, prereg_fitness_score);
            // gtsam_wrapper_ptr->perform_pcl_normal_icp(model_cloud, scan_cloud, result_cloud, converged, final_transformation, prereg_fitness_score);

            // only if ICP converges and the resulting fitness-score is really low (e.g. MSD < 0.1) we proceed with the preregistration
            if (converged && prereg_fitness_score <= params.loop_closure.max_prereg_icp_fitness)
            {
                std::cout << "PREREGISTRATION SUCCESSFUL!!!" << std::endl;
                std::cout << "Fitness score: " << prereg_fitness_score << std::endl;
                std::cout << "Final preregistration_matrix: " << std::endl
                          << Pose(final_transformation) << std::endl;

                // auto projected_translation = final_transformation.block<3, 1>(0, 3);
                // projected_translation.z() = 0; // ignore translation in z direction
                // Eigen::Matrix3f projected_rotation = final_transformation.block<3, 3>(0, 0);
                // Vector3f angles = projected_rotation.eulerAngles(0, 1, 2);
                // final_transformation = poseFromEuler(projected_translation.x(), projected_translation.y(), projected_translation.z(), 0, 0, angles.z()).getTransformationMatrix();
                // std::cout << "Final preregistration_matrix (projected): " << std::endl
                //           << Pose(final_transformation) << std::endl;

                Matrix4f new_scan_to_model = model_to_scan_initial.inverse() * final_transformation;
                Matrix4f new_scan_to_map = last_pose->getTransformationMatrix() * new_scan_to_model;

                Pose new_scan_to_map_pose(new_scan_to_map);
                // new_scan_to_map_pose.pos.z() = 0;
                // new_scan_to_map_pose.quat.x() = 0;
                // new_scan_to_map_pose.quat.y() = 0;

                new_scan_to_model = getTransformationMatrixBetween(last_pose->getTransformationMatrix(), new_scan_to_map_pose.getTransformationMatrix());

                path->add_pose(new_scan_to_map_pose);
                // path->add_pose(new_scan_to_map);
                gtsam_pose_delta = new_scan_to_model;
            }
            else // if it is not converged we simply add the initial estimate.
            {
                std::cout << "PREREGISTRATION FAILED!!!" << std::endl;
                std::cout << "Fitness score: " << prereg_fitness_score << std::endl;

                path->add_pose(Pose(input_pose_transformed));
            }
        }
        else
        {
            path->add_pose(Pose(input_pose_transformed));
        }

        // END PREREGISTRATION
    }
    else
    {
        path->add_pose(input_pose);
    }

    // obtain newly inserted pose
    input_pose = *path->at(path->get_length() - 1);
    input_pose_mat = input_pose.getTransformationMatrix();

    std::cout << "[Slam6D_Listener] Pose (after prereg): " << input_pose << std::endl;

    // publish cloud and transform for robot coordinate system
    broadcast_robot_pose(*path->at(path->get_length() - 1));
    broadcast_robot_path(path);

    sensor_msgs::PointCloud2 input_cloud_tf;
    pcl::toROSMsg(*input_cloud, input_cloud_tf);
    input_cloud_tf.header.frame_id = "robot";
    input_cloud_pub.publish(input_cloud_tf);

    // publish normals for the input cloud
    // auto normal_marker = ROSViewhelper::visualize_pcl_normals(input_cloud);
    // normal_marker.header.frame_id = "robot";
    // normal_publisher.publish(normal_marker);

#pragma endregion

#pragma region TSDF_UPDATE

    // CREATE POINTCLOUD USED FOR TSDF UPDATE
    pcl::VoxelGrid<PointType> grid;
    grid.setInputCloud(input_cloud);
    grid.setLeafSize(params.map.resolution / 1000.0f, params.map.resolution / 1000.0f, params.map.resolution / 1000.0f);
    grid.filter(*tsdf_cloud);

    pcl::transformPointCloud(*tsdf_cloud, *tsdf_cloud, input_pose_mat);

    std::vector<Eigen::Vector3i> points_original(tsdf_cloud->size());

    // transform points to map coordinates
#pragma omp parallel for schedule(static) default(shared)
    for (int i = 0; i < tsdf_cloud->size(); ++i)
    {
        const auto &cp = (*tsdf_cloud)[i];
        points_original[i] = Eigen::Vector3i(cp.x * 1000.f, cp.y * 1000.f, cp.z * 1000.f);
    }

    // Shift
    Vector3i input_3d_pos = real_to_map(input_pose_mat.block<3, 1>(0, 3));

    std::cout << "SHIFT_POSITION" << std::endl
              << input_3d_pos << std::endl;
    local_map_ptr->shift(input_3d_pos);

    Eigen::Matrix4i rot = Eigen::Matrix4i::Identity();
    rot.block<3, 3>(0, 0) = to_int_mat(input_pose_mat).block<3, 3>(0, 0);
    Eigen::Vector3i up = transform_point(Eigen::Vector3i(0, 0, MATRIX_RESOLUTION), rot);

    // create TSDF Volume
    update_tsdf(points_original, input_3d_pos, up, *local_map_ptr, params.map.tau, params.map.max_weight, params.map.resolution);
    local_map_ptr->write_back();

#pragma endregion

#pragma region CLOUD_GM_VIS
    auto gm_data = global_map_ptr->get_full_data();
    auto marker = ROSViewhelper::marker_from_gm_read(gm_data);
    // auto marker = ROSViewhelper::initTSDFmarkerPose(local_map_ptr, new Pose(pose));
    tsdf_pub.publish(marker);

    sensor_msgs::PointCloud2 filtered_ros_cloud;
    pcl::toROSMsg(*tsdf_cloud, filtered_ros_cloud);
    tsdf_cloud_pub.publish(filtered_ros_cloud);

    path_pub.publish(ROSViewhelper::initPathMarker(path, Colors::ColorNames::lime));

#pragma endregion

#pragma region GTSAM_EDGE_CONSTRAINTS

    // add gtsam constraints

    // right here at least one object has already been added
    if (path->get_length() == 1)
    {
        Matrix4f pose_mat = path->at(0)->getTransformationMatrix();
        gtsam_wrapper_ptr->add_prior_constraint(pose_mat);

        ready_flag_pub.publish(ready_msg);

        return;
    }
    else
    {
        int from_idx = path->get_length() - 2;
        int to_idx = path->get_length() - 1;
        // auto transform_diff = getTransformationMatrixBetween(path->at(from_idx)->getTransformationMatrix(), path->at(to_idx)->getTransformationMatrix());
        // gtsam_wrapper_ptr->add_in_between_contraint(transform_diff, from_idx, to_idx);

        // when a new pose is added, the between factor added to the graph is the same as the initial delta
        gtsam_wrapper_ptr->add_in_between_contraint(gtsam_pose_delta, from_idx, to_idx, prereg_fitness_score);
    }
#pragma endregion

#pragma region LOOP_IDENTIFICATION

    // check for loops
    std::vector<std::pair<int, int>> lc_pairs;
    if (path->get_length() > 231 && path->get_length() < 270)
    {
        lc_pairs = path->find_loop_kd_min_dist_backwards(path->get_length() - 1, 20, 4, false);
    }
    else
    {
        lc_pairs = path->find_loop_kd_min_dist_backwards(path->get_length() - 1, params.loop_closure.max_dist_lc, params.loop_closure.min_traveled_lc, false);
    }
    std::vector<std::pair<Matrix4f, std::pair<int, int>>> lc_candidate_pairs;

    // check the potentially found loop closure candidates

    // when no pair is returned, go to the next pose
    if (lc_pairs.size() == 0)
    {
        std::cout << "No loop found when inserting Pose" << path->get_length() << ":" << std::endl
                  << *path->at(path->get_length() - 1) << std::endl;

        ready_flag_pub.publish(ready_msg);

        return;
    }
    else
    {
        std::cout << "Found " << lc_pairs.size() << " loop closure pairs" << std::endl;
    }

    for (auto lc_pair : lc_pairs)
    {

        // std::cout << "Found LC Candidate between pose " << lc_pair.first << " and " << lc_pair.second << std::endl;

        // check if the detected loop might be viable considering already found loops
        if (!is_viable_lc(lc_pair))
        {
            // std::cout << "LC pair not viable!" << std::endl;

            continue;
        }
        else
        {
            // std::cout << "Possible LC found between " << lc_pair.first << " and " << lc_pair.second << std::endl;
            lc_candidate_pairs.push_back(std::make_pair(Matrix4f::Identity(), lc_pair));
        }
    }

    // publish candidate
    auto lc_candidate_markers = ROSViewhelper::init_loop_detected_marker_multiple(path, lc_candidate_pairs, Colors::ColorNames::black);

    lc_candidate_publisher.publish(lc_candidate_markers);

#pragma endregion

#pragma region LOOP_EXAMINATION

    bool converged = false;

    for (auto lc_pair : lc_candidate_pairs)
    {
        //////////////////////////////////////////////////////////////
        // The identified loops are only candidates at the moment.
        // To verify them we need to check if a scan matching between
        // The model (aka the pointcloud of the previous pose) and
        // The scan (aka the pointcloud of the current)
        // not only converges, but also has minimal error.
        // To ensure this, ICP needs to have a low MSD between the clouds
        // after registration (e.g. 0.3)
        //
        // NORMALLY one would pretransform the scan towards the model
        // using the inital estimates. Here we proceed the other way
        // round:
        //
        // We transform the model into the coordinate system of the
        // scan, because it is the current robot position and therefore
        // the current coordinate system of the roboter.
        // So all this is actually for display purposes.
        //
        // This transformation needs to be considered later on in the
        // following manner:
        // TODO: add math
        /////////////////////////////////////////////////////////////

        pcl::PointCloud<PointType>::Ptr icp_cloud;
        icp_cloud.reset(new pcl::PointCloud<PointType>());

        auto cur_to_map = path->at(lc_pair.second.second)->getTransformationMatrix();
        auto prev_to_map = path->at(lc_pair.second.first)->getTransformationMatrix();

        // transform from the current pose (aka the scan) to the previous pose (aka the model) (aka.: switching from current coordinate system to the previous)
        // this has to be added to the final transformation!!!!
        // auto cur_to_prev_initial = getTransformationMatrixBetween(prev_to_map, cur_to_map);
        auto prev_to_cur_initial = getTransformationMatrixBetween(cur_to_map, prev_to_map);

        // TODO: evaluate the performance of gicp based on this
        // if ((cur_to_map.block<3, 1>(0, 3) - prev_to_map.block<3, 1>(0, 3)).norm() > params.loop_closure.max_dist_lc / 2.0f)
        // {
        //     Matrix4f tmp = Matrix4f::Identity();

        //     tmp.block<3, 3>(0, 0) = prev_to_cur_initial.block<3, 3>(0, 0);

        //     std::cout << "It's far... !!!" << std::endl;
        //     std::cout << "PrevToCur:" << std::endl
        //               << Pose(prev_to_cur_initial) << std::endl;
        //     std::cout << "OnlyRot:" << std::endl
        //               << Pose(tmp) << std::endl;

        //     prev_to_cur_initial = tmp;
        // }

        pcl::PointCloud<PointType>::Ptr model_cloud;
        pcl::PointCloud<PointType>::Ptr scan_cloud;
        model_cloud.reset(new pcl::PointCloud<PointType>(*dataset_clouds[lc_pair.second.first]));
        scan_cloud.reset(new pcl::PointCloud<PointType>(*dataset_clouds[lc_pair.second.second]));

        // transform the scan cloud into the coordinate system of the model cloud using the initial estimate
        // pcl::transformPointCloud(*scan_cloud, *scan_cloud, cur_to_prev_initial);

        // transform the model cloud into the coordinate system of the scan cloud using the initial estimate
        pcl::transformPointCloud(*model_cloud, *model_cloud, prev_to_cur_initial);

        Matrix4f final_transformation = Matrix4f::Identity();
        float fitness_score = 10.0f;
        bool converged_unit = gtsam_wrapper_ptr->add_loop_closure_constraint(lc_pair.second, model_cloud, scan_cloud,
                                                                             icp_cloud, fitness_score, final_transformation, prev_to_cur_initial, path);

        // add lc if unit converged, update converge flag for all found lcs for the current position
        if (converged_unit)
        {
            converged = true;
            lc_index_pairs.push_back(std::make_pair(final_transformation, lc_pair.second));
            lc_fitness_scores.push_back(fitness_score);
            // break;
        }

        else
        {
            // std::cout << "Unit not converged" << std::endl;
        }

        cur_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(scan_cloud, "robot");
        prev_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(model_cloud, "robot");
        icp_pcl_msg = ROSViewhelper::marker_from_pcl_pointcloud(icp_cloud, "robot");

        // std::cout << "Number of points in icp cloud: " << icp_cloud->size() << std::endl;

        approx_pcl_pub_cur.publish(cur_pcl_msg);
        approx_pcl_pub_prev.publish(prev_pcl_msg);
        approx_pcl_pub_icp.publish(icp_pcl_msg);
    }

    // if the lc found did not converge, we skip everything else
    if (!converged)
    {
        ready_flag_pub.publish(ready_msg);

        return;
    }

#pragma endregion

    // THE FOLLOWING IS ONLY EXECUTED, WHEN A LOOP CLOSURE WAS IDENTIFIED AND SCAN MATCHING CONVERGED IN A NICE MANNER //

#pragma region LOOP_PROCESSING_AND_VISUALIZATION
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

    // VERY IMPORTANT: perform graph optimization using the last initial estimate
    auto new_values = gtsam_wrapper_ptr->perform_optimization(initial);

    fill_optimized_path(new_values);

    optimized_path_pub.publish(ROSViewhelper::initPathMarker(optimized_path, Colors::ColorNames::fuchsia));

    // copy values from optimized path back to the path
    path = new Path(*optimized_path);

    // publish loops again to be consistant
    lc_marker = ROSViewhelper::init_loop_detected_marker_multiple(path, lc_index_pairs);
    loop_pub.publish(lc_marker);

    // refill the graph based on the updated positions
    refill_graph();

#pragma endregion

    // clear and update the tsdf
    clear_and_update_tsdf();

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

    message_filters::Subscriber<sensor_msgs::PointCloud2> slam6d_cloud_sub(n, "/slam6d_cloud", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> slam6d_pose_sub(n, "/slam6d_pose", 1);
    message_filters::Subscriber<std_msgs::String> slam6d_filename_sub(n, "/slam6d_filename", 1);

    typedef sync_policies::ExactTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(40), slam6d_cloud_sub, slam6d_pose_sub);
    sync.registerCallback(boost::bind(&handle_slam6d_cloud_callback, _1, _2));

    optimized_path_pub = n.advertise<visualization_msgs::Marker>("/optimized_path", 1);
    path_pub = n.advertise<visualization_msgs::Marker>("/path", 1);
    loop_pub = n.advertise<visualization_msgs::Marker>("/loop", 1);
    approx_pcl_pub_cur = n.advertise<sensor_msgs::PointCloud2>("/approx_pcl_cur", 1);
    approx_pcl_pub_prev = n.advertise<sensor_msgs::PointCloud2>("/approx_pcl_prev", 1);
    approx_pcl_pub_icp = n.advertise<sensor_msgs::PointCloud2>("/approx_pcl_icp", 1);
    input_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/input_cloud", 1);
    tsdf_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/tsdf_cloud", 1);
    tsdf_pub = n.advertise<visualization_msgs::Marker>("/tsdf", 1);
    ready_flag_pub = n.advertise<std_msgs::String>("/slam6d_listener_ready", 1);
    lc_candidate_publisher = n.advertise<visualization_msgs::Marker>("/lc_candidates", 1);
    normal_publisher = n.advertise<visualization_msgs::Marker>("/input_cloud_normals", 1);

    // bond
    std::string id = "42";
    bond_ptr.reset(new bond::Bond("/data_bound", id));
    bond_ptr->setHeartbeatTimeout(1000000);
    bond_ptr->start();
    bond_ptr->setBrokenCallback(clear_and_update_tsdf);

    ros::spin();

    return EXIT_SUCCESS;
}
