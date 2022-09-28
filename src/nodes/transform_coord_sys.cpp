#include <loop_closure/coordinate_systems/coord_sys_transform.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <loop_closure/visualization/ros_viewhelper.h>
#include <loop_closure/path/path.h>
#include <loop_closure/params/loop_closure_params.h>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <eigen_conversions/eigen_msg.h>

namespace fs = boost::filesystem;

LoopClosureParams params;

Path *path;

int global_scan_counter = 0;

bool first_ready = false;

std::pair<std::vector<fs::path>, std::vector<fs::path>> scan_pose_filename_pairs;

ros::Publisher trans_cloud_pub;
ros::Publisher cloud_pub;
ros::Publisher trans_pose_pub;
ros::Publisher pose_pub;
ros::Publisher filename_pub;

sensor_msgs::PointCloud2 pcl_to_ros(pcl::PointCloud<PointType>::Ptr cloud)
{
    sensor_msgs::PointCloud2 transformed;
    pcl::toROSMsg(*cloud.get(), transformed);
    transformed.header.frame_id = "map";
    transformed.header.stamp = ros::Time::now();

    return transformed;
}

/**
 * @brief converts an eigen 4x4 transform matrix to a ros pose stamped, with the stamp being the current time
 *
 * @param transform
 * @return geometry_msgs::PoseStamped
 */
geometry_msgs::PoseStamped transform_to_ros_pose_stamped(Matrix4f &transform)
{
    Eigen::Quaternionf quat(transform.block<3, 3>(0, 0));
    Eigen::Vector3f vec = transform.block<3, 1>(0, 3);
    geometry_msgs::Point ros_point;
    geometry_msgs::Quaternion ros_quat;

    tf::pointEigenToMsg(vec.cast<double>(), ros_point);
    tf::quaternionEigenToMsg(quat.cast<double>(), ros_quat);

    geometry_msgs::PoseStamped pose;
    pose.pose.orientation = ros_quat;
    pose.pose.position = ros_point;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();

    return pose;
}

void prepare()
{
    path = new Path();

    boost::filesystem::path directory_name = fs::path("/home/patrick/data/hannover1/");

    scan_pose_filename_pairs = CoordSysTransform::get_filenames(directory_name);
    size_t num_scans = scan_pose_filename_pairs.first.size();

    // sort the filenames
    std::sort(scan_pose_filename_pairs.first.begin(), scan_pose_filename_pairs.first.end());
    std::sort(scan_pose_filename_pairs.second.begin(), scan_pose_filename_pairs.second.end());

    // for (auto pose_path : scan_pose_filename_pairs.second)
    // {
    //     auto pose_mat = CoordSysTransform::getTransformationFromPose(pose_path);
    //     Pose pose(pose_mat);
    //     path->add_pose(pose);
    // }
}

void publish_next_data()
{
    // no need to proceed, if all data is already published
    if (global_scan_counter >= scan_pose_filename_pairs.first.size())
    {
        // all data is published, shut down
        exit(EXIT_SUCCESS);
    }

    auto scan_path = scan_pose_filename_pairs.first[global_scan_counter];
    auto pose_path = scan_pose_filename_pairs.second[global_scan_counter];

    auto transformed_cloud = CoordSysTransform::read_scan_file_and_transform(scan_path);

    // pcl::io::savePCDFile("/home/patrick/maps/generated/test.pcd", *transformed_cloud);

    // ros::shutdown();

    auto cloud = CoordSysTransform::read_scan_file_and_transform(scan_path);

    auto pose_mat = CoordSysTransform::getTransformationFromPose(pose_path);

    pcl::transformPointCloud(*transformed_cloud.get(), *transformed_cloud.get(), pose_mat);

    // std::cout << "Pose transform for Pose " << pose_path.string() << ": " << std::endl
    //           << pose_mat << std::endl;

    auto trans_pose_marker = ROSViewhelper::initPoseMarker(new Pose(pose_mat));

    sensor_msgs::PointCloud2 ros_cloud = pcl_to_ros(cloud);
    geometry_msgs::PoseStamped ros_pose = transform_to_ros_pose_stamped(pose_mat);

    sensor_msgs::PointCloud2 transformed_ros_cloud = pcl_to_ros(transformed_cloud);

    // std::cout << "Timestamp cloud: " << transformed_ros_cloud.header.stamp << std::endl;
    // std::cout << "Timestamp pose: " << ros_pose.header.stamp << std::endl;

    cloud_pub.publish(ros_cloud);
    // pose_pub.publish(ros_pose);

    trans_cloud_pub.publish(transformed_ros_cloud);

    // trans_pose_pub.publish(trans_pose_marker);
    trans_pose_pub.publish(ros_pose);

    // publish the current filename
    std_msgs::String string_msg;
    string_msg.data = scan_path.string();

    filename_pub.publish(string_msg);

    global_scan_counter++;
}

/**
 * @brief this method handles the subscribtion to the /slam6d_listener_ready topic
 *        a new message to this topic indicates, that the listener is done processing the previously published data
 *        and is ready for something new. This makes sure, that no data ist lost due to queue overload
 *
 * @param msg
 */
void handle_slam6d_listener_ready(const std_msgs::String::ConstPtr &msg)
{
    std::cout << "READY" << std::endl;

    first_ready = true;

    publish_next_data();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_coord_sys");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    params = LoopClosureParams(params);

    // init publishers
    trans_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_transformed", 0);
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/slam6d_cloud", 0);
    trans_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/slam6d_pose", 0);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/pose", 0);
    filename_pub = n.advertise<std_msgs::String>("/slam6d_filename", 0);

    // subscribe to the ready topic
    ros::Subscriber slam6d_listener_ready_sub = n.subscribe("/slam6d_listener_ready", 1, handle_slam6d_listener_ready);

    // prepare everything needed for this node
    prepare();

    ros::Rate r(1.0f);

    while (ros::ok())
    {
        if (!first_ready)
        {
            publish_next_data();
        }

        ros::spinOnce();
        r.sleep();
    }
}