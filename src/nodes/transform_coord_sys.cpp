#include <loop_closure/coordinate_systems/coord_sys_transform.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <loop_closure/visualization/ros_viewhelper.h>
#include <loop_closure/path/path.h>
#include <loop_closure/params/loop_closure_params.h>
#include <loop_closure/util/point.h>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <eigen_conversions/eigen_msg.h>
#include <bondcpp/bond.h>

namespace fs = boost::filesystem;

LoopClosureParams params;

Path *path;

int global_scan_counter = 0;

std::pair<std::vector<fs::path>, std::vector<fs::path>> scan_pose_filename_pairs;

ros::Publisher cloud_pub;
ros::Publisher pose_pub;
ros::Publisher filename_pub;
ros::Publisher initial_path_pub;

// create a data bound between the data publisher (this node) and the data subscriber
std::unique_ptr<bond::Bond> bond_ptr;

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

    std::cout << "[TRANSFORM_COORD_SYS] Using dataset in folder: " << directory_name.string() << std::endl;

    scan_pose_filename_pairs = CoordSysTransform::get_filenames(directory_name);
    size_t num_scans = scan_pose_filename_pairs.first.size();

    // sort the filenames
    std::sort(scan_pose_filename_pairs.first.begin(), scan_pose_filename_pairs.first.end());
    std::sort(scan_pose_filename_pairs.second.begin(), scan_pose_filename_pairs.second.end());

    for (auto pose_path : scan_pose_filename_pairs.second)
    {
        auto pose_mat = CoordSysTransform::getTransformationFromPose(pose_path);
        Pose pose(pose_mat);
        path->add_pose(pose);
    }

    // global_scan_counter = 230;
}

void publish_next_data()
{
    // no need to proceed, if all data is already published
    if (global_scan_counter == scan_pose_filename_pairs.first.size())
    {
        // all data is published, shut down
        std::cout << "[TransformCoordSys] Breaking the Bond to the listener" << std::endl;
        bond_ptr->breakBond();
        return;
    }

    // testing
    if (global_scan_counter > 67)
    {
        // all data is published, break the bound and shutdown

        std::cout << "[TransformCoordSys] Breaking the Bond to the listener (test)" << std::endl;
        bond_ptr->breakBond();
        return;
    }

    auto scan_path = scan_pose_filename_pairs.first[global_scan_counter];
    auto pose_path = scan_pose_filename_pairs.second[global_scan_counter];

    auto transformed_cloud = CoordSysTransform::read_scan_file_and_transform(scan_path);

    // pcl::io::savePCDFile("/home/patrick/maps/generated/test.pcd", *transformed_cloud);

    // ros::shutdown();

    auto cloud = CoordSysTransform::read_scan_file_and_transform(scan_path);

    auto pose_mat = CoordSysTransform::getTransformationFromPose(pose_path);

    auto trans_pose_marker = ROSViewhelper::initPoseMarker(new Pose(pose_mat));

    sensor_msgs::PointCloud2 ros_cloud = pcl_to_ros(cloud);
    geometry_msgs::PoseStamped ros_pose = transform_to_ros_pose_stamped(pose_mat);
    ros_pose.header.stamp = ros_cloud.header.stamp;

    sensor_msgs::PointCloud2 transformed_ros_cloud = pcl_to_ros(transformed_cloud);

    // std::cout << "Timestamp cloud: " << ros_cloud.header.stamp << std::endl;
    // std::cout << "Timestamp pose: " << ros_pose.header.stamp << std::endl;

    cloud_pub.publish(ros_cloud);
    pose_pub.publish(ros_pose);

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
    // std::cout << "READY" << std::endl;

    publish_next_data();
}

void bond_formed_callback()
{
    std::cout << "[TransformCoordSys] Connection established.." << std::endl;

    publish_next_data();
}

void bond_broken_callback()
{
    std::cout << "[TransformCoordSys] Done publishing.." << std::endl;

    bond_ptr.reset();
    exit(EXIT_SUCCESS);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_coord_sys");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    params = LoopClosureParams(params);

    // init publishers
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/slam6d_cloud", 0);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/slam6d_pose", 0);
    filename_pub = n.advertise<std_msgs::String>("/slam6d_filename", 0);
    initial_path_pub = n.advertise<visualization_msgs::Marker>("/initial_path", 0);

    // subscribe to the ready topic
    ros::Subscriber slam6d_listener_ready_sub = n.subscribe("/slam6d_listener_ready", 1, handle_slam6d_listener_ready);

    // prepare everything needed for this node
    prepare();

    std::string id = "42"; // id currently hardcoded
    bond_ptr.reset(new bond::Bond("/data_bound", id));
    bond_ptr->setHeartbeatTimeout(1000000);
    bond_ptr->start();
    bond_ptr->setFormedCallback(bond_formed_callback);
    bond_ptr->setBrokenCallback(bond_broken_callback);

    ros::Rate r(1.0f);

    while (ros::ok())
    {
        initial_path_pub.publish(ROSViewhelper::initPathMarker(path, Colors::ColorNames::red));

        ros::spinOnce();
        r.sleep();
    }
}