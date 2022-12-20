#include <loop_closure/coordinate_systems/coord_sys_transform.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <loop_closure/visualization/ros_viewhelper.h>
#include <loop_closure/path/path.h>
#include <loop_closure/params/loop_closure_params.h>
#include <loop_closure/util/point.h>
#include <loop_closure/map/global_map.h>
#include <loop_closure/util/csv_wrapper.h>

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <eigen_conversions/eigen_msg.h>
#include <bondcpp/bond.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace fs = boost::filesystem;

LoopClosureParams params;

// prefix for printing
const std::string print_prefix = "[Data-Publishing]: ";

// wrapper for a path
Path *path;
int global_scan_counter = 0;

// vector containing the pcd filenames
std::vector<fs::path> pcd_filenames;

// ros publishers
ros::Publisher cloud_pub;
ros::Publisher pose_pub;
ros::Publisher filename_pub;
ros::Publisher initial_path_pub;

// create a data bound between the data publisher (this node) and the data subscriber
std::unique_ptr<bond::Bond> bond_ptr;

std::unique_ptr<CSVWrapper> csv_wrapper_ptr;

// global map used to read positional data
std::unique_ptr<GlobalMap> map_ptr;

sensor_msgs::PointCloud2 pcl_to_ros(pcl::PointCloud<PointType>::Ptr cloud)
{
    sensor_msgs::PointCloud2 transformed;
    pcl::toROSMsg(*cloud.get(), transformed);
    transformed.header.frame_id = "map";
    transformed.header.stamp = ros::Time::now();

    return transformed;
}

void csv_from_path(std::string name, Path *in_path)
{
    auto path_object = csv_wrapper_ptr->create_object(name);

    CSVWrapper::CSVRow x_coords;
    CSVWrapper::CSVRow y_coords;
    CSVWrapper::CSVRow z_coords;
    CSVWrapper::CSVRow header;

    int cnt = 0;
    for(auto pose : in_path->getPoses())
    {
        auto translation = pose.pos;

        x_coords.add(std::to_string(translation.x()));
        y_coords.add(std::to_string(translation.y()));
        z_coords.add(std::to_string(translation.z()));

        header.add(std::to_string(cnt));
        cnt++;
    }

    path_object->add_row(x_coords);
    path_object->add_row(y_coords);
    path_object->add_row(z_coords);
    path_object->add_row(header);
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

/**
 * @brief Will get the filenames of all pcd files in the passed path object, store them in a vector and return them later
 *
 * @param path
 * @return std::vector<boost::filesystem::path>
 */
std::vector<boost::filesystem::path> get_pcd_filenames(boost::filesystem::path &path)
{
    // check if it is a directory
    if (!boost::filesystem::is_directory(path))
    {
        throw std::invalid_argument("[Coordinate Transformation] The delivered path is not a directory");
    }

    std::vector<boost::filesystem::path> scan_files;
    boost::filesystem::directory_iterator lastFile;

    for (boost::filesystem::directory_iterator it(path); it != lastFile; it++)
    {
        boost::filesystem::path p = it->path();

        if (p.extension().string() == ".pcd")
        {
            scan_files.push_back(p);
        }
    }

    std::cout << print_prefix << "Obtained " << scan_files.size() << " clouds from the delivered location" << std::endl;

    return scan_files;
}

void broadcast_robot_path(Path *path)
{
    // std::cout << "Broadcasting transform.. " << std::endl;

    static tf2_ros::StaticTransformBroadcaster initial_path_br;

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

        transformStamped.transform.rotation.x = current->quat.x();
        transformStamped.transform.rotation.y = current->quat.y();
        transformStamped.transform.rotation.z = current->quat.z();
        transformStamped.transform.rotation.w = current->quat.w();

        initial_path_br.sendTransform(transformStamped);
    }
}

/**
 * @brief Prepares the data for publishing
 *
 */
void prepare()
{
    path = new Path();

    map_ptr.reset(new GlobalMap(params.dataset_params.data_set.h5_file_name));
    // map_ptr.reset(new GlobalMap("/home/patrick/data/physik_unten/physik_unten.h5"));
    // map_ptr.reset(new GlobalMap("/home/patrick/data/vorplatz/vorplatz.h5"));

    auto poses = map_ptr->get_path();

    // create a path from the poses
    for (auto pose : poses)
    {
        path->add_pose(pose);
    }

    csv_wrapper_ptr.reset(new CSVWrapper(params.loop_closure.csv_save_path, ','));
    csv_from_path("initial_path", path);
    csv_wrapper_ptr->write_all();
    csv_wrapper_ptr.release();

    boost::filesystem::path directory_name = fs::path(params.dataset_params.data_set.pcd_cloud_location);
    // boost::filesystem::path directory_name = fs::path("/home/patrick/data/physik_unten/physik_unten");
    // boost::filesystem::path directory_name = fs::path("/home/patrick/data/vorplatz/vorplatz_19_10");

    std::cout << "[PCD_HDF5_PUBLISHER] Using dataset in folder: " << directory_name.string() << std::endl;

    pcd_filenames = get_pcd_filenames(directory_name);

    // sort the filenames (assures correct handling of the files)
    std::sort(pcd_filenames.begin(), pcd_filenames.end());

    // check if the number of poses and scans are identical
    if (path->get_length() != pcd_filenames.size())
    {
        throw std::invalid_argument(print_prefix + " The number of poses and scans are not identical! Aborting...");
    }
}

void publish_next_data()
{
    // no need to proceed, if all data is already published
    if (global_scan_counter == pcd_filenames.size())
    {
        // all data is published, shut down
        std::cout << "[TransformCoordSys] Breaking the Bond to the listener" << std::endl;
        bond_ptr->breakBond();
        return;
    }

    auto scan_path = pcd_filenames[global_scan_counter];
    auto pose_mat = path->at(global_scan_counter)->getTransformationMatrix();

    pcl::PointCloud<PointType>::Ptr cloud_ptr;
    cloud_ptr.reset(new pcl::PointCloud<PointType>());

    int status = pcl::io::loadPCDFile<PointType>(scan_path.string(), *cloud_ptr);

    if (status == -1)
    {
        throw std::logic_error(print_prefix + " Could not read pcd file for path: " + scan_path.string());
    }

    // as these clouds are pretransformed by the respective poses and we do not want that at all, we transform it back
    pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, pose_mat.inverse());

    auto trans_pose_marker = ROSViewhelper::initPoseMarker(new Pose(pose_mat));

    sensor_msgs::PointCloud2 ros_cloud = pcl_to_ros(cloud_ptr);
    geometry_msgs::PoseStamped ros_pose = transform_to_ros_pose_stamped(pose_mat);
    ros_pose.header.stamp = ros_cloud.header.stamp;

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
    std::cout << print_prefix << "Connection established.." << std::endl;

    publish_next_data();
}

void bond_broken_callback()
{
    std::cout << print_prefix << "Done publishing.." << std::endl;

    bond_ptr.release();
    exit(EXIT_SUCCESS);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_hdf5_publisher");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    params = LoopClosureParams(nh);

    // init publishers
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/slam6d_cloud", 0);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/slam6d_pose", 0);
    filename_pub = n.advertise<std_msgs::String>("/slam6d_filename", 0);
    initial_path_pub = n.advertise<nav_msgs::Path>("/initial_path", 0);

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
        initial_path_pub.publish(type_transform::to_ros_path(path->getPoses()));
        // broadcast_robot_path(path);

        ros::spinOnce();
        r.sleep();
    }
}