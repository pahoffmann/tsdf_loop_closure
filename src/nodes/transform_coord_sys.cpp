#include <loop_closure/coordinate_systems/coord_sys_transform.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <loop_closure/visualization/ros_viewhelper.h>
#include <loop_closure/path/path.h>

namespace fs = boost::filesystem;

sensor_msgs::PointCloud2 pcl_to_ros(pcl::PointCloud<PointType>::Ptr cloud)
{
    sensor_msgs::PointCloud2 transformed;
    pcl::toROSMsg(*cloud.get(), transformed);
    transformed.header.frame_id = "map";
    transformed.header.stamp = ros::Time::now();

    return transformed;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_coord_sys");
    ros::NodeHandle nh("~");

    ros::Publisher trans_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/transformed_cloud", 0);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 0);
    ros::Publisher trans_pose_pub = nh.advertise<visualization_msgs::Marker>("/transformed_pose", 0);
    ros::Publisher pose_pub = nh.advertise<visualization_msgs::Marker>("/pose", 0);

    boost::filesystem::path directory_name = fs::path("/home/patrick/data/hannover1/");

    auto scan_pose_filename_pairs = CoordSysTransform::get_filenames(directory_name);
    size_t num_scans = scan_pose_filename_pairs.first.size();

    // sort the filenames
    std::sort(scan_pose_filename_pairs.first.begin(), scan_pose_filename_pairs.first.end());
    std::sort(scan_pose_filename_pairs.second.begin(), scan_pose_filename_pairs.second.end());

    Path *path = new Path();

    for(auto pose_path : scan_pose_filename_pairs.second)
    {
        auto pose_mat = CoordSysTransform::getTransformationFromPose(pose_path);
        Pose pose(pose_mat);
        path->add_pose(pose);
    }

    auto path_marker = ROSViewhelper::initPathMarker(path);

    ros::Rate r(1.0f);

    int counter = 0;

    while (ros::ok())
    {
        r.sleep();

        if (counter >= num_scans)
        {
            continue;
        }

        auto scan_path = scan_pose_filename_pairs.first[counter];
        auto pose_path = scan_pose_filename_pairs.second[counter];

        auto transformed_cloud = CoordSysTransform::read_scan_file_and_transform(scan_path);
        auto cloud = CoordSysTransform::read_scan_file(scan_path);

        auto pose_mat = CoordSysTransform::getTransformationFromPose(pose_path);

        std::cout << "Pose transform for Pose " << pose_path.string() << ": " << std::endl
                  << pose_mat << std::endl;

        auto trans_pose_marker = ROSViewhelper::initPoseMarker(new Pose(pose_mat));

        sensor_msgs::PointCloud2 ros_cloud = pcl_to_ros(cloud);
        sensor_msgs::PointCloud2 transformed_ros_cloud = pcl_to_ros(transformed_cloud);

        cloud_pub.publish(ros_cloud);
        trans_cloud_pub.publish(transformed_ros_cloud);

        //trans_pose_pub.publish(trans_pose_marker);
        trans_pose_pub.publish(path_marker);

        counter++;
    }
}