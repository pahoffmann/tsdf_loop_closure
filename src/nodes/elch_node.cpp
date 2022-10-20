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
#include <loop_closure/map/map_updater.h>

// transform between ros and eigen
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/elch.h>

#include <iostream>
#include <string>

#include <vector>

using PointType = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointType>;
using CloudConstPtr = Cloud::ConstPtr;
using CloudPtr = Cloud::Ptr;
using CloudPair = std::pair<Matrix4f, CloudPtr>;
using CloudVector = std::vector<CloudPair>;

LoopClosureParams params;

std::shared_ptr<LocalMap> local_map_ptr;
std::shared_ptr<GlobalMap> global_map_ptr;

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

// Boncpp
std::unique_ptr<bond::Bond> bond_ptr;

bool loopDetection(int end, const CloudVector &clouds, double dist, int &first, int &last)
{
    static double min_dist = -1;
    int state = 0;

    for (int i = end - 1; i > 0; i--)
    {
        Eigen::Vector4f cstart, cend;
        // TODO use pose of scan
        pcl::compute3DCentroid(*(clouds[i].second), cstart);
        pcl::compute3DCentroid(*(clouds[end].second), cend);
        Eigen::Vector4f diff = cend - cstart;

        double norm = diff.norm();

        // std::cout << "distance between " << i << " and " << end << " is " << norm << " state is " << state << std::endl;

        if (state == 0 && norm > dist)
        {
            state = 1;
            // std::cout << "state 1" << std::endl;
        }
        if (state > 0 && norm < dist)
        {
            state = 2;
            // std::cout << "loop detected between scan " << i << " (" << clouds[i].first << ") and scan " << end << " (" << clouds[end].first << ")" << std::endl;
            if (min_dist < 0 || norm < min_dist)
            {
                min_dist = norm;
                first = i;
                last = end;
            }
        }
    }
    // std::cout << "min_dist: " << min_dist << " state: " << state << " first: " << first << " end: " << end << std::endl;
    if (min_dist > 0 && (state < 2 || end == int(clouds.size()) - 1)) // TODO
    {
        min_dist = -1;
        return true;
    }
    return false;
}

void handle_data_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr, const geometry_msgs::PoseStampedConstPtr &pose_ptr)
{
    double dist = 0.1;

    double rans = 0.1;

    int iter = 100;

    pcl::registration::ELCH<PointType> elch;
    pcl::IterativeClosestPoint<PointType, PointType>::Ptr icp(new pcl::IterativeClosestPoint<PointType, PointType>);
    icp->setMaximumIterations(iter);
    icp->setMaxCorrespondenceDistance(dist);
    icp->setRANSACOutlierRejectionThreshold(rans);
    elch.setReg(icp);

    std::vector<int> pcd_indices;
    pcd_indices = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

    CloudVector clouds;
    for (std::size_t i = 0; i < pcd_indices.size(); i++)
    {
        CloudPtr pc(new Cloud);
        pcl::io::loadPCDFile(argv[pcd_indices[i]], *pc);
        clouds.push_back(CloudPair(argv[pcd_indices[i]], pc));
        std::cout << "loading file: " << argv[pcd_indices[i]] << " size: " << pc->size() << std::endl;
        elch.addPointCloud(clouds[i].second);
    }

    int first = 0, last = 0;

    for (std::size_t i = 0; i < clouds.size(); i++)
    {

        if (loopDetection(int(i), clouds, 3.0, first, last))
        {
            std::cout << "Loop between " << first << " (" << clouds[first].first << ") and " << last << " (" << clouds[last].first << ")" << std::endl;
            elch.setLoopStart(first);
            elch.setLoopEnd(last);
            elch.compute();
        }
    }

    for (const auto &cloud : clouds)
    {
        std::string result_filename(cloud.first);
        result_filename = result_filename.substr(result_filename.rfind('/') + 1);
        pcl::io::savePCDFileBinary(result_filename, *(cloud.second));
        std::cout << "saving result to " << result_filename << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam6d_listener");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // specify ros loop rate
    ros::Rate loop_rate(1.0f);

    // load params from nodehandle
    params = LoopClosureParams(nh);
    // init_obj();

    message_filters::Subscriber<sensor_msgs::PointCloud2> slam6d_cloud_sub(n, "/slam6d_cloud", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> slam6d_pose_sub(n, "/slam6d_pose", 1);
    message_filters::Subscriber<std_msgs::String> slam6d_filename_sub(n, "/slam6d_filename", 1);

    typedef sync_policies::ExactTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(40), slam6d_cloud_sub, slam6d_pose_sub);
    sync.registerCallback(boost::bind(&handle_data_callback, _1, _2));

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

    return 0;
}