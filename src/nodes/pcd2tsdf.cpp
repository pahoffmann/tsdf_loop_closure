// ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

// loop closure
#include <loop_closure/util/runtime_evaluator.h>
#include <loop_closure/util/update_tsdf.h>
#include <loop_closure/map/local_map.h>
#include <loop_closure/visualization/ros_viewhelper.h>
#include <loop_closure/util/point.h>

namespace fs = boost::filesystem;

// global variables

// Configuration stuff //
LoopClosureParams params;

Eigen::Matrix4f last_updated_pose_ = Eigen::Matrix4f::Identity();

/// Map Stuff ///
std::shared_ptr<GlobalMap> global_map_ptr_;
std::shared_ptr<LocalMap> local_map_ptr_;

/**
 * @brief wrapper function to shift the localmap and update it using the tsdf update method
 *
 * @param pose the new pose
 * @param points the belonging datapoints
 * @param params
 * @param local_map_ptr
 */
void shift_update_visualize(const Eigen::Matrix4f &pose, const std::vector<Eigen::Vector3i> &points, std::shared_ptr<LocalMap> local_map_ptr)
{
  // Create a local copy of the ring buffer / local map
  auto suv_buffer_ptr = std::make_shared<LocalMap>(*local_map_ptr);

  // Shift
  Eigen::Vector3i pos = real_to_map(pose.block<3, 1>(0, 3));
  suv_buffer_ptr->shift(pos);

  Eigen::Matrix4i rotation_mat = Eigen::Matrix4i::Identity();
  rotation_mat.block<3, 3>(0, 0) = to_int_mat(pose).block<3, 3>(0, 0);
  Eigen::Vector3i up = transform_point(Eigen::Vector3i(0, 0, MATRIX_RESOLUTION), rotation_mat);

  update_tsdf(points, pos, up, *suv_buffer_ptr, params.map.tau, params.map.max_weight, params.map.resolution);

  // Update the pointer (global member for registration) to the locally updated ring buffer / local map
  local_map_ptr = suv_buffer_ptr; // TODO which constructor is used here
  last_updated_pose_ = pose;

  // Visualize
  // std_msgs::Header header;
  // header.frame_id = "map";
  // header.stamp = ros::Time::now();
  // visualize_local_map(header, tsdf_map_pub_, *suv_buffer_ptr, params.get_tau(), params.get_map_resolution());
  // ROS_INFO_STREAM("tsdf updated" << (params.map.refinement ? " w/ refinement" : " w/o refinement"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcd2tsdf");
  ros::NodeHandle nh("~");

  // load params from nodehandle
  params = LoopClosureParams(nh);
  params.load(nh);
  auto tsdf_pub = nh.advertise<visualization_msgs::Marker>("/tsdf_pcd", 0);
  auto pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_pcd", 0);

  // setup point data
  pcl::PointCloud<PointType>::Ptr orig(new pcl::PointCloud<PointType>);
  //fs::path filename = fs::path(DATA_PATH) / "frame_1000.pcd";
  fs::path filename = fs::path(DATA_PATH) / "test.pcd";

  if (pcl::io::loadPCDFile<PointType>(filename.string(), *orig) == -1)
  {
    PCL_ERROR("Couldn't read test pcd\n");
    return (-1);
  }

  // filter input cloud: one
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
  pcl::VoxelGrid<PointType> sor;
  sor.setInputCloud(orig);
  sor.setLeafSize(params.map.resolution / 1000.0f, params.map.resolution / 1000.0f, params.map.resolution / 1000.0f);
  sor.filter(*cloud);

  // create ros pcl message
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloud.get(), ros_cloud);
  ros_cloud.header.frame_id = "map";
  ros_cloud.header.stamp = ros::Time::now();

  std::vector<Eigen::Vector3i> points_original(cloud->size());

#pragma omp parallel for schedule(static) default(shared)
  for (int i = 0; i < cloud->size(); ++i)
  {
    const auto &cp = (*cloud)[i];
    points_original[i] = Eigen::Vector3i(cp.x * 1000.f, cp.y * 1000.f, cp.z * 1000.f);
  }

  // setup local map
  std::shared_ptr<GlobalMap> global_map_ptr_;
  std::shared_ptr<LocalMap> local_map_ptr_;
  global_map_ptr_.reset(new GlobalMap(params.map));
  local_map_ptr_.reset(new LocalMap(params.map.size.x(), params.map.size.y(), params.map.size.z(), global_map_ptr_));

  // Shift
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  Vector3i pos = real_to_map(pose.block<3, 1>(0, 3));
  local_map_ptr_->shift(pos);

  Eigen::Matrix4i rot = Eigen::Matrix4i::Identity();
  rot.block<3, 3>(0, 0) = to_int_mat(pose).block<3, 3>(0, 0);
  Eigen::Vector3i up = transform_point(Eigen::Vector3i(0, 0, MATRIX_RESOLUTION), rot);

  // create TSDF Volume
  update_tsdf(points_original, pos, up, *local_map_ptr_, params.map.tau, params.map.max_weight, params.map.resolution);

  ros::Rate r(1.0);

  while (ros::ok())
  {
    r.sleep();
    // visualize_local_map(header, pub, *local_map_ptr_, params.map.tau, params.map.resolution);

    auto marker = ROSViewhelper::initTSDFmarkerPose(local_map_ptr_, new Pose(pose));

    tsdf_pub.publish(marker);
    pcl_pub.publish(ros_cloud);
  }
}
