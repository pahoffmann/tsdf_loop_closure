#pragma once

#include <omp.h>
#include <chrono>

#include <loop_closure/util/point.h>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>

#ifdef USE_CUDA
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

namespace VGICP
{

  static std::string vgicp_description()
  {
#ifdef USE_CUDA
    return "(gpu)";
#else
    return "(cpu)";
#endif
  }

  static inline Eigen::Matrix4f apply_registration(pcl::PointCloud<PointType>::Ptr &source, pcl::PointCloud<PointType>::Ptr &target, float &fitness_score_ref, bool &converged)
  {

#ifdef USE_CUDA
    // assumption: weak cpu, strong gpu -> bruteforce covariance estimation
    // TODO compare with rbf_kernel and cpu-based parallel kdtree
    fast_gicp::FastVGICPCuda<PointType, PointType> vgicp;
    vgicp.setResolution(0.5);
    vgicp.setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::CPU_PARALLEL_KDTREE);
    // kernel width (and distance threshold) need to be tuned
    vgicp.setKernelWidth(0.5);
#else
    fast_gicp::FastVGICP<PointType, PointType> vgicp;
    vgicp.setResolution(0.5);
    vgicp.setNumThreads(omp_get_max_threads());
#endif
    double fitness_score = 0.0;
    static int n_calls = 0;
    static float vgicp_sum = 0.0f;

    // single run
    auto t1 = std::chrono::high_resolution_clock::now();
    // fast_gicp reuses calculated covariances if an input cloud is the same as the previous one
    // to prevent this for benchmarking, force clear source and target clouds
    vgicp.clearTarget();
    vgicp.clearSource();
    vgicp.setInputTarget(target);
    vgicp.setInputSource(source);
    vgicp.align(*source);
    auto t2 = std::chrono::high_resolution_clock::now();
    fitness_score = vgicp.getFitnessScore();
    converged = vgicp.hasConverged();
    fitness_score_ref = fitness_score;
    double single = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;

    n_calls += 1;
    vgicp_sum += single;
    // ROS_INFO_STREAM("vgicp " << vgicp_description() << ": " << single << "[msec] | health: " << std::setw(3) << fitness_score << " | avg: " << vgicp_sum / n_calls << "[msec]");

    if (fitness_score > 3)
    {
      // ROS_WARN_STREAM("vgicp " << vgicp_description() << " fitness score " << fitness_score << " is shit");
      return Eigen::Matrix4f::Identity();
    }

    return vgicp.getFinalTransformation();
  }

  static inline Eigen::Matrix4f gicp_transform(pcl::PointCloud<PointType>::Ptr &source, pcl::PointCloud<PointType>::Ptr &target, float &fitness_score_ref, bool &converged)
  {
    pcl::ApproximateVoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);
    voxelgrid.setInputCloud(target);
    voxelgrid.filter(*target);
    voxelgrid.setInputCloud(source);
    voxelgrid.filter(*source);
    return apply_registration(source, target, fitness_score_ref, converged);
  }

}