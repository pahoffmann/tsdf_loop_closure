#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include <loop_closure/util/point.h>
#include <loop_closure/params/loop_closure_params.h>

#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class GTSAMWrapper
{
private:
    // gtsam graph
    std::unique_ptr<gtsam::NonlinearFactorGraph> graph;
    LoopClosureParams params;

    // declare noises for the prior, in between and loop_closure constraints
    // SIEHE:
    // https://gtsam.org/tutorials/intro.html#magicparlabel-65728
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise;
    gtsam::noiseModel::Diagonal::shared_ptr in_between_noise;
    gtsam::noiseModel::Diagonal::shared_ptr loop_closure_noise;

    std::string print_prefix = "[GTSAMWrapper] "; 

    /**
     * @brief performs basic ICP between source and target, will return information about the performance of the icp
     *
     * @param source_cloud
     * @param target_cloud
     * @param converged
     * @param final_transformation
     * @param fitness_score
     */
    void perform_pcl_icp(pcl::PointCloud<PointType>::Ptr source_cloud, pcl::PointCloud<PointType>::Ptr target_cloud,
                         pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score);

    /**
     * @brief performs generalized icp between the two pointclouds, will return information about the performance of the icp
     *
     * @param source_cloud
     * @param target_cloud
     * @param converged
     * @param final_transformation
     * @param fitness_score
     */
    void perform_pcl_gicp(pcl::PointCloud<PointType>::Ptr source_cloud, pcl::PointCloud<PointType>::Ptr target_cloud,
                          pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score);

public:

    /**
     * @brief Construct a new GTSAMWrapper, will intialize the noises(TODO: from parameters)
     *
     */
    GTSAMWrapper(LoopClosureParams &input_params);
    ~GTSAMWrapper() = default;

    /**
     * @brief adds a prior constraint for the initial pose of the graph
     *
     */
    void add_prior_constraint(Matrix4f &transform);

    /**
     * @brief adds a constraint for the edges of the factor graph, meaning which transform is present between to pose indices
     *        according to odometry, imu, other estimations
     *
     * @param transform
     * @param from_idx
     * @param to_idx
     */
    void add_in_between_contraint(Eigen::Matrix4f transform, int from_idx, int to_idx);

    /**
     * @brief adds a loop closure constraint to the factor graph, if icp / gicp converges
     *
     * @return if the loop closure constraint was added to the graph
     */
    bool add_loop_closure_constraint(std::pair<int, int> lc_indices, pcl::PointCloud<PointType>::Ptr current_cloud, pcl::PointCloud<PointType>::Ptr previous_cloud,
                                    pcl::PointCloud<PointType>::Ptr icp_cloud);

    /**
     * @brief will optimize the gtsam factor graph
     * 
     */
    gtsam::Values perform_optimization(gtsam::Values initial);

    /**
     * @brief will reset the graph
     * 
     */
    void reset();
};
