#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include <loop_closure/util/point.h>
#include <loop_closure/path/path.h>
#include <loop_closure/params/loop_closure_params.h>
#include <loop_closure/util/vgicp.h>

// rejectors
#include <loop_closure/loop_closure/lc_line_rejector.h>
#include <loop_closure/loop_closure/lc_range_rejector.h>

#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/features/fpfh.h>

// teaser++
#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <teaser/matcher.h>

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
     * @brief will use the teaser++ library and FPFH Features (+ Normal estimation) to register two pcl's
     *
     * @param model_cloud
     * @param scan_cloud
     * @param result
     * @param converged
     * @param final_transformation
     * @param fitness_score
     */
    void perform_teaser_plus_plus(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
                                  pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score);

    /**
     * @brief will use the teaser++ library and FPFH Features (+ Normal estimation) to register two pcl's
     *
     * @param model_cloud
     * @param scan_cloud
     * @param result
     * @param converged
     * @param final_transformation
     * @param fitness_score
     */
    void perform_own_teaser_plus_plus(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
                                      pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score);

    /**
     * @brief will calculate the Mean Squared Distance between the two input pointclouds
     *
     * @param model_cloud
     * @param scan_cloud
     * @return float
     */
    float calculate_fitness_score(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud);

    /**
     * @brief will preprocess the incoming scans used to check for a closed loop, might also pretransform the cur cloud in order to achieve better results
     *
     * @param source_cloud
     * @param target_cloud
     * @param pretransform_mat will contain the final pretransform matrix
     */
    void preprocess_scans(pcl::PointCloud<PointType>::Ptr cur_cloud, pcl::PointCloud<PointType>::Ptr prev_cloud, Eigen::Matrix4f &pretransform_mat);

    /**
     * @brief will calculate the normals for 'cloud' and add them to a new cloud 'cloud_with_normals'
     *
     * @param cloud
     * @param cloud_with_normals
     */
    void addNormal(pcl::PointCloud<PointType>::Ptr cloud,
                   pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals);

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
    void add_in_between_contraint(Eigen::Matrix4f transform, int from_idx, int to_idx, float input_fitness_noise = -1.0f);

    /**
     * @brief
     *
     * @return
     */

    /**
     * @brief adds a loop closure constraint to the factor graph, if icp / gicp converges
     *
     * @param lc_indices
     * @param model_cloud
     * @param scan_cloud
     * @param icp_cloud
     * @param final_transformation
     * @param prev_to_cur_initial
     * @return true if the loop closure constraint was added to the graph
     * @return false if not
     */
    bool add_loop_closure_constraint(std::pair<int, int> lc_indices, pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
                                     pcl::PointCloud<PointType>::Ptr icp_cloud, float &fitness_score_ref, Matrix4f &final_transformation, Matrix4f prev_to_cur_initial, Path *path);

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

    /**
     * @brief performs basic ICP between source and target, will return information about the performance of the icp
     *
     * @param source_cloud
     * @param target_cloud
     * @param converged
     * @param final_transformation
     * @param fitness_score
     */
    void perform_pcl_icp(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
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
    void perform_pcl_gicp(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
                          pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score);

    /**
     * @brief performs adaptive generalized icp between the two pointclouds, will return information about the performance of the gicp
     *
     * @param source_cloud
     * @param target_cloud
     * @param converged
     * @param final_transformation
     * @param fitness_score
     */
    void perform_adaptive_pcl_gicp(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
                          pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score);

    /**
     * @brief performs generalized icp between the two pointclouds, will return information about the performance of the gicp
     *
     * @param source_cloud
     * @param target_cloud
     * @param converged
     * @param final_transformation
     * @param fitness_score
     */
    void perform_vgicp(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
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
    void perform_pcl_normal_icp(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
                                pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score);
};
