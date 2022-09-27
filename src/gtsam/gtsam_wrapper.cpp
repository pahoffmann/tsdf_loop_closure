#include <loop_closure/gtsam/gtsam_wrapper.h>

GTSAMWrapper::GTSAMWrapper(LoopClosureParams &input_params)
{
    // reset unique pointer
    graph.reset(new gtsam::NonlinearFactorGraph());
    params = input_params;

    // create noises
    // initialize prior and in between noise with default values used for noising the constraints
    prior_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
    in_between_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished());
}

void GTSAMWrapper::add_prior_constraint(Matrix4f &transform)
{
    // transform eigen matrix4f to gtsam pose3
    gtsam::Rot3 rot3_prior(transform.block<3, 3>(0, 0).cast<double>());
    Eigen::Vector3d translation = transform.block<3, 1>(0, 3).cast<double>();
    gtsam::Point3 point3_prior(translation.x(), translation.y(), translation.z());

    // create prior factor constraint from input transform (pose)
    gtsam::PriorFactor<gtsam::Pose3> factor(0, gtsam::Pose3(rot3_prior, point3_prior), prior_noise);

    // add constraint to graph
    graph->add(factor);
}

void GTSAMWrapper::add_in_between_contraint(Eigen::Matrix4f transform, int from_idx, int to_idx)
{
    // transform eigen matrix4f to gtsam pose3
    gtsam::Rot3 rot3(transform.block<3, 3>(0, 0).cast<double>());
    Eigen::Vector3d pos_diff = transform.block<3, 1>(0, 3).cast<double>();
    gtsam::Point3 point3(pos_diff.x(), pos_diff.y(), pos_diff.z());

    // create between factor constraint for input transform
    gtsam::BetweenFactor<gtsam::Pose3> factor(from_idx, to_idx, gtsam::Pose3(rot3, point3), in_between_noise);

    // add constraint to graph
    graph->add(factor);
}

bool GTSAMWrapper::add_loop_closure_constraint(std::pair<int, int> lc_indices, pcl::PointCloud<PointType>::Ptr current_cloud, pcl::PointCloud<PointType>::Ptr previous_cloud,
                                               pcl::PointCloud<PointType>::Ptr icp_cloud)
{
    // variables for icp/gicp
    Matrix4f final_transformation;
    float fitness_score;
    bool converged;

    // different scan matching possibilities
    //perform_pcl_icp(current_cloud, previous_cloud, icp_cloud, converged, final_transformation, fitness_score);
    perform_pcl_gicp(current_cloud, previous_cloud, icp_cloud, converged, final_transformation, fitness_score);

    if (converged)
    {
        std::cout << print_prefix << "Scan matching converged for lc with indices: " << lc_indices.first << " | " << lc_indices.second << std::endl;
        std::cout << print_prefix << "ICP Fitness Score: " << fitness_score << std::endl;
        std::cout << print_prefix << "Final tranformation: " << std::endl
                  << final_transformation << std::endl;

        // check if the fitness score is above a certain upper boundary, if so scan matching was not good enough
        if (fitness_score > 0.3)
        {
            std::cout << print_prefix << "Though scan matching converged, ICP doesn't seem to have found an appropriate transformation!" << std::endl;
            return false;
        }
    }

    // create loop closure noise from scan matching fitness score
    gtsam::Vector Vector6(6);
    float noiseScore = fitness_score;
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = final_transformation;

    // transform to gtsam pose3
    Eigen::Affine3f tCorrect = correctionLidarFrame; // * tWrong; // pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 between_trans = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));

    // create a between factor based on the findings
    // the constraint is always added from the pose with greater index to the one with a lower index
    auto between_fac = gtsam::BetweenFactor<gtsam::Pose3>(lc_indices.second, lc_indices.first, between_trans, constraintNoise);

    // add the defined constraint to the graph
    graph->add(between_fac);

    // this will always be 'true' here
    return converged;
}

gtsam::Values GTSAMWrapper::perform_optimization(gtsam::Values initial)
{
    std::cout << print_prefix << "START FACTORGRAPH OPTIMIZATION!" << std::endl;

    double error_prev = graph->error(initial);

    std::cout << print_prefix << "Error (previous): " << error_prev << std::endl;

    // optimize intial values using levenberg marquardt
    gtsam::LevenbergMarquardtOptimizer optimizer(*graph, initial);
    auto values = optimizer.optimize();

    // gtsam::GaussNewtonOptimizer gauss_optimizer(*graph, initial);
    // auto values_gauss = gauss_optimizer.optimize();

    std::cout << print_prefix << "Error (after): " << graph->error(values) << std::endl;

    return values;
}

void GTSAMWrapper::reset()
{
    graph.reset(new gtsam::NonlinearFactorGraph());
}

void GTSAMWrapper::perform_pcl_icp(pcl::PointCloud<PointType>::Ptr source_cloud, pcl::PointCloud<PointType>::Ptr target_cloud,
                                   pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score)
{

    // ICP Settings
    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    // icp.setMaxCorrespondenceDistance(0.2f); // hardcoded for now
    icp.setMaximumIterations(params.loop_closure.max_icp_iterations);
    // icp.setTransformationEpsilon(1e-6);
    // icp.setEuclideanFitnessEpsilon(1e-6);
    // icp.setRANSACIterations(0);

    // Align clouds
    // icp.setInputSource(pointcloud_cur_pretransformed);
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    pcl::PointCloud<PointType>::Ptr icp_result(new pcl::PointCloud<PointType>());
    icp.align(*icp_result);

    // fill data with icp results
    fitness_score = icp.getFitnessScore();
    final_transformation = icp.getFinalTransformation();
    converged = icp.hasConverged();
    result = icp_result;
}

void GTSAMWrapper::perform_pcl_gicp(pcl::PointCloud<PointType>::Ptr source_cloud, pcl::PointCloud<PointType>::Ptr target_cloud,
                                    pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score)
{
    // align the clouds using generalized icp
    static pcl::GeneralizedIterativeClosestPoint<PointType, PointType> g_icp;
    g_icp.setMaximumIterations(params.loop_closure.max_icp_iterations);

    g_icp.setInputSource(source_cloud);
    g_icp.setInputTarget(target_cloud);
    pcl::PointCloud<PointType>::Ptr g_icp_result(new pcl::PointCloud<PointType>());
    g_icp.align(*g_icp_result);

    // fill data with g_icp results
    fitness_score = g_icp.getFitnessScore();
    final_transformation = g_icp.getFinalTransformation();
    converged = g_icp.hasConverged();
    result = g_icp_result;
}
