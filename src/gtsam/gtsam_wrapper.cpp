#include <loop_closure/gtsam/gtsam_wrapper.h>

GTSAMWrapper::GTSAMWrapper(LoopClosureParams &input_params)
{
    // reset unique pointer
    graph.reset(new gtsam::NonlinearFactorGraph());
    params = input_params;

    // create noises

    // SIEHE:
    // https://gtsam.org/tutorials/intro.html#magicparlabel-65728
    // 20cm noise in x, y und z Richtung, 0.1 radiants fehler in z richtung
    // gtsam::Vector6 noise_vec;
    // noise_vec << 0.5, 0.5, 0.5, 0.3, 0.3, 0.3;

    // initialize prior and in between noise with default values used for noising the constraints
    //prior_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
    prior_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 0.2, 0.2, 0.2).finished()); // rad*rad, meter*meter
    //in_between_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished());
    in_between_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 0.3, 0.3, 0.3).finished());

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
                                               pcl::PointCloud<PointType>::Ptr icp_cloud, Matrix4f cur_cloud_transform, Matrix4f prev_cloud_transform,
                                               Matrix4f &final_transformation)
{
    // variables for icp/gicp
    float fitness_score;
    bool converged;

    // proprocessing
    Matrix4f pretransform_mat = Matrix4f::Identity();

    // calculate rotation difference between cur and prev transforms
    // pretransform_mat.block<3, 3>(0, 0) = getTransformationMatrixDiff(cur_cloud_transform, prev_cloud_transform).block<3, 3>(0, 0);

    // pretransform mat will contain the final pretransform, needs to be combined with
    //preprocess_scans(current_cloud, previous_cloud, pretransform_mat);

    // different scan matching possibilities
    //perform_pcl_icp(current_cloud, previous_cloud, icp_cloud, converged, final_transformation, fitness_score);
    perform_pcl_gicp(current_cloud, previous_cloud, icp_cloud, converged, final_transformation, fitness_score);

    final_transformation = final_transformation * pretransform_mat;

    if (converged)
    {
        std::cout << print_prefix << "Scan matching converged for lc with indices: " << lc_indices.first << " | " << lc_indices.second << std::endl;
        std::cout << print_prefix << "ICP Fitness Score: " << fitness_score << std::endl;
        std::cout << print_prefix << "Final tranformation: " << std::endl
                  << final_transformation << std::endl;

        // check if the fitness score is above a certain upper boundary, if so scan matching was not good enough
        if (fitness_score > 0.5)
        {
            std::cout << print_prefix << "Though scan matching converged, ICP doesn't seem to have found an appropriate transformation!" << std::endl;
            return false;
        }
    }
    else
    {
        std::cout << print_prefix << "ICP did not converge" << std::endl;
        return false;
    }

    // create loop closure noise from scan matching fitness score
    gtsam::Vector Vector6(6);
    float noiseScore = fitness_score;
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = final_transformation;

    // LIOSAM
    // transform from world origin to wrong pose
    // Eigen::Affine3f tWrong(path->at(loop_key_cur)->getTransformationMatrix());

    // // transform from world origin to corrected pose
    // Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
    // pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

    // gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    // gtsam::Pose3 poseTo(gtsam::Rot3(path->at(lc_indices.first)->quat.cast<double>()),
    //                     gtsam::Point3(path->at(lc_indices.first)->pos.cast<double>()));

    // auto between_fac = gtsam::BetweenFactor<gtsam::Pose3>(lc_indices.second, lc_indices.first, poseFrom.between(poseTo), constraintNoise);

    // ME
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
    icp.align(*result);

    // fill data with icp results
    fitness_score = icp.getFitnessScore();
    final_transformation = icp.getFinalTransformation();
    converged = icp.hasConverged();
}

void GTSAMWrapper::perform_pcl_gicp(pcl::PointCloud<PointType>::Ptr source_cloud, pcl::PointCloud<PointType>::Ptr target_cloud,
                                    pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score)
{
    // align the clouds using generalized icp
    static pcl::GeneralizedIterativeClosestPoint<PointType, PointType> g_icp;
    g_icp.setMaximumIterations(params.loop_closure.max_icp_iterations);
    g_icp.setMaximumOptimizerIterations(100);
    g_icp.setTransformationEpsilon(0.01);
    g_icp.setMaxCorrespondenceDistance(10.0);
    g_icp.setRANSACIterations(100);
    g_icp.setRANSACOutlierRejectionThreshold(1.0);
    g_icp.setUseReciprocalCorrespondences(false);

    g_icp.setInputSource(source_cloud);
    g_icp.setInputTarget(target_cloud);
    g_icp.align(*result);

    // fill data with g_icp results
    fitness_score = g_icp.getFitnessScore();
    final_transformation = g_icp.getFinalTransformation();
    converged = g_icp.hasConverged();
}

void GTSAMWrapper::preprocess_scans(pcl::PointCloud<PointType>::Ptr cur_cloud, pcl::PointCloud<PointType>::Ptr prev_cloud, Eigen::Matrix4f &pretransform_mat)
{
    // as the estimated clouds will be very far from each other most of the time, we will
    // pretransform the current pcl

    // CENTROID COMPUTATION AS PRETRANSFORM

    Eigen::Vector4d centroid_prev, centroid_cur;
    pcl::compute3DCentroid(*cur_cloud.get(), centroid_cur);
    pcl::compute3DCentroid(*prev_cloud.get(), centroid_prev);

    // calculate centoid diff from cur to prev
    Eigen::Vector3f centroid_diff(centroid_prev.x() - centroid_cur.x(), centroid_prev.y() - centroid_cur.y(), centroid_prev.z() - centroid_cur.z());

    std::cout << "Computed centroid diff: " << std::endl
              << centroid_diff << std::endl;

    // STATISTICAL OUTLIER FILTERING in the respective clouds
    std::cout << "Before filter size: " << cur_cloud->size() << std::endl;

    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud(cur_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cur_cloud);

    std::cout << "After filter size: " << cur_cloud->size() << std::endl;

    std::cout << "Before filter size: " << prev_cloud->size() << std::endl;

    sor.setInputCloud(prev_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*prev_cloud);

    std::cout << "After filter size: " << prev_cloud->size() << std::endl;
    // END OUTLIER FILTERING

    // centroid diff is new translation pretransform
    pretransform_mat.block<3, 1>(0, 3) = centroid_diff;

    // pretransform cur
    pcl::transformPointCloud(*cur_cloud, *cur_cloud, pretransform_mat);
}
