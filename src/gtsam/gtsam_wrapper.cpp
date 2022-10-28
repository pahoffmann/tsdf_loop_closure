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
    // prior_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
    // in_between_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 0.3, 0.3, 0.3).finished());
    // in_between_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 0.2, 0.2, 0.2, 1.0, 1.0, 1.0).finished());

    prior_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << std::pow(params.loop_closure.prior_rotation_noise_x, 2),
         std::pow(params.loop_closure.prior_rotation_noise_y, 2),
         std::pow(params.loop_closure.prior_rotation_noise_z, 2),
         std::pow(params.loop_closure.prior_translation_noise_x, 2),
         std::pow(params.loop_closure.prior_translation_noise_y, 2),
         std::pow(params.loop_closure.prior_translation_noise_z, 2))
            .finished()); // rad*rad, meter*meter

    in_between_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << std::pow(params.loop_closure.between_rotation_noise_x, 2),
         std::pow(params.loop_closure.between_rotation_noise_y, 2),
         std::pow(params.loop_closure.between_rotation_noise_z, 2),
         std::pow(params.loop_closure.between_translation_noise_x, 2),
         std::pow(params.loop_closure.between_translation_noise_y, 2),
         std::pow(params.loop_closure.between_translation_noise_z, 2))
            .finished()); // rad*rad, meter*meter
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

void GTSAMWrapper::add_in_between_contraint(Eigen::Matrix4f transform, int from_idx, int to_idx, float input_fitness_noise)
{
    // transform eigen matrix4f to gtsam pose3
    gtsam::Rot3 rot3(transform.block<3, 3>(0, 0).cast<double>());
    Eigen::Vector3d pos_diff = transform.block<3, 1>(0, 3).cast<double>();
    gtsam::Point3 point3(pos_diff.x(), pos_diff.y(), pos_diff.z());

    // create between factor constraint for input transform
    if (input_fitness_noise != -1.0f)
    {
        std::cout << print_prefix << " Input fitness score: " << input_fitness_noise << std::endl;
        // use input noise instead of default values
        gtsam::noiseModel::Diagonal::shared_ptr in_between_noise_tmp = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << input_fitness_noise, input_fitness_noise, input_fitness_noise,
                                                                                                               input_fitness_noise, input_fitness_noise, input_fitness_noise)
                                                                                                                  .finished());
        gtsam::BetweenFactor<gtsam::Pose3> factor(from_idx, to_idx, gtsam::Pose3(rot3, point3), in_between_noise_tmp);
        graph->add(factor); // add constraint to graph
    }
    else
    {
        gtsam::BetweenFactor<gtsam::Pose3> factor(from_idx, to_idx, gtsam::Pose3(rot3, point3), in_between_noise);
        graph->add(factor); // add constraint to graph
    }
}

bool GTSAMWrapper::add_loop_closure_constraint(std::pair<int, int> lc_indices, pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
                                               pcl::PointCloud<PointType>::Ptr icp_cloud, float &fitness_score_ref, Matrix4f &final_transformation, Matrix4f prev_to_cur_initial, Path *path)
{
    // variables for icp/gicp

    // for the respective algorithms
    float fitness_score_gicp;
    bool converged_gicp;
    Matrix4f final_transform_gicp;
    pcl::PointCloud<PointType>::Ptr gicp_cloud(new pcl::PointCloud<PointType>());

    float fitness_score_icp;
    bool converged_icp;
    Matrix4f final_transform_icp;

    // total
    float fitness_score;
    bool converged;
    Eigen::Matrix4f teaser_transform;

    // different scan matching possibilities

    // pretransform using teaser ++
    // perform_teaser_plus_plus(model_cloud, scan_cloud, icp_cloud, converged, teaser_transform, fitness_score);
    // perform_own_teaser_plus_plus(model_cloud, scan_cloud, icp_cloud, converged, final_transformation, fitness_score);

    // transform model cloud towards proposed pose from teaser++
    // Eigen::Matrix4f teaser_inversed = teaser_transform.inverse();
    // std::cout << "Inverse transform of T++: " << std::endl << Pose(teaser_inversed) << std::endl;
    // pcl::transformPointCloud(*model_cloud, *model_cloud, teaser_transform.inverse());

    // perform_pcl_icp(model_cloud, scan_cloud, icp_cloud, converged, final_transformation, fitness_score);
    // perform_pcl_gicp(model_cloud, scan_cloud, gicp_cloud, converged_gicp, final_transform_gicp, fitness_score_gicp);
    perform_adaptive_pcl_gicp(model_cloud, scan_cloud, gicp_cloud, converged_gicp, final_transform_gicp, fitness_score_gicp);
    // perform_vgicp(model_cloud, scan_cloud, icp_cloud, converged, final_transformation, fitness_score);

    final_transformation = final_transform_gicp;
    fitness_score = fitness_score_gicp;
    converged = converged_gicp;

    // if gicp is good, but not good enough, check if icp does a better job instead
    // if (converged_gicp && fitness_score_gicp > params.loop_closure.max_lc_icp_fitness)
    // {
    // perform_pcl_icp(model_cloud, scan_cloud, icp_cloud, converged_icp, final_transform_icp, fitness_score_icp);

    // if icp is better, update this stuff
    // if (fitness_score_icp < fitness_score_gicp)
    // {
    //     std::cout << "ICP IS BETTER: " << fitness_score_icp << std::endl;
    //     fitness_score = fitness_score_icp;
    //     final_transformation = final_transform_icp;
    //     converged = converged_icp;
    // }
    // else
    // {
    pcl::copyPointCloud(*gicp_cloud, *icp_cloud);
    // }
    // }
    // ignore z translation and x y achsis rotation, as this is faulty for this dataset
    Pose final_transformation_pose(final_transformation);
    // final_transformation_pose.pos.z() = 0;
    // final_transformation_pose.quat.x() = 0;
    // final_transformation_pose.quat.y() = 0;

    final_transformation = final_transformation_pose.getTransformationMatrix();

    // return fitness score
    fitness_score_ref = fitness_score;

    // std::cout << "Teaser++ transformation:: (readable)" << std::endl
    //           << Pose(teaser_transform) << std::endl;

    // std::cout << "ICP transformation:: (readable)" << std::endl
    //           << Pose(final_transformation) << std::endl;

    // std::cout << "Prev to cur transformation:: (readable)" << std::endl
    //           << Pose(prev_to_cur_initial) << std::endl;

    // this is basically the most important point. we just calculated the transformation
    // P_cur' -> P_cur (as icp is executed in the P_cur coordinate system)
    // to get the transformation P_prev -> P_cur', we need to apply P_prev -> P_cur and P_cur -> P_cur'
    // (which is the inverse of the calculated transform)
    // keep in mind the order of the transformations: execution order is right to left
    final_transformation = final_transformation.inverse() /* * teaser_transform.inverse() */ * prev_to_cur_initial;

    if (converged)
    {
        std::cout << print_prefix << "Scan matching converged for lc with indices: " << lc_indices.first << " | " << lc_indices.second << std::endl;
        std::cout << print_prefix << "ICP Fitness Score: " << fitness_score << std::endl;

        // check if the fitness score is above a certain upper boundary, if so scan matching was not good enough
        if (fitness_score > params.loop_closure.max_lc_icp_fitness)
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

    // REJECTORS:
    if (LCRejectors::reject_line_loop_closure(path, lc_indices.first, lc_indices.second, final_transformation))
    {
        std::cout << print_prefix << "LC Rejected (LINE)" << std::endl;
        return false;
    }
    else if (LCRejectors::reject_range_loop_closure_new(path, lc_indices.first, lc_indices.second, final_transformation, params))
    {
        std::cout << print_prefix << "LC Rejected (RANGE New)" << std::endl;
        return false;
    }
    // else if (LCRejectors::reject_range_loop_closure(path, lc_indices.first, lc_indices.second, final_transformation, params))
    // {
    //     std::cout << print_prefix << "LC Rejected (RANGE)" << std::endl;
    //     return false;
    // }

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
    // Eigen::Affine3f tWrong(path->at(lc_indices.second)->getTransformationMatrix());

    // // transform from world origin to corrected pose
    // Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
    // pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

    // gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    // gtsam::Pose3 poseTo(gtsam::Rot3(path->at(lc_indices.first)->quat.cast<double>()),
    //                     gtsam::Point3(path->at(lc_indices.first)->pos.cast<double>()));

    // auto between_fac = gtsam::BetweenFactor<gtsam::Pose3>(lc_indices.second, lc_indices.first, poseFrom.between(poseTo), constraintNoise);

    // ME
    // transform to gtsam pose3
    Eigen::Affine3f tCorrect = correctionLidarFrame; // the correction lidar frame is the transformation prev -> cur' (corrected pose)
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

void GTSAMWrapper::perform_pcl_icp(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
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
    icp.setInputSource(scan_cloud);
    icp.setInputTarget(model_cloud);
    icp.align(*result);

    // fill data with icp results
    fitness_score = icp.getFitnessScore();
    final_transformation = icp.getFinalTransformation();
    converged = icp.hasConverged();
}

void GTSAMWrapper::perform_pcl_gicp(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
                                    pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score, float max_corr_dist)
{
    if (max_corr_dist == -1.0f)
    {
        max_corr_dist = params.loop_closure.max_dist_lc * 2;
    }

    // align the clouds using generalized icp
    static pcl::GeneralizedIterativeClosestPoint<PointType, PointType> g_icp;
    g_icp.setMaximumIterations(params.loop_closure.max_icp_iterations);
    g_icp.setMaximumOptimizerIterations(100);
    g_icp.setTransformationEpsilon(0.01);
    g_icp.setMaxCorrespondenceDistance(params.loop_closure.max_dist_lc * 2);
    g_icp.setRANSACIterations(100);
    g_icp.setRANSACOutlierRejectionThreshold(params.loop_closure.max_dist_lc * 2);
    // g_icp.setUseReciprocalCorrespondences(false);
    // g_icp.addCorrespondenceRejector(pcl::r)

    g_icp.setInputSource(scan_cloud);
    g_icp.setInputTarget(model_cloud);
    g_icp.align(*result);

    // fill data with g_icp results
    fitness_score = g_icp.getFitnessScore();
    final_transformation = g_icp.getFinalTransformation();
    converged = g_icp.hasConverged();
}

void GTSAMWrapper::perform_adaptive_pcl_gicp(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
                                             pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score, float max_corr_dist)
{

    if (max_corr_dist == -1.0f)
    {
        max_corr_dist = params.loop_closure.max_dist_lc * 2;
    }

    Matrix4f current_transform = Matrix4f::Identity();
    pcl::PointCloud<PointType>::Ptr scan_copy;
    scan_copy.reset(new pcl::PointCloud<PointType>());

    // align the clouds using generalized icp
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> g_icp;
    g_icp.setMaximumIterations(1);
    g_icp.setTransformationEpsilon(0.001);
    g_icp.setMaxCorrespondenceDistance(max_corr_dist);
    // g_icp.setRANSACIterations(1);
    // g_icp.setRANSACOutlierRejectionThreshold(max_corr_dist);
    // g_icp.setUseReciprocalCorrespondences(false); // wechselseitig, also A=B und B=A

    int iterations = 0;
    float factor = 0.95f;

    while (iterations < 40)//params.loop_closure.max_icp_iterations)
    {
        iterations++;
        factor = std::pow(factor, (float)iterations);

        pcl::transformPointCloud(*scan_cloud, *scan_copy, current_transform);

        g_icp.setInputSource(scan_copy);
        g_icp.setInputTarget(model_cloud);
        g_icp.align(*result);

        // fill data with g_icp results
        fitness_score = g_icp.getFitnessScore();
        final_transformation = g_icp.getFinalTransformation();
        converged = g_icp.hasConverged();

        g_icp.setMaxCorrespondenceDistance(std::max(0.1, g_icp.getMaxCorrespondenceDistance() * factor));

        // std::cout << "Current corr dist: " << g_icp.getMaxCorrespondenceDistance() << std::endl;
        std::cout << "Current fitness score: " << g_icp.getFitnessScore() << std::endl;
        // std::cout << "GICP converged: " << g_icp.hasConverged() << std::endl;
    }

    std::cout << "AdaptiveGICP: Needed " << iterations << " Iterations" << std::endl;
    std::cout << "Final fitness score: " << g_icp.getFitnessScore() << std::endl;
}

void GTSAMWrapper::perform_vgicp(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
                                 pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score)
{
    // align the clouds using voxelized gicp and fill fitness score & co
    final_transformation = VGICP::gicp_transform(scan_cloud, model_cloud, fitness_score, converged);

    return;
}

void GTSAMWrapper::addNormal(pcl::PointCloud<PointType>::Ptr cloud,
                             pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<PointType>::Ptr searchTree(new pcl::search::KdTree<PointType>);
    searchTree->setInputCloud(cloud);

    pcl::NormalEstimation<PointType, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud);
    normalEstimator.setSearchMethod(searchTree);
    normalEstimator.setKSearch(15);
    normalEstimator.compute(*normals);

    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
}

void GTSAMWrapper::perform_pcl_normal_icp(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
                                          pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score)
{
    pcl::PointCloud<PointType>::Ptr cloud_scan_trans(new pcl::PointCloud<PointType>());
    cloud_scan_trans = scan_cloud;

    // prepare could with normals
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_scan_normals(new pcl::PointCloud<pcl::PointXYZINormal>());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_model_normals(new pcl::PointCloud<pcl::PointXYZINormal>());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_scan_trans_normals(new pcl::PointCloud<pcl::PointXYZINormal>());

    addNormal(scan_cloud, cloud_scan_normals);
    addNormal(model_cloud, cloud_model_normals);
    addNormal(cloud_scan_trans, cloud_scan_trans_normals);

    pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr icp(new pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>());
    icp->setMaximumIterations(params.loop_closure.max_icp_iterations);
    icp->setInputSource(cloud_scan_trans_normals); // not cloud_source, but cloud_source_trans!
    icp->setInputTarget(cloud_model_normals);

    icp->align(*cloud_scan_trans_normals); // use cloud with normals for ICP

    // fill data with icp results
    fitness_score = icp->getFitnessScore();
    final_transformation = icp->getFinalTransformation();
    converged = icp->hasConverged();

    return;
}

void GTSAMWrapper::perform_teaser_plus_plus(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
                                            pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score)
{
    // pcl pointclouds to teaser clouds

    // transform model cloud
    teaser::PointCloud t_model_cloud;

    for (auto point : *model_cloud)
    {
        teaser::PointXYZ t_point;
        t_point.x = point.x;
        t_point.y = point.y;
        t_point.z = point.z;

        t_model_cloud.push_back(t_point);
    }

    // transform scan cloud
    teaser::PointCloud t_scan_cloud;

    for (auto point : *scan_cloud)
    {
        teaser::PointXYZ t_point;
        t_point.x = point.x;
        t_point.y = point.y;
        t_point.z = point.z;

        t_scan_cloud.push_back(t_point);
    }

    // calculate features
    // Compute FPFH
    teaser::FPFHEstimation fpfh;
    auto obj_descriptors = fpfh.computeFPFHFeatures(t_scan_cloud, 0.08, 0.24);
    auto scene_descriptors = fpfh.computeFPFHFeatures(t_model_cloud, 0.08, 0.24);

    std::cout << "Number of scan descriptors: " << obj_descriptors->size() << std::endl;
    std::cout << "Number of model descriptors: " << scene_descriptors->size() << std::endl;

    teaser::Matcher matcher;
    auto correspondences = matcher.calculateCorrespondences(
        t_scan_cloud, t_model_cloud, *obj_descriptors, *scene_descriptors, false, true, false, 0.95);

    std::cout << "Number of FPFH correspondences: " << correspondences.size() << std::endl;

    // Run TEASER++ registration
    // Prepare solver parameters
    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = 0.05;
    params.cbar2 = 1;
    params.estimate_scaling = false;
    params.rotation_max_iterations = 100;
    params.rotation_gnc_factor = 1.4;
    params.rotation_estimation_algorithm =
        teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_cost_threshold = 0.005;

    // Solve with TEASER++
    teaser::RobustRegistrationSolver solver(params);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    solver.solve(t_scan_cloud, t_model_cloud, correspondences);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    auto solution = solver.getSolution();

    final_transformation = Matrix4f::Identity();

    final_transformation.block<3, 3>(0, 0) = solution.rotation.cast<float>();
    final_transformation.block<3, 1>(0, 3) = solution.translation.cast<float>();

    // std::cout << "Final transformation from TEASER++:: (readable)" << std::endl
    //           << Pose(final_transformation) << std::endl;

    if (!solution.valid)
    {
        std::cout << "Solution not valid" << std::endl;
    }

    // std::cout << "Time taken for T++ (s): "
    //           << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() /
    //                  1000000.0
    //           << std::endl;
}

void GTSAMWrapper::perform_own_teaser_plus_plus(pcl::PointCloud<PointType>::Ptr model_cloud, pcl::PointCloud<PointType>::Ptr scan_cloud,
                                                pcl::PointCloud<PointType>::Ptr result, bool &converged, Matrix4f &final_transformation, float &fitness_score)
{
    // estimate normals for model and scan cloud
    pcl::NormalEstimation<PointType, pcl::Normal> normal_estimation;
    normal_estimation.setRadiusSearch(0.1); // 10 cm
    pcl::PointCloud<pcl::Normal>::Ptr model_normals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr scan_normals(new pcl::PointCloud<pcl::Normal>());

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_model(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_scan(new pcl::PointCloud<pcl::FPFHSignature33>());

    normal_estimation.setInputCloud(model_cloud);
    normal_estimation.compute(*model_normals);

    normal_estimation.setInputCloud(scan_cloud);
    normal_estimation.compute(*scan_normals);

    pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh;

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    fpfh.setSearchMethod(tree);

    // Use all neighbors in a sphere of radius 15cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch(0.2);

    fpfh.setInputCloud(model_cloud);
    fpfh.setInputNormals(model_normals);

    // Compute the features
    fpfh.compute(*fpfhs_model);

    fpfh.setInputCloud(scan_cloud);
    fpfh.setInputNormals(scan_normals);

    // Compute the features
    fpfh.compute(*fpfhs_scan);

    std::cout << "Number of descriptors for scan cloud: " << fpfhs_scan->size() << std::endl;
    std::cout << "Number of descriptors for model cloud: " << fpfhs_model->size() << std::endl;

    // try estimating correspondences
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    est.setInputSource(fpfhs_scan);
    est.setInputTarget(fpfhs_model);
    est.determineCorrespondences(*correspondences);

    int counter = 0;
    int min_weight = std::numeric_limits<int>::max();
    int max_weight = -std::numeric_limits<int>::max();
    int num_bigger_zero = 0;

    long long avg_weight = 0;
    std::vector<int> weight_vector;

    for (auto correspondence : *correspondences)
    {
        if (correspondence.index_match != -1)
        {
            counter++;
        }

        if (correspondence.index_match >= fpfhs_model->size())
        {
            std::cout << "Index out of bounds of the model descriptor size" << std::endl;
        }

        if (correspondence.weight < min_weight)
        {
            min_weight = correspondence.weight;
        }

        if (correspondence.weight > max_weight)
        {
            max_weight = correspondence.weight;
        }

        if (correspondence.weight > 0)
        {
            num_bigger_zero++;
        }

        avg_weight += correspondence.weight;
        weight_vector.push_back(correspondence.weight);
    }

    std::sort(weight_vector.begin(), weight_vector.end());

    int median_weight = weight_vector.at(weight_vector.size() / 2);

    std::cout << "Found correspondences: " << counter << std::endl;
    std::cout << "Found min_weight:      " << min_weight << std::endl;
    std::cout << "Found max_weight:      " << max_weight << std::endl;
    std::cout << "Found average_weight:      " << avg_weight / weight_vector.size() << std::endl;
    std::cout << "Found median_weight:      " << median_weight << std::endl;
    std::cout << "Found weights > 0:      " << num_bigger_zero << std::endl;

    for (auto correspondence : *correspondences)
    {
        if (correspondence.index_match != -1)
        {
            counter++;
        }

        if (correspondence.index_match >= fpfhs_model->size())
        {
            std::cout << "Index out of bounds of the model descriptor size" << std::endl;
        }

        if (correspondence.weight < min_weight)
        {
            min_weight = correspondence.weight;
        }

        if (correspondence.weight > max_weight)
        {
            max_weight = correspondence.weight;
        }

        if (correspondence.weight > 0)
        {
            num_bigger_zero++;
        }

        avg_weight += correspondence.weight;
        weight_vector.push_back(correspondence.weight);
    }

    // holds <scan, match> index correspondences
    std::vector<std::pair<int, int>> pcl_correspondences;

    for (auto correspondence : *correspondences)
    {
        if (correspondence.weight > avg_weight)
        {
            if (correspondence.index_match < 0 || correspondence.index_query < 0)
                std::cout << "index < 0 !!" << std::endl;

            pcl_correspondences.push_back(std::make_pair(correspondence.index_query, correspondence.index_match));

            std::cout << correspondence.index_query << " | " << correspondence.index_match << std::endl;
        }
    }

    // transform model cloud
    teaser::PointCloud t_model_cloud;

    for (auto point : *model_cloud)
    {
        teaser::PointXYZ t_point;
        t_point.x = point.x;
        t_point.y = point.y;
        t_point.z = point.z;

        t_model_cloud.push_back(t_point);
    }

    // transform scan cloud
    teaser::PointCloud t_scan_cloud;

    for (auto point : *scan_cloud)
    {
        teaser::PointXYZ t_point;
        t_point.x = point.x;
        t_point.y = point.y;
        t_point.z = point.z;

        t_scan_cloud.push_back(t_point);
    }

    std::cout << __LINE__ << std::endl;

    // Run TEASER++ registration
    // Prepare solver parameters
    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = 0.05;
    params.cbar2 = 1;
    params.estimate_scaling = false;
    params.rotation_max_iterations = 100;
    params.rotation_gnc_factor = 1.4;
    params.rotation_estimation_algorithm =
        teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_cost_threshold = 0.005;

    std::cout << __LINE__ << std::endl;

    // Solve with TEASER++
    teaser::RobustRegistrationSolver solver(params);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    solver.solve(t_scan_cloud, t_model_cloud, pcl_correspondences);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << __LINE__ << std::endl;

    auto solution = solver.getSolution();

    std::cout << __LINE__ << std::endl;

    Eigen::Matrix4d solution_mat = Eigen::Matrix4d::Identity();
    solution_mat.block<3, 3>(0, 0) = solution.rotation;
    solution_mat.block<3, 1>(0, 3) = solution.translation;

    std::cout << __LINE__ << std::endl;

    std::cout << "Rotation output: " << std::endl
              << Pose(solution_mat.cast<float>()) << std::endl;

    std::cout << "Time taken for T++ (s): "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() /
                     1000000.0
              << std::endl;
}
