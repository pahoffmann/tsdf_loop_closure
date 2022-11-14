#include <loop_closure/map/map_updater.h>

namespace Map_Updater
{
    void partial_map_update_reverse(Path *old_path, Path *new_path, float transl_delta, float rotation_delta,
                                    std::vector<pcl::PointCloud<PointType>::Ptr> &clouds,
                                    GlobalMap *global_map_ptr, LocalMap *local_map_ptr,
                                    LoopClosureParams &params)
    {
        // indicates, wether there needs to be an update for the belonging poses
        std::vector<bool> update_incidences(old_path->get_length(), false);

        int num_updates = 0;

        for (int i = 0; i < old_path->get_length(); i++)
        {
            auto old_pose = old_path->at(i);
            auto new_pose = new_path->at(i);

            // std::cout << "Old pose: " << *old_pose << std::endl;
            // std::cout << "New pose: " << *new_pose << std::endl;

            Matrix4f old_tf_mat = old_pose->getTransformationMatrix();
            Matrix4f new_tf_mat = new_pose->getTransformationMatrix();

            Vector3f old_transl = old_tf_mat.block<3, 1>(0, 3);
            Vector3f new_transl = new_tf_mat.block<3, 1>(0, 3);

            auto transl_diff = (old_transl - new_transl).cwiseAbs();
            auto rotation_diff = get_rotation_diff(old_tf_mat, new_tf_mat) * (180 / M_PI); // rot diff in range [-180, 180]
            auto rotation_diff_abs = rotation_diff.cwiseAbs();                             // check the absolute rotation

            // indicates, wether the pose needs an update
            if (transl_diff.x() > transl_delta || transl_diff.z() > transl_delta || transl_diff.y() > transl_delta)
            {
                update_incidences[i] = true;
                num_updates++;
            }
            else if (rotation_diff_abs.x() > rotation_delta || rotation_diff_abs.y() > rotation_delta || rotation_diff_abs.z() > rotation_delta)
            {
                update_incidences[i] = true;
                num_updates++;
            }
        }

        if(num_updates == 0) return;

        global_map_ptr->clean_poses(update_incidences);
        local_map_ptr->reload();
        global_map_ptr->get_full_data();

        // now that we know which poses needs to be updated, we need to clear the map first and update it afterwards
        // go from last to first because this is the logical approach as this part of the map has been updated most recently
//         for (int i = update_incidences.size() - 1; i >= 0; i--)
//         {

//             // guard clause
//             if (!update_incidences[i])
//             {
//                 continue;
//             }

//             std::cout << "Clearing pose " << i << std::endl;

//             // needs to be deleted and renewed
//             auto current_pose_ptr = old_path->at(i);
//             auto pose_mat = current_pose_ptr->getTransformationMatrix();

//             pcl::PointCloud<PointType>::Ptr tsdf_cloud;
//             tsdf_cloud.reset(new pcl::PointCloud<PointType>());
//             pcl::PointCloud<PointType>::Ptr cur_cloud = clouds[i];
//             pcl::copyPointCloud(*cur_cloud, *tsdf_cloud);

//             // transform cloud according to new position
//             pcl::transformPointCloud(*tsdf_cloud, *tsdf_cloud, pose_mat);

//             // CREATE POINTCLOUD USED FOR TSDF UPDATE
//             pcl::VoxelGrid<PointType> sor;
//             sor.setInputCloud(tsdf_cloud);
//             sor.setLeafSize(params.map.resolution / 1000.0f, params.map.resolution / 1000.0f, params.map.resolution / 1000.0f);
//             sor.filter(*tsdf_cloud.get());

//             std::vector<Eigen::Vector3i> points_original(tsdf_cloud->size());

//             // transform points to map coordinates
// #pragma omp parallel for schedule(static) default(shared)
//             for (int j = 0; j < tsdf_cloud->size(); ++j)
//             {
//                 const auto &cp = (*tsdf_cloud)[j];
//                 points_original[j] = Eigen::Vector3i(cp.x * 1000.f, cp.y * 1000.f, cp.z * 1000.f);
//             }

//             // Shift
//             Vector3i input_3d_pos = real_to_map(pose_mat.block<3, 1>(0, 3));
//             local_map_ptr->shift(input_3d_pos);

//             // auto lmap_center_diff_abs = (local_map_ptr->get_pos() - input_3d_pos).cwiseAbs();
//             // Eigen::Vector3f l_map_half_f = local_map_ptr->get_size().cast<float>();
//             // l_map_half_f *= 0.5f;
//             // Eigen::Vector3i l_map_half = l_map_half_f.cast<int>();

//             // if (lmap_center_diff_abs.x() > l_map_half.x() || lmap_center_diff_abs.y() > l_map_half.y() || lmap_center_diff_abs.z() > l_map_half.z())
//             // {
//             //     local_map_ptr->shift(input_3d_pos);
//             // }

//             Eigen::Matrix4i rot = Eigen::Matrix4i::Identity();
//             rot.block<3, 3>(0, 0) = to_int_mat(pose_mat).block<3, 3>(0, 0);
//             Eigen::Vector3i up = transform_point(Eigen::Vector3i(0, 0, MATRIX_RESOLUTION), rot);

//             // create TSDF Volume
//             reverse_update_tsdf(points_original, input_3d_pos, up, *local_map_ptr, params.map.tau, params.map.max_weight, params.map.resolution, i);

//             // write data back
//             local_map_ptr->write_back();
//         }

        

        // iterate again, update map, here start from the front.
        for (int i = 0; i < update_incidences.size(); i++)
        {
            // guard clause
            if (!update_incidences[i])
            {
                continue;
            }

            std::cout << "Updating for pose " << i << std::endl;

            auto current_pose_ptr = new_path->at(i);
            auto pose_mat = current_pose_ptr->getTransformationMatrix();

            pcl::PointCloud<PointType>::Ptr tsdf_cloud;
            tsdf_cloud.reset(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr cur_cloud = clouds[i];
            pcl::copyPointCloud(*cur_cloud, *tsdf_cloud);

            // transform cloud according to new position
            pcl::transformPointCloud(*tsdf_cloud, *tsdf_cloud, pose_mat);

            // CREATE POINTCLOUD USED FOR TSDF UPDATE
            pcl::VoxelGrid<PointType> sor;
            sor.setInputCloud(tsdf_cloud);
            sor.setLeafSize(params.map.resolution / 1000.0f, params.map.resolution / 1000.0f, params.map.resolution / 1000.0f);
            sor.filter(*tsdf_cloud.get());

            std::vector<Eigen::Vector3i> points_original(tsdf_cloud->size());

            // transform points to map coordinates
#pragma omp parallel for schedule(static) default(shared)
            for (int j = 0; j < tsdf_cloud->size(); ++j)
            {
                const auto &cp = (*tsdf_cloud)[j];
                points_original[j] = Eigen::Vector3i(cp.x * 1000.f, cp.y * 1000.f, cp.z * 1000.f);
            }

            // Shift
            Vector3i input_3d_pos = real_to_map(pose_mat.block<3, 1>(0, 3));
            local_map_ptr->shift(input_3d_pos);

            // auto lmap_center_diff_abs = (local_map_ptr->get_pos() - input_3d_pos).cwiseAbs();
            // Eigen::Vector3f l_map_half_f = local_map_ptr->get_size().cast<float>();
            // l_map_half_f *= 0.5f;
            // Eigen::Vector3i l_map_half = l_map_half_f.cast<int>();

            // if (lmap_center_diff_abs.x() > l_map_half.x() || lmap_center_diff_abs.y() > l_map_half.y() || lmap_center_diff_abs.z() > l_map_half.z())
            // {
            //     local_map_ptr->shift(input_3d_pos);
            // }

            Eigen::Matrix4i rot = Eigen::Matrix4i::Identity();
            rot.block<3, 3>(0, 0) = to_int_mat(pose_mat).block<3, 3>(0, 0);
            Eigen::Vector3i up = transform_point(Eigen::Vector3i(0, 0, MATRIX_RESOLUTION), rot);

            // create TSDF Volume
            update_tsdf(points_original, input_3d_pos, up, *local_map_ptr, params.map.tau, params.map.max_weight, params.map.resolution, i);

            // write data back
            local_map_ptr->write_back();
        }
    }

    void partial_map_update(Path *old_path, Path *new_path, float transl_delta, float rotation_delta,
                            std::vector<pcl::PointCloud<PointType>::Ptr> &clouds,
                            GlobalMap *global_map_ptr, LocalMap *local_map_ptr,
                            LoopClosureParams &params, RayTracer *tracer)
    {
        // indicates, wether there needs to be an update for the belonging poses
        std::vector<bool> update_incidences(old_path->get_length(), false);

        for (int i = 0; i < old_path->get_length(); i++)
        {
            auto old_pose = old_path->at(i);
            auto new_pose = new_path->at(i);

            std::cout << "Old pose: " << *old_pose << std::endl;
            std::cout << "New pose: " << *new_pose << std::endl;

            Matrix4f old_tf_mat = old_pose->getTransformationMatrix();
            Matrix4f new_tf_mat = new_pose->getTransformationMatrix();

            Vector3f old_transl = old_tf_mat.block<3, 1>(0, 3);
            Vector3f new_transl = new_tf_mat.block<3, 1>(0, 3);

            auto transl_diff = (old_transl - new_transl).cwiseAbs();
            auto rotation_diff = get_rotation_diff(old_tf_mat, new_tf_mat) * (180 / M_PI); // rot diff in range [-180, 180]
            auto rotation_diff_abs = rotation_diff.cwiseAbs();                             // check the absolute rotation

            std::cout << "Got rotation difference: " << std::endl
                      << rotation_diff << std::endl;

            std::cout << "Got translation difference: " << std::endl
                      << transl_diff << std::endl;

            // indicates, wether the pose needs an update
            if (transl_diff.x() > transl_delta || transl_diff.z() > transl_delta || transl_diff.y() > transl_delta)
            {
                update_incidences[i] = true;
                std::cout << "Pose " << i << " needs an update (transl)" << std::endl;
            }
            else if (rotation_diff_abs.x() > rotation_delta || rotation_diff_abs.y() > rotation_delta || rotation_diff_abs.z() > rotation_delta)
            {
                update_incidences[i] = true;
                std::cout << "Pose " << i << " needs an update (rotation)" << std::endl;
            }
        }

        // might need to check the incidence array here
        // if there are holes meaning: true false true, it might be useful, to still update the one in between
        // also: if (true false false true) and the bounding boxes of surrounding true localmaps overlap, still upate the ones in the center
        // to remain consistant
        for (int i = 0; i < update_incidences.size(); i++)
        {
            if (i != 0 && i != update_incidences.size() - 1 && update_incidences.at(i) == false)
            {
            }
        }

        // now that we know which poses needs to be updated, we need to clear the map first and update it afterwards
        // go from last to first because this is the logical approach as this part of the map has been updated most recently
        for (int i = update_incidences.size() - 1; i >= 0; i--)
        {
            // guard clause
            if (!update_incidences[i])
            {
                continue;
            }

            // needs to be deleted and renewed

            // do a raytracing approach to remove any belonging cells
            tracer->local_removal(old_path->at(i), i);
        }

        // iterate again, update map, here start from the front.
        //         for (int i = 0; i < update_incidences.size(); i++)
        //         {
        //             // guard clause
        //             if (!update_incidences[i])
        //             {
        //                 continue;
        //             }

        //             auto current_pose_ptr = new_path->at(i);
        //             auto pose_mat = current_pose_ptr->getTransformationMatrix();

        //             pcl::PointCloud<PointType>::Ptr tsdf_cloud;
        //             tsdf_cloud.reset(new pcl::PointCloud<PointType>());
        //             pcl::PointCloud<PointType>::Ptr cur_cloud = clouds[i];
        //             pcl::copyPointCloud(*cur_cloud, *tsdf_cloud);

        //             // transform cloud according to new position
        //             pcl::transformPointCloud(*tsdf_cloud, *tsdf_cloud, pose_mat);

        //             // CREATE POINTCLOUD USED FOR TSDF UPDATE
        //             pcl::VoxelGrid<PointType> sor;
        //             sor.setInputCloud(tsdf_cloud);
        //             sor.setLeafSize(params.map.resolution / 1000.0f, params.map.resolution / 1000.0f, params.map.resolution / 1000.0f);
        //             sor.filter(*tsdf_cloud.get());

        //             std::vector<Eigen::Vector3i> points_original(tsdf_cloud->size());

        //             // transform points to map coordinates
        // #pragma omp parallel for schedule(static) default(shared)
        //             for (int j = 0; j < tsdf_cloud->size(); ++j)
        //             {
        //                 const auto &cp = (*tsdf_cloud)[j];
        //                 points_original[j] = Eigen::Vector3i(cp.x * 1000.f, cp.y * 1000.f, cp.z * 1000.f);
        //             }

        //             // Shift
        //             Vector3i input_3d_pos = real_to_map(pose_mat.block<3, 1>(0, 3));

        //             auto lmap_center_diff_abs = (local_map_ptr->get_pos() - input_3d_pos).cwiseAbs();
        //             Eigen::Vector3f l_map_half_f = local_map_ptr->get_size().cast<float>();
        //             l_map_half_f *= 0.5f;
        //             Eigen::Vector3i l_map_half = l_map_half_f.cast<int>();

        //             if (lmap_center_diff_abs.x() > l_map_half.x() || lmap_center_diff_abs.y() > l_map_half.y() || lmap_center_diff_abs.z() > l_map_half.z())
        //             {
        //                 local_map_ptr->shift(input_3d_pos);
        //             }

        //             Eigen::Matrix4i rot = Eigen::Matrix4i::Identity();
        //             rot.block<3, 3>(0, 0) = to_int_mat(pose_mat).block<3, 3>(0, 0);
        //             Eigen::Vector3i up = transform_point(Eigen::Vector3i(0, 0, MATRIX_RESOLUTION), rot);

        //             // create TSDF Volume
        //             update_tsdf(points_original, input_3d_pos, up, *local_map_ptr, params.map.tau, params.map.max_weight, params.map.resolution, i);

        //             // write data back
        //             local_map_ptr->write_back();
        //         }
    }

    void full_map_update(Path *new_path, std::vector<pcl::PointCloud<PointType>::Ptr> &clouds,
                         GlobalMap *global_map_ptr, LocalMap *local_map_ptr,
                         LoopClosureParams &params, std::string suffix)
    {
        // reset global and localmap

        std::cout << print_prefix << "Resetting old global and local map..." << std::endl;
        auto num_cells_prior = global_map_ptr->get_full_data().size();

        // refill tsdf map
        for (int i = 0; i < new_path->get_length(); i++)
        {
            std::cout << "Update process: (" << i + 1 << "/" << new_path->get_length() << ")" << std::endl;

            auto current_pose_ptr = new_path->at(i);
            auto pose_mat = current_pose_ptr->getTransformationMatrix();

            pcl::PointCloud<PointType>::Ptr tsdf_cloud;
            tsdf_cloud.reset(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr cur_cloud = clouds[i];
            pcl::copyPointCloud(*cur_cloud, *tsdf_cloud);

            // transform cloud according to new position
            pcl::transformPointCloud(*tsdf_cloud, *tsdf_cloud, pose_mat);

            // CREATE POINTCLOUD USED FOR TSDF UPDATE
            pcl::VoxelGrid<PointType> sor;
            sor.setInputCloud(tsdf_cloud);
            sor.setLeafSize(params.map.resolution / 1000.0f, params.map.resolution / 1000.0f, params.map.resolution / 1000.0f);
            sor.filter(*tsdf_cloud.get());

            std::vector<Eigen::Vector3i> points_original(tsdf_cloud->size());

            // transform points to map coordinates
#pragma omp parallel for schedule(static) default(shared)
            for (int j = 0; j < tsdf_cloud->size(); ++j)
            {
                const auto &cp = (*tsdf_cloud)[j];
                points_original[j] = Eigen::Vector3i(cp.x * 1000.f, cp.y * 1000.f, cp.z * 1000.f);
            }

            // Shift
            Vector3i input_3d_pos = real_to_map(pose_mat.block<3, 1>(0, 3));
            local_map_ptr->shift(input_3d_pos);

            // auto lmap_center_diff_abs = (local_map_ptr->get_pos() - input_3d_pos).cwiseAbs();
            // Eigen::Vector3f l_map_half_f = local_map_ptr->get_size().cast<float>();
            // l_map_half_f *= 0.5f;
            // Eigen::Vector3i l_map_half = l_map_half_f.cast<int>();

            // if (lmap_center_diff_abs.x() > l_map_half.x() || lmap_center_diff_abs.y() > l_map_half.y() || lmap_center_diff_abs.z() > l_map_half.z())
            // {
            //     local_map_ptr->shift(input_3d_pos);
            // }

            Eigen::Matrix4i rot = Eigen::Matrix4i::Identity();
            rot.block<3, 3>(0, 0) = to_int_mat(pose_mat).block<3, 3>(0, 0);
            Eigen::Vector3i up = transform_point(Eigen::Vector3i(0, 0, MATRIX_RESOLUTION), rot);

            // create TSDF Volume
            update_tsdf(points_original, input_3d_pos, up, *local_map_ptr, params.map.tau, params.map.max_weight, params.map.resolution, i);

            // write data back
            local_map_ptr->write_back();
        }

        std::cout << print_prefix << "Done updating global and local map" << std::endl;

        auto num_cells_after = global_map_ptr->get_full_data().size();

        std::cout << print_prefix << "Num cells in global map prior: " << num_cells_prior << std::endl;
        std::cout << print_prefix << "Num cells in global map after: " << num_cells_after << std::endl;
    }
}
