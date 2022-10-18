#include <loop_closure/map/map_updater.h>

namespace Map_Updater
{

    void partial_map_update(Path *old_path, Path *new_path, float transl_delta, float rotation_delta,
                            std::vector<pcl::PointCloud<PointType>::Ptr> &clouds,
                            std::shared_ptr<GlobalMap> global_map_ptr, std::shared_ptr<LocalMap> local_map_ptr,
                            LoopClosureParams &params)
    {
        // indicates, wether there needs to be an update for the belonging poses
        std::vector<bool> update_incidences(old_path->get_length(), false);

        for (int i = 0; i < old_path->get_length(); i++)
        {
            auto old_pose = old_path->at(i);
            auto new_pose = new_path->at(i);

            Matrix4f old_tf_mat = old_pose->getTransformationMatrix();
            Matrix4f new_tf_mat = new_pose->getTransformationMatrix();

            Vector3f old_transl = old_tf_mat.block<3, 1>(0, 3);
            Vector3f new_transl = new_tf_mat.block<3, 1>(0, 3);

            auto transl_diff = (old_transl - new_transl).cwiseAbs();

            // indicates, wether the pose needs an update
            if (transl_diff.x() > transl_delta || transl_diff.z() > transl_delta || transl_diff.y() > transl_delta)
            {
                update_incidences[i] = true;
            }
        }
    }

    void full_map_update(Path *new_path, std::vector<pcl::PointCloud<PointType>::Ptr> &clouds,
                         std::shared_ptr<GlobalMap> global_map_ptr, std::shared_ptr<LocalMap> local_map_ptr,
                         LoopClosureParams &params, std::string suffix)
    {
        // reset global and localmap

        std::cout << print_prefix << "Resetting old global and local map..." << std::endl;

        auto previous_filename_path = params.map.filename;
        // create new map
        params.map.filename = previous_filename_path.parent_path() / (boost::filesystem::path(previous_filename_path.stem().string() + "_" + suffix).string() + previous_filename_path.extension().string());

        // delete old
        boost::filesystem::remove(previous_filename_path);

        // reset pointers
        global_map_ptr.reset(new GlobalMap(params.map));
        local_map_ptr.reset(new LocalMap(params.map.size.x(), params.map.size.y(), params.map.size.y(), global_map_ptr));

        // refill tsdf map
        for (int i = 0; i < new_path->get_length(); i++)
        {
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
            for (int i = 0; i < tsdf_cloud->size(); ++i)
            {
                const auto &cp = (*tsdf_cloud)[i];
                points_original[i] = Eigen::Vector3i(cp.x * 1000.f, cp.y * 1000.f, cp.z * 1000.f);
            }

            // Shift
            Vector3i input_3d_pos = real_to_map(pose_mat.block<3, 1>(0, 3));
            local_map_ptr->shift(input_3d_pos);

            Eigen::Matrix4i rot = Eigen::Matrix4i::Identity();
            rot.block<3, 3>(0, 0) = to_int_mat(pose_mat).block<3, 3>(0, 0);
            Eigen::Vector3i up = transform_point(Eigen::Vector3i(0, 0, MATRIX_RESOLUTION), rot);

            // create TSDF Volume
            update_tsdf(points_original, input_3d_pos, up, *local_map_ptr, params.map.tau, params.map.max_weight, params.map.resolution);

            // write data back
            local_map_ptr->write_back();
        }

        std::cout << print_prefix << "Done updating global and local map" << std::endl;
        auto num_cells = global_map_ptr->get_full_data().size();

        std::cout << print_prefix << "Num cells in map updater: " << num_cells << std::endl;
    }
}
