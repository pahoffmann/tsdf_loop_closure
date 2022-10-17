#include <loop_closure/util/point.h>
#include <loop_closure/path/path.h>
#include <loop_closure/params/loop_closure_params.h>

namespace LCRejectors
{

    /**
     * @brief identifies wether the found lc might be rejected, because it is a line lc, that has been smushed
     *
     * @param path
     * @param start_idx
     * @param end_idx
     * @param final_transform
     * @return true
     * @return false
     */
    static bool reject_range_loop_closure(Path *path, int start_idx, int end_idx, Matrix4f final_transform, LoopClosureParams params)
    {
        // STEP 1: Initialize required components

        // calculate the initial transform
        Matrix4f initial_transform = getTransformationMatrixBetween(path->at(end_idx)->getTransformationMatrix(), path->at(start_idx)->getTransformationMatrix());
        // Matrix4f initial_transform = getTransformationMatrixBetween(path->at(start_idx)->getTransformationMatrix(), path->at(end_idx)->getTransformationMatrix());

        auto initial_transl = initial_transform.block<3, 1>(0, 3);
        auto new_transl = final_transform.block<3, 1>(0, 3);

        Eigen::Matrix3f initial_rotation = initial_transform.block<3, 3>(0, 0);
        Eigen::Matrix3f new_rotation = final_transform.block<3, 3>(0, 0);

        // STEP 2: DETECT WETHER THE FINAL TRANSFORM IS NOT IN RANGE AROUND THE ACTUAL TF, DETERMINED BY THE TRANSLATION AND ROTATION NOISE

#ifdef TEST_DEBUG
        std::cout << "Previous Transform: " << std::endl
                  << Pose(initial_transform) << std::endl;

        std::cout << "New Transform: " << std::endl
                  << Pose(final_transform) << std::endl;
#endif

        Vector3f translation_noise;
        translation_noise << params.loop_closure.between_translation_noise_x, params.loop_closure.between_translation_noise_y, params.loop_closure.between_translation_noise_z;
        Vector3f rotation_noise;
        rotation_noise << params.loop_closure.between_rotation_noise_x, params.loop_closure.between_rotation_noise_y, params.loop_closure.between_rotation_noise_z;

        Vector3f acc_transl_noise = translation_noise * (end_idx - start_idx);
        Vector3f acc_rot_noise = rotation_noise * (end_idx - start_idx);

        // check translation range
        auto translation_diff = initial_transl - new_transl;
        auto translation_diff_abs = translation_diff.cwiseAbs();

        Vector3f range_from_transl = initial_transl - acc_transl_noise;
        Vector3f range_to_transl = initial_transl + acc_transl_noise;

#ifdef TEST_DEBUG
        std::cout << "Previous translation: " << std::endl
                  << initial_transl << std::endl;

        std::cout << "New translation: " << std::endl
                  << new_transl << std::endl;
#endif

        std::cout << "Allowed range: " << std::endl
                  << range_from_transl << std::endl
                  << "-------------------------------" << std::endl
                  << range_to_transl << std::endl;

        if (translation_diff_abs.x() > acc_transl_noise.x() || translation_diff_abs.y() > acc_transl_noise.y() || translation_diff_abs.z() > acc_transl_noise.z())
        {
            std::cout << "[LCLineRejector] RANGE LC REJECTED (TRANS)" << std::endl;
            return true;
        }

        // rotation

        // get euler angles in range [0, 2pi]
        Eigen::Affine3f init_trans_affine;
        init_trans_affine = initial_transform;
        Eigen::Affine3f final_trans_affine;
        final_trans_affine = final_transform;

        float initial_roll, initial_pitch, initial_yaw;
        pcl::getEulerAngles(init_trans_affine, initial_roll, initial_pitch, initial_yaw);

        float new_roll, new_pitch, new_yaw;
        pcl::getEulerAngles(final_trans_affine, new_roll, new_pitch, new_yaw);

        // auto initial_rot_vec = initial_rotation.eulerAngles(0, 1, 2) + Vector3f::Ones() * M_PI;
        // auto new_rot_vec = new_rotation.eulerAngles(0, 1, 2) + Vector3f::Ones() * M_PI;

        auto initial_rot_vec = Vector3f(initial_roll, initial_pitch, initial_yaw) + Vector3f::Ones() * M_PI;
        auto new_rot_vec = Vector3f(new_roll, new_pitch, new_yaw) + Vector3f::Ones() * M_PI;

        Vector3f rotation_diff = initial_rot_vec - new_rot_vec;
        Vector3f rotation_diff_abs = rotation_diff.cwiseAbs();

        // if the diff is > 180, we need to use the inverse: 360 - diff
        if (rotation_diff_abs.x() > M_PI)
        {
            rotation_diff_abs.x() = 2 * M_PI - rotation_diff_abs.x();
        }
        else if (rotation_diff_abs.y() > M_PI)
        {
            rotation_diff_abs.y() = 2 * M_PI - rotation_diff_abs.y();
        }
        else if (rotation_diff_abs.z() > M_PI)
        {
            rotation_diff_abs.z() = 2 * M_PI - rotation_diff_abs.z();
        }

        Vector3f range_from_rot = (initial_rot_vec + acc_rot_noise);
        Vector3f range_to_rot = initial_rot_vec - acc_rot_noise;

#ifdef TEST_DEBUG
        std::cout << "Previous rotation: " << std::endl
                  << initial_rot_vec * (180.0f / M_PI) << std::endl;

        std::cout << "New rotation: " << std::endl
                  << new_rot_vec * (180.0f / M_PI) << std::endl;

        std::cout << "Rotation diff: " << std::endl
                  << rotation_diff * (180.0f / M_PI) << std::endl;

        std::cout << "In between noise: " << std::endl
                  << rotation_noise * (180.0f / M_PI) << std::endl;

        std::cout << "In between noise: " << std::endl
                  << acc_rot_noise << std::endl;
#endif

        std::cout
            << "Allowed range: " << std::endl
            << range_from_rot * (180.0f / M_PI) << std::endl
            << "-------------------------------" << std::endl
            << range_to_rot * (180.0f / M_PI) << std::endl;

        if (rotation_diff_abs.x() > acc_rot_noise.x() || rotation_diff_abs.y() > acc_rot_noise.y() || rotation_diff_abs.z() > acc_rot_noise.z())
        {
            std::cout << "[LCLineRejector] RANGE LC REJECTED (ROT)" << std::endl;
            return true;
        }

        return false;
    }
}