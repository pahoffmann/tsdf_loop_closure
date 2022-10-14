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

        auto initial_transl = initial_transform.block<3, 1>(0, 3);
        auto new_transl = final_transform.block<3, 1>(0, 3);

        Eigen::Matrix3f initial_rotation = initial_transform.block<3, 3>(0, 0);
        Eigen::Matrix3f initial_rotation = final_transform.block<3, 3>(0, 0);

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

        // WIP: go on rejecting


        // if the change is bigger than the accumulated translation noise defined in each pose, this is bad
        // float between_transl_noise = 0.1f; // todo: get this from parametrizatuion
        // int num_between_poses = (end_idx - start_idx - 1);
        // Vector3f acc_noise_vector = Vector3f::Ones() * (between_transl_noise * num_between_poses);

        // Vector3f range_from = previous_transl + acc_noise_vector;
        // Vector3f range_to = previous_transl - acc_noise_vector;

        // std::cout << "Allowed range: " << std::endl
        //           << previous_transl + acc_noise_vector << std::endl
        //           << "-------------------------------" << std::endl
        //           << previous_transl - acc_noise_vector << std::endl;

        // auto diff_vector = previous_transl - new_transl;
        // auto diff_vector_abs = diff_vector.cwiseAbs();

        // // if (std::abs(previous_transl.norm() - new_transl.norm()) > acc_noise_vector.norm())
        // if (diff_vector_abs.x() > acc_noise_vector.x() || diff_vector_abs.y() > acc_noise_vector.y() || diff_vector_abs.z() > acc_noise_vector.z())
        // {
        //     std::cout << "[LCLineRejector] LINE LC REJECTED" << std::endl;
        //     return true;
        // }

        return false;
    }
}