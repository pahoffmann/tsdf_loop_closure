#include <loop_closure/path/path.h>
#include <loop_closure/util/csv_wrapper.h>
#include <loop_closure/util/point.h>

namespace Evaluation
{
    static std::string print_prefix = "[Evaluation] ";

    static float calc_absolute_translation_error(Path *path, Path *ground_truth, bool avg = true)
    {
        float acc_error = 0.0f;

        if (path->get_length() < 1)
        {
            return acc_error;
        }

        for (int i = 0; i < path->get_length(); i++)
        {
            auto pose = path->at(i);
            auto gt_pose = ground_truth->at(i);

            Vector3f transl_diff = gt_pose->getTransformationMatrix().block<3, 1>(0, 3) - pose->getTransformationMatrix().block<3, 1>(0, 3);
            Vector3f transl_diff_abs = transl_diff.cwiseAbs();

            // std::cout << "Pose: " << std::endl
            //           << pose->getTransformationMatrix().block<3, 1>(0, 3) << std::endl;
            // std::cout << "GT Pose: " << std::endl
            //           << gt_pose->getTransformationMatrix().block<3, 1>(0, 3) << std::endl;
            // std::cout << "Diff: " << std::endl
            //           << transl_diff_abs << std::endl;

            acc_error += transl_diff_abs.norm();
        }

        if (avg)
        {
            acc_error /= path->get_length();
        }

        std::cout << "Absolute translation error to GT: " << acc_error << std::endl;

        return acc_error;
    }

    static float calc_relative_translation_error(Path *path, Path *ground_truth, bool avg = true)
    {
        float acc_error = 0.0f;

        if (path->get_length() < 2)
        {
            return acc_error;
        }

        for (int i = 0; i < path->get_length() - 1; i++)
        {
            auto pose_1 = path->at(i);
            auto gt_pose_1 = ground_truth->at(i);

            auto pose_2 = path->at(i + 1);
            auto gt_pose_2 = ground_truth->at(i + 1);

            auto diff = (pose_2->getTransformationMatrix().block<3, 1>(0, 3) - pose_1->getTransformationMatrix().block<3, 1>(0, 3)).cwiseAbs();
            auto gt_diff = (gt_pose_2->getTransformationMatrix().block<3, 1>(0, 3) - gt_pose_1->getTransformationMatrix().block<3, 1>(0, 3)).cwiseAbs();

            auto relative_diff = (gt_diff - diff).cwiseAbs();

            // std::cout << "Diff: " << std::endl
            //           << diff << std::endl;
            // std::cout << "GT Diff: " << std::endl
            //           << gt_diff << std::endl;
            // std::cout << "Relative diff: " << std::endl
            //           << relative_diff << std::endl;

            acc_error += relative_diff.norm();
        }

        if (avg)
        {
            acc_error /= path->get_length();
        }

        std::cout << "Absolute translation error to GT: " << acc_error << std::endl;

        return acc_error;
    }
}