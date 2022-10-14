#include <loop_closure/util/point.h>
#include <loop_closure/path/path.h>

namespace LCRejectors
{

    /**
     * @brief detects wether the the sub-path between start_idx and end_idx is a line
     *
     * @param path
     * @param start_idx
     * @param end_idx
     * @return true
     * @return false
     */
    static bool is_line(Path *path, int start_idx, int end_idx, float max_line_dist = 0.5f)
    {

        // calculate the difference between start and end
        Vector3f line_vector = path->at(end_idx)->pos - path->at(start_idx)->pos;

#ifdef TEST_DEBUP
        std::cout << "line vector: " << std::endl
                  << line_vector << std::endl;
#endif
        Vector3f line_location_vector = path->at(start_idx)->pos;

#ifdef TEST_DEBUG
        std::cout << "line start point vector: " << std::endl
                  << line_location_vector << std::endl;
#endif
        Vector3f line_end_vector = path->at(end_idx)->pos;

#ifdef TEST_DEBUG
        std::cout << "line end point vector: " << std::endl
                  << line_end_vector << std::endl;
#endif

        int in_between_start_idx = start_idx + 1;
        int in_between_end_idx = end_idx - 1;

        // the loop closure was found between poses, which directly follow each other (end_idx = start_idx + 1)
        if (in_between_end_idx > in_between_end_idx)
        {
            return false;
        }

        // Todo: make this dependend on the number of poses between the lc poses
        float max_angle_between_poses = 20.0f; // maybe for later

        for (int idx = in_between_start_idx; idx <= in_between_end_idx; idx++)
        {
            Vector3f point = path->at(idx)->pos;

#ifdef TEST_DEBUG
            std::cout << "Point: " << std::endl
                      << point << std::endl;
#endif
            // check distance to line
            // float distance = ((line_end_vector - line_location_vector).cross(line_location_vector - point)).norm() / (line_end_vector - line_location_vector).norm();
            float distance = ((point - line_location_vector).cross(point - line_end_vector)).norm() / (line_end_vector - line_location_vector).norm();

#ifdef TEST_DEBUG
            std::cout << "Calculated dist: " << distance << std::endl;
#endif

            // not a line
            if (distance > max_line_dist)
            {
                return false;
            }
            // check angle (todo)
        }

        return true;
    }

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
    static bool reject_line_loop_closure(Path *path, int start_idx, int end_idx, Matrix4f &final_transform)
    {
        // STEP 1: DETECT LINE LC

        // calculate the difference
        Vector3f line_vector = path->at(end_idx)->pos - path->at(start_idx)->pos;
        Vector3f line_location_vector = path->at(start_idx)->pos;
        Vector3f line_end_vector = path->at(end_idx)->pos;

        auto lc_is_line = is_line(path, start_idx, end_idx, 2.0f);

        if(!lc_is_line)
        {
            std::cout << "[LCLineRejector] Not a line" << std::endl;
            return false;
        }

        // here we know, that it actually is a line
        std::cout << "[LCLineRejector].... Detected a line LC, checking if the LC needs to be rejected ..." << std::endl;

        // STEP 2: DETECT WETHER THE FINAL TRANSFORM SMUSHES THE POSES TOGETHER LIKE A ZIEHHARMONIKA
        Matrix4f previous_diff = getTransformationMatrixBetween(path->at(end_idx)->getTransformationMatrix(), path->at(start_idx)->getTransformationMatrix());
        auto previous_transl = previous_diff.block<3, 1>(0, 3);
        auto new_transl = final_transform.block<3, 1>(0, 3);

#ifdef TEST_DEBUG
        std::cout << "Previous translation: " << std::endl
                  << previous_transl << std::endl;

        std::cout << "New translation: " << std::endl
                  << new_transl << std::endl;
#endif

        // STEP 3: DECIDE IF THE LC NEEDS TO BE REJECTED

        // if the change is bigger than the accumulated translation noise defined in each pose, this is bad
        float between_transl_noise = 0.1f; // todo: get this from parametrizatuion
        int num_between_poses = (end_idx - start_idx);
        Vector3f acc_noise_vector = Vector3f::Ones() * (between_transl_noise * num_between_poses);

        Vector3f range_from = previous_transl + acc_noise_vector;
        Vector3f range_to = previous_transl - acc_noise_vector;

        std::cout << "Allowed range: " << std::endl
                  << previous_transl + acc_noise_vector << std::endl
                  << "-------------------------------" << std::endl
                  << previous_transl - acc_noise_vector << std::endl;

        auto diff_vector = previous_transl - new_transl;
        auto diff_vector_abs = diff_vector.cwiseAbs();
        
        // if (std::abs(previous_transl.norm() - new_transl.norm()) > acc_noise_vector.norm())
        if (diff_vector_abs.x() > acc_noise_vector.x() || diff_vector_abs.y() > acc_noise_vector.y() || diff_vector_abs.z() > acc_noise_vector.z())
        {
            std::cout << "[LCLineRejector] LINE LC REJECTED" << std::endl;
            return true;
        }

        return false;
    }
}