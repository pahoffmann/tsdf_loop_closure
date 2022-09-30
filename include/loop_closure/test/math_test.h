#pragma once

/**
 * @file math_test.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief
 * @version 0.1
 * @date 2022-08-24
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <unordered_map>
#include <map>
#include <math.h>

#include <boost/unordered_map.hpp>
#include <loop_closure/util/point.h>
#include <loop_closure/util/constants.h>
#include <loop_closure/util/colors.h>
#include <loop_closure/util/eigen_vs_ros.h>
#include <loop_closure/path/path.h>
#include <loop_closure/util/random.h>
#include <loop_closure/data_association/association.h>
#include <loop_closure/ray_tracer/ray_tracer.h>
#include <loop_closure/visualization/ros_viewhelper.h>

namespace Testing
{
    enum MapUpdateMethod
    {
        MEAN,
        WEIGHTED,
        RAND
    };

    /**
     * @brief Get the pose differences between two paths
     *
     * @param before
     * @param after
     * @return std::vector<Eigen::Matrix4f>
     */
    static std::vector<Eigen::Matrix4f> get_pose_differences(Path *before, Path *after)
    {
        std::vector<Matrix4f> pose_differences;

        for (int i = 0; i < before->get_length(); i++)
        {
            Matrix4f previous_pose = before->at(i)->getTransformationMatrix();
            Matrix4f new_pose = after->at(i)->getTransformationMatrix();

            pose_differences.push_back(getTransformationMatrixBetween(previous_pose, new_pose));
        }

        return pose_differences;
    }

    /**
     * @brief Get the pose differences between two paths (rotation and translation seperate)
     *
     * @param before
     * @param after
     * @return std::vector<std::pair<Eigen::Matrix3f, Eigen::Vector3f>>
     */
    static std::vector<std::pair<Eigen::Matrix3f, Eigen::Vector3f>> get_pose_differences_pair(Path *before, Path *after)
    {
        std::vector<std::pair<Eigen::Matrix3f, Eigen::Vector3f>> pose_differences;

        for (int i = 0; i < before->get_length(); i++)
        {
            Eigen::Matrix3f rotation_before = before->at(i)->rotationMatrixFromQuaternion();
            Eigen::Matrix3f rotation_after = after->at(i)->rotationMatrixFromQuaternion();

            Eigen::Matrix3f rotation_diff = rotation_before.inverse() * rotation_after;
            Eigen::Vector3f translation_diff = after->at(i)->pos - before->at(i)->pos;

            pose_differences.push_back(std::make_pair(rotation_diff, translation_diff));
        }

        return pose_differences;
    }

    /**
     * @brief Method testing the math behind the transformation of a distinct cell (here: a distinct point), given a new and an old path
     *        This method will use the mean approach, meaning that it will calculate the mean cell out of all pose differences.
     *
     * @param cells_before
     * @param before
     * @param after
     * @return std::pair<visualization_msgs::Marker, visualization_msgs::Marker>
     */
    static std::pair<visualization_msgs::Marker, visualization_msgs::Marker> test_cell_transformation(
        std::vector<Vector3i> cells_before, Path *before, Path *after, int start_idx = -1, int end_idx = -1)
    {
        // determine indices
        if (start_idx == -1)
        {
            start_idx = 0;
        }

        if (end_idx == -1)
        {
            end_idx = before->get_length() - 1;
        }

        if (start_idx < 0 || start_idx > end_idx || end_idx >= before->get_length())
        {
            throw std::logic_error("[MathTest] Error with indexation of cell transformation test");
        }

        // obtain pose differences
        std::vector<Matrix4f> pose_differences = get_pose_differences(before, after);

        std::vector<Vector3i> cells_after(cells_before.size());

        // transform input points
        for (int i = 0; i < cells_before.size(); i++)
        {
            Vector3i cell = cells_before[i];
            Vector3f real_cell_pos = map_to_real(cell);

            Vector3f new_pos = Vector3f::Zero();
            int counter = 0;

            Matrix4f accumulated_mat = Matrix4f::Identity();

            // update this taking into account the start and end index
            for (int j = start_idx; j <= end_idx; j++)
            {
                // get beloning pose diff
                Matrix4f diff_mat = pose_differences[j];

                // for testing purposes
                accumulated_mat = diff_mat * accumulated_mat;

                Eigen::Matrix3f rot_mat = diff_mat.block<3, 3>(0, 0);
                Vector3f transl_vec = diff_mat.block<3, 1>(0, 3);

                // calc new vector, if rotated around before vec

                // random assertion ( wrong though )
                if (before->at(j)->pos + transl_vec != after->at(j)->pos)
                {
                    // throw std::logic_error("[MathTest] The two operands should be equal");
                }

                // newpos -> rotate old cell pose (which has been transformed to (0,0,0) for adequate rotation), transform back and apply translation vector
                Eigen::Vector4f before_4d;
                before_4d << before->at(j)->pos, 1;

                Vector3f transformed = rot_mat * (real_cell_pos - before->at(j)->pos) + before->at(j)->pos + transl_vec;
                //Vector3f transformed = (diff_mat * before_4d).head(3);

                new_pos += transformed;
                counter++;
            }

            new_pos /= counter;

            auto centroid_before = before->get_centroid();
            auto centroid_after = after->get_centroid();

            // cells_after[i] = real_to_map_relative(new_pos, centroid_before);
            // cells_after[i] = real_to_map_relative(new_pos, centroid_after);
            cells_after[i] = real_to_map(new_pos);
        }

        // create ros markers from arrays
        auto marker_before = ROSViewhelper::marker_from_map_points(cells_before);
        auto marker_after = ROSViewhelper::marker_from_map_points(cells_after);

        return std::make_pair(marker_before, marker_after);
    }

    /**
     * @brief Method testing the math behind the transformation of a distinct cell (here: a distinct point), given a new and an old path
     *        This method will
     *
     * @param cells_before
     * @param before
     * @param after
     * @todo
     * @return std::pair<visualization_msgs::Marker, visualization_msgs::Marker>
     */
    static std::pair<visualization_msgs::Marker, visualization_msgs::Marker> test_cell_transformation_weighted(std::vector<Vector3i> cells_before, Path *before, Path *after)
    {
        auto pose_differences = get_pose_differences_pair(before, after);

        // test idea behind pose differences

        for (int i = 0; i < pose_differences.size(); i++)
        {
            auto rotation_diff = pose_differences[i].first;
            auto translation_diff = pose_differences[i].second;

            if (before->at(i)->pos + translation_diff != after->at(i)->pos)
            {
                std::cout << "WHAT? TRANSL: " << std::endl
                          << std::endl
                          << before->at(i)->pos << std::endl
                          << std::endl
                          << after->at(i)->pos << std::endl
                          << std::endl
                          << translation_diff << std::endl
                          << std::endl
                          << before->at(i)->pos + translation_diff << std::endl
                          << std::endl;
            }

            if (before->at(i)->rotationMatrixFromQuaternion() * rotation_diff != after->at(i)->rotationMatrixFromQuaternion())
            {
                std::cout << "WHAT? ROT: " << std::endl
                          << std::endl
                          << before->at(i)->rotationMatrixFromQuaternion() << std::endl
                          << std::endl
                          << after->at(i)->rotationMatrixFromQuaternion() << std::endl
                          << std::endl
                          << rotation_diff << std::endl
                          << std::endl
                          << rotation_diff * before->at(i)->rotationMatrixFromQuaternion() << std::endl
                          << std::endl;
            }

            Eigen::Matrix4f transformation_before_after = after->at(i)->getTransformationMatrix() * before->at(i)->getTransformationMatrix().inverse();
            Eigen::Vector4f before_4d, after_4d;
            before_4d << before->at(i)->pos , 1;
            after_4d << after->at(i)->pos , 1;
            Eigen::Vector4f transformed_form = transformation_before_after * before_4d;

            if (transformed_form.head(3) != after_4d.head(3))
            {
                std::cout << "WHAT? FORM: " << std::endl
                          << std::endl
                          << before_4d << std::endl
                          << std::endl
                          << before->at(i)->getTransformationMatrix() << std::endl
                          << std::endl
                          << after->at(i)->getTransformationMatrix() << std::endl
                          << std::endl
                          << after_4d << std::endl
                          << std::endl
                          << transformation_before_after << std::endl
                          << std::endl
                          << transformed_form << std::endl
                          << std::endl;
            }
        }

        std::vector<Vector3i> cells_after(cells_before.size());

        // create ros markers from arrays
        auto marker_before = ROSViewhelper::marker_from_map_points(cells_before);
        auto marker_after = ROSViewhelper::marker_from_map_points(cells_after);

        return std::make_pair(marker_before, marker_after);
    }

    /**
     * @brief Method testing the math behind the transformation of a distinct cell (here: a distinct point), given a new and an old path
     *        This method will
     *
     * @param cells_before
     * @param before
     * @param after
     * @return std::pair<visualization_msgs::Marker, visualization_msgs::Marker>
     */
    static std::pair<visualization_msgs::Marker, visualization_msgs::Marker> test_cell_transformation_rnd(
        std::vector<Vector3i> cells_before, Path *before, Path *after)
    {
        std::vector<Matrix4f> pose_differences = get_pose_differences(before, after);

        std::vector<Vector3i> cells_after(cells_before.size());

        // transform input points
        for (int i = 0; i < cells_before.size(); i++)
        {
            Vector3i cell = cells_before[i];
            Vector3f real_cell_pos = map_to_real(cell);

            Vector3f new_pos = Vector3f::Zero();
            int counter = 0;

            Matrix4f accumulated_mat = Matrix4f::Identity();

            // cells are seen by a random number of random path poses - simulate this (kinda)
            // 5-10 cells of the path

            int start_idx = 0;
            int end_idx = 0;

            if (before->get_length() < 10)
            {
                end_idx = before->get_length() - 1;
            }
            else
            {
                end_idx = generateRandomNumber(9, before->get_length() - 1);
                start_idx = end_idx - 5;
                // end_idx = 0;
                // start_idx = 0;
            }

            // update this taking into account the start and end index
            for (int j = start_idx; j <= end_idx; j++)
            {
                // get beloning pose diff
                Matrix4f diff_mat = pose_differences[j];

                // for testing purposes
                accumulated_mat = diff_mat * accumulated_mat;

                Eigen::Matrix3f rot_mat = diff_mat.block<3, 3>(0, 0);
                Vector3f transl_vec = diff_mat.block<3, 1>(0, 3);

                // calc new vector, if rotated around before vec

                // random assertion ( wrong though )
                if (before->at(j)->pos + transl_vec != after->at(j)->pos)
                {
                    // throw std::logic_error("[MathTest] The two operands should be equal");
                }

                // newpos -> rotate old cell pose (which has been transformed to (0,0,0) for adequate rotation), transform back and apply translation vector
                Vector3f transformed = rot_mat * (real_cell_pos - before->at(j)->pos) + before->at(j)->pos + transl_vec;
                new_pos += transformed;
                counter++;
            }

            new_pos /= counter;

            auto centroid_before = before->get_centroid();
            auto centroid_after = after->get_centroid();

            // cells_after[i] = real_to_map_relative(new_pos, centroid_before);
            // cells_after[i] = real_to_map_relative(new_pos, centroid_after);
            cells_after[i] = real_to_map(new_pos);
        }

        // create ros markers from arrays
        auto marker_before = ROSViewhelper::marker_from_map_points(cells_before);
        auto marker_after = ROSViewhelper::marker_from_map_points(cells_after);

        return std::make_pair(marker_before, marker_after);
    }

    /**
     * @brief generates a wall (a layer) of cells to check out transformations
     *
     * @param start_pos
     * @param dir_1
     * @param dir_2
     * @return std::vector<Vector3i>
     */
    static std::vector<Vector3i> generate_cell_wall(
        Eigen::Vector3i start_pos, Vector3i dir_1, Vector3i dir_2, int num_steps1 = 10, int num_steps2 = 20)
    {
        std::vector<Vector3i> wall;
        for (int i = 0; i < num_steps1; i++)
        {
            for (int j = 0; j < num_steps2; j++)
            {
                // calc wall point
                Vector3i tmp = start_pos + i * dir_1 + j * dir_2;

                // add to wall
                wall.push_back(tmp);
            }
        }

        return wall;
    }

    /**
     * @brief generates a cube (a block) of cells to check out transformations
     *
     * @param start_pos
     * @param dir_1
     * @param dir_2
     * @param dir_3
     * @return std::vector<Vector3i>
     */
    static std::vector<Vector3i> generate_cell_cube(
        Eigen::Vector3i start_pos, Vector3i dir_1, Vector3i dir_2, Vector3i dir_3, int num_steps1 = 10, int num_steps2 = 20, int num_steps3 = 40)
    {
        std::vector<Vector3i> cube;
        for (int i = 0; i < num_steps1; i++)
        {
            for (int j = 0; j < num_steps2; j++)
            {
                for (int k = 0; k < num_steps3; k++)
                {
                    // calc wall point
                    Vector3i tmp = start_pos + i * dir_1 + j * dir_2 + k * dir_3;

                    // add to wall
                    cube.push_back(tmp);
                }
            }
        }

        return cube;
    }

    /**
     * @brief
     * @todo This is not yet fully implemented and momentarily just an idea to be pursued to reduce the number of code lines
     *
     * @param before
     * @param after
     * @param cells_before
     * @param pose_differences
     * @param start_idx
     * @param end_idx
     * @param method
     * @return std::vector<Vector3i>
     */
    /*static std::vector<Vector3i> calc_new_cell_poses(
        Path *before, Path *after, std::vector<Vector3i> &cells_before, std::vector<Eigen::Matrix4f> pose_differences, int start_idx, int end_idx, MapUpdateMethod method)
    {
        // vector to be filled and returned
        std::vector<Vector3i> cells_after;

        // transform input points
        for (int i = 0; i < cells_before.size(); i++)
        {
            Vector3i cell = cells_before[i];
            Vector3f real_cell_pos = map_to_real(cell);

            Vector3f new_pos = Vector3f::Zero();
            int counter = 0;

            Matrix4f accumulated_mat = Matrix4f::Identity();

            // random start and end index, if method == rand
            if (method == MapUpdateMethod::RAND)
            {
                if (before->get_length() < 10)
                {
                    end_idx = before->get_length() - 1;
                }
                else
                {
                    end_idx = generateRandomNumber(9, before->get_length() - 1);
                    start_idx = end_idx - 5;
                }
            }

            // update this taking into account the start and end index
            for (int j = start_idx; j <= end_idx; j++)
            {
                // get beloning pose diff
                Matrix4f diff_mat = pose_differences[j];

                // for testing purposes
                accumulated_mat = diff_mat * accumulated_mat;

                Eigen::Matrix3f rot_mat = diff_mat.block<3, 3>(0, 0);
                Vector3f transl_vec = diff_mat.block<3, 1>(0, 3);

                // calc new vector, if rotated around before vec

                // newpos -> rotate old cell pose (which has been transformed to (0,0,0) for adequate rotation), transform back and apply translation vector
                Vector3f transformed = rot_mat * (real_cell_pos - before->at(j)->pos) + before->at(j)->pos + transl_vec;
                new_pos += transformed;
                counter++;
            }

            new_pos /= counter;

            auto centroid_before = before->get_centroid();
            auto centroid_after = after->get_centroid();

            // cells_after[i] = real_to_map_relative(new_pos, centroid_before);
            // cells_after[i] = real_to_map_relative(new_pos, centroid_after);
            cells_after[i] = real_to_map(new_pos);
        }
    }*/
}