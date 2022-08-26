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
#include <boost/unordered_map.hpp>
#include <loop_closure/util/point.h>
#include <loop_closure/util/constants.h>
#include <loop_closure/util/colors.h>
#include <loop_closure/util/eigen_vs_ros.h>
#include <loop_closure/path/path.h>
#include <loop_closure/data_association/association.h>
#include <loop_closure/ray_tracer/ray_tracer.h>
#include <loop_closure/visualization/ros_viewhelper.h>

namespace Testing
{
    /**
     * @brief Method testing the math behind the transformation of a distinct cell (here: a distinct point), given a new and an old path
     *        This method will
     *
     * @param cell_before
     * @param before
     * @param after
     * @return std::pair<visualization_msgs::Marker, visualization_msgs::Marker>
     */
    static std::pair<visualization_msgs::Marker, visualization_msgs::Marker> test_cell_transformation(std::vector<Vector3i> cells_before, Path *before, Path *after, int start_idx = -1, int end_idx = -1)
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

        std::vector<Matrix4f> pose_differences;

        for (int i = 0; i < before->get_length(); i++)
        {
            Matrix4f previous_pose = before->at(i)->getTransformationMatrix();
            Matrix4f new_pose = after->at(i)->getTransformationMatrix();

            pose_differences.push_back(getTransformationMatrixDiffComp(previous_pose, new_pose));
        }

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

                // random assertion
                if (before->at(j)->pos + transl_vec != after->at(j)->pos)
                {
                    // throw std::logic_error("[MathTest] The two operands should be equal");
                }

                Vector3f transformed = rot_mat * (real_cell_pos - before->at(j)->pos) + before->at(j)->pos + transl_vec;
                new_pos += transformed;
                counter++;
            }

            new_pos /= counter;

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
    static std::vector<Vector3i> generate_cell_wall(Eigen::Vector3i start_pos, Vector3i dir_1, Vector3i dir_2, int num_steps1 = 10, int num_steps2 = 20)
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
    static std::vector<Vector3i> generate_cell_cube(Eigen::Vector3i start_pos, Vector3i dir_1, Vector3i dir_2, Vector3i dir_3, int num_steps1 = 10, int num_steps2 = 20, int num_steps3 = 40)
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
}