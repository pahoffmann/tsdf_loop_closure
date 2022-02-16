#pragma once

/**
 * @file point.h
 * @author Malte Hillmann, Patrick Hoffmann
 */

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>

using Eigen::Vector3i;
using Eigen::Vector3f;
using Eigen::Matrix4i;
using Eigen::Matrix4f;
using Eigen::Quaternionf;
using ScanPoints_t = std::vector<Vector3i>;
using ScanPointType = int32_t;
using ScanPoint = Eigen::Matrix<ScanPointType, 3, 1>;

struct Pose {
    Eigen::Quaternionf quat;
    Eigen::Vector3f pos;
    
    Eigen::Matrix3f rotationMatrixFromQuaternion()
    {
        return quat.matrix();
    }
};

/**
 * @brief calculates floor(a / b), except that a is a Vector and all values are integers
 *
 * @param a the numerator
 * @param b the denominator
 * @return floor(a / b)
 */
static inline Vector3i floor_divide(const Vector3i& a, int b)
{
    return Vector3i(
               std::floor(static_cast<float>(a[0]) / b),
               std::floor(static_cast<float>(a[1]) / b),
               std::floor(static_cast<float>(a[2]) / b)
           );
}

