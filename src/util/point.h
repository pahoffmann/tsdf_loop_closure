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
    
    // gets the rotation matrix from the quat
    Eigen::Matrix3f rotationMatrixFromQuaternion()
    {
        return quat.matrix();
    }
    
    Pose() {
        // default
    }

    // copy constructor
    Pose(const Pose &other) {
        quat = other.quat;
        pos = other.pos;
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

/**
 * @brief Creates a pose from euler angles and position
 * 
 * @param x 
 * @param y 
 * @param z 
 * @param roll 
 * @param pitch 
 * @param yaw 
 * @return Pose 
 */
static Pose poseFromEuler(float x, float y, float z, float roll, float pitch, float yaw)
{
    Pose pose;
    // create eigen quaternion from euler
    Eigen::Quaternionf q;
    auto rollAngle = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
    auto pitchAngle = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY());
    auto yawAngle = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    q = yawAngle * pitchAngle * rollAngle;

    // build point
    Eigen::Vector3f point(x, y, z);

    // merge into pose msg
    pose.quat = q;
    pose.pos = point;

    return pose;
}

