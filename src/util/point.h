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
#include "constants.h"

using Eigen::Matrix4f;
using Eigen::Matrix4i;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using Eigen::Vector3i;
using ScanPoints_t = std::vector<Vector3i>;
using ScanPointType = int32_t;
using ScanPoint = Eigen::Matrix<ScanPointType, 3, 1>;

/**
 * @brief struct defining a pose, might also use combinded 4x4 matrix representation, see:
 *        https://stackoverflow.com/questions/25504397/eigen-combine-rotation-and-translation-into-one-matrix
 *
 */
struct Pose
{
    Eigen::Quaternionf quat;
    Eigen::Vector3f pos;

    // gets the rotation matrix from the quat
    Eigen::Matrix3f rotationMatrixFromQuaternion()
    {
        return quat.matrix();
    }

    Pose()
    {
        // default
    }

    // create a pose from a matrix 4f
    Pose(const Matrix4f &mat) {
        quat = mat.block<3, 3>(0, 0);
        pos = mat.block<3, 1>(0, 3);
    }

    // copy constructor
    Pose(const Pose &other)
    {
        quat = other.quat;
        pos = other.pos;
    }

    /**
     * @brief Used to add two poses
     *
     * @param other
     * @return Pose
     */
    Pose operator+(const Pose &other)
    {
        Pose tmp;
        tmp.quat = this->quat * other.quat;
        tmp.pos = this->pos + other.pos;
        return tmp;
    }

    friend std::ostream& operator<<(std::ostream& os, const Pose& pose) {
        os << pose.pos.x() << " | " << pose.pos.y() << " | " << pose.pos.z() << std::endl;
        return os;
    }

};

/**
 * @brief calculates floor(a / b), except that a is a Vector and all values are integers
 *
 * @param a the numerator
 * @param b the denominator
 * @return floor(a / b)
 */
static inline Vector3i floor_divide(const Vector3i &a, int b)
{
    return Vector3i(
        std::floor(static_cast<float>(a[0]) / b),
        std::floor(static_cast<float>(a[1]) / b),
        std::floor(static_cast<float>(a[2]) / b));
}

/**
 * @brief calculates floor(a / b), except that a is a Vector and all values are integers
 *
 * @param a the numerator
 * @param b the denominator
 * @return floor(a / b)
 */
static inline Vector3i ceil_divide(const Vector3i &a, int b)
{
    return Vector3i(
        std::ceil(static_cast<float>(a[0]) / b),
        std::ceil(static_cast<float>(a[1]) / b),
        std::ceil(static_cast<float>(a[2]) / b));
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

/**
 * @brief transforms a vector into a tag used for hashmaps
 * 
 * @param vec 
 * @return std::string 
 */
static std::string tag_from_vec(Vector3i vec)
{
    std::stringstream ss;
    ss << vec.x() << "_" << vec.y() << "_" << vec.z();
    return ss.str();
}

/**
 * @brief transforms a vector into a tag used for hashmaps
 * 
 * @param vec 
 * @return std::string 
 */
static std::string tag_from_vec(Vector3f vec)
{
    std::stringstream ss;
    ss << vec.x() << "_" << vec.y() << "_" << vec.z();
    return ss.str();
}


/**
 * @brief transforms a hashmap tag back into a vector
 * 
 * @param tag 
 * @return Vector3i 
 */
static Vector3i vec_from_tag(std::string tag)
{
    std::stringstream tmp(tag);
    std::string segment;
    std::vector<std::string> seglist;

    while (std::getline(tmp, segment, '_'))
    {
        seglist.push_back(segment);
    }

    if (seglist.size() != 3)
    {
        throw std::logic_error("Error when transforming a tag into a chunk pos");
    }

    // get coords
    int x = std::stoi(seglist[0]);
    int y = std::stoi(seglist[1]);
    int z = std::stoi(seglist[2]);

    return Vector3i(x, y, z);
}


/**
 * @brief Converts a real world values points (in metres) to a point in map coordinates
 * 
 * @param real 
 * @return Vector3i 
 */
static Vector3i real_to_map(Vector3f real) {
    return (real * (1000.0f / MAP_RESOLUTION)).cast<int>();
}

/**
 * @brief Converts a map coordinate point to a real world point (metres).
 * 
 * @param map 
 * @return Vector3f 
 */
static Vector3f map_to_real(Vector3i map) {
    return map.cast<float>() * (MAP_RESOLUTION / 1000.0f);
}
