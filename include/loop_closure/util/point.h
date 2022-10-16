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
#include <sstream>
#include <iomanip>
#include <boost/unordered_map.hpp>

#include <loop_closure/util/constants.h>
#include <loop_closure/util/algorithm.h>

#include <pcl/point_types.h>

using Eigen::Matrix4f;
using Eigen::Matrix4i;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using Eigen::Vector3i;
using ScanPoints_t = std::vector<Vector3i>;
using ScanPointType = int32_t;
using ScanPoint = Eigen::Matrix<ScanPointType, 3, 1>;

typedef pcl::PointXYZI PointType;

/**
 * @brief struct defining a pose, might also use combinded 4x4 matrix representation, see:
 *        https://stackoverflow.com/questions/25504397/eigen-combine-rotation-and-translation-into-one-matrix
 *
 */
struct Pose
{
    Eigen::Quaternionf quat;
    Eigen::Vector3f pos;

    // TODO: use covariance
    Eigen::MatrixXf covariance;

    // gets the rotation matrix from the quat
    Eigen::Matrix3f rotationMatrixFromQuaternion()
    {
        return quat.matrix();
    }

    Eigen::Matrix4f getTransformationMatrix()
    {
        //  identity so bottom right corner is filled
        Eigen::Matrix4f tmp = Eigen::Matrix4f::Identity();
        tmp.block<3, 3>(0, 0) = rotationMatrixFromQuaternion();
        tmp.block<3, 1>(0, 3) = pos;
        
        return tmp;
    }

    Pose()
    {
        // default
    }

    // create a pose from a matrix 4f
    Pose(const Matrix4f &mat)
    {
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

    /**
     * @brief function used to add a vector to the current 3d position
     *
     * @param vec
     */
    void add(Vector3f vec)
    {
        this->pos += vec;
    }

    friend std::ostream &operator<<(std::ostream &os, const Pose &pose)
    {
        // -180 - 180
        Vector3f euler = pose.quat.toRotationMatrix().eulerAngles(0, 1, 2);
        euler *= (180.0f / M_PI);
        os << std::fixed << std::setprecision(2)
           << "Pos:    " << pose.pos.x() << " | " << pose.pos.y() << " | " << pose.pos.z() << std::endl
           << "Angles: " << euler.x() << " | " << euler.y() << " | " << euler.z() << std::endl;
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
 * @brief Creates a pose from euler angles and position ( the angles need to be in degree, not rediants)
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
    auto rollAngle = Eigen::AngleAxisf(roll * (M_PI / 180.0f), Eigen::Vector3f::UnitX());
    auto pitchAngle = Eigen::AngleAxisf(pitch * (M_PI / 180.0f), Eigen::Vector3f::UnitY());
    auto yawAngle = Eigen::AngleAxisf(yaw * (M_PI / 180.0f), Eigen::Vector3f::UnitZ());

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
static Vector3i real_to_map(Vector3f real)
{
    return (real * (1000.0f) / MAP_RESOLUTION).unaryExpr([](float val)
                                                         { return std::floor(val); })
        .cast<int>();
    // return (real * (1000.0f / MAP_RESOLUTION)).cast<int>();
}

/**
 * @brief because in c++ a round operation always rounds to zero we will encounter problems here,
 *        because it is supposed to round towards the center of the local map
 *
 * @param real
 * @param center
 * @return Vector3i
 */
static Vector3i real_to_map_relative(Vector3f real, Vector3f center)
{
    Vector3f tmp = real - center;
    Vector3i real_tmp = real_to_map(tmp);
    Vector3i center_tmp = real_to_map(center);

    return real_tmp + center_tmp;
}

/**
 * @brief function used as a test case to ensure, rounding is appropriate
 *
 * @param real
 * @return Vector3f
 */
static Vector3f real_to_map_float(Vector3f real)
{
    return real * (1000.0f / MAP_RESOLUTION);
}

/**
 * @brief Converts a map coordinate point to a real world point (metres).
 *
 * @param map
 * @return Vector3f
 */
static Vector3f map_to_real(Vector3i map)
{
    return map.cast<float>() * (MAP_RESOLUTION / 1000.0f);
}

static size_t hash_from_vec(Vector3i vec)
{
    size_t seed = 0;
    boost::hash_combine(seed, vec.x());
    boost::hash_combine(seed, vec.y());
    boost::hash_combine(seed, vec.z());

    return seed;
}

/**
 * @brief Get the transformation difference between to 3D Positions, displayed as 2 4x4 matrices, do this component-wise
 *
 * @param mat1
 * @param mat2
 * @return Eigen::Matrix4f
 */
static Eigen::Matrix4f getTransformationMatrixBetweenComp(Eigen::Matrix4f mat1, Eigen::Matrix4f mat2)
{
    // extract components
    Eigen::Matrix3f rot_com1 = mat1.block<3, 3>(0, 0);
    Eigen::Matrix3f rot_com2 = mat2.block<3, 3>(0, 0);
    Vector3f transl_comp1 = mat1.block<3, 1>(0, 3);
    Vector3f transl_comp2 = mat2.block<3, 1>(0, 3);

    Matrix4f tmp = Eigen::Matrix4f::Identity();

    Eigen::Matrix3f rot_diff = rot_com1.inverse() * rot_com2;
    Eigen::Vector3f transl_diff = transl_comp2 - transl_comp1;

    tmp.block<3, 3>(0, 0) = rot_diff;
    tmp.block<3, 1>(0, 3) = transl_diff;

    return tmp;
}

/**
 * @brief Calculates the delta between poses, aka. the transformation needed to get from coordinate system mat2 to coordinate system mat1
 *
 * @param mat1
 * @param mat2
 * @return Eigen::Matrix4f
 */
static Eigen::Matrix4f getTransformationMatrixBetween(Eigen::Matrix4f mat1, Eigen::Matrix4f mat2)
{
    // as matrix multiplication is not commutative, the order here is extremely important
    return mat1.inverse() * mat2;
}

/// THE FOLLOWING IS USED TO PREPARE DATA FOR TSDF UPDATE ///
inline Eigen::Matrix4i to_int_mat(const Eigen::Matrix4f &mat)
{
    return (mat * MATRIX_RESOLUTION).cast<int>();
}

inline Eigen::Vector3i transform_point(const Eigen::Vector3i &input, const Eigen::Matrix4i &mat)
{
    Eigen::Vector4i v;
    v << input, 1;
    return (mat * v).block<3, 1>(0, 0) / MATRIX_RESOLUTION;
}
/// THE PREVIOUS IS USED TO PREPARE DATA FOR TSDF UPDATE ///
