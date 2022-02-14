#pragma once

#include <vector>
#include "point.h"


class Path
{
private:
    std::vector<Pose> poses;
    
public:
    Path();
    void addPoseFromEuler(float x, float y, float z, float roll, float pitch, float yaw);
};

/**
 * @brief constructor, currently empty
 * 
 */
Path::Path(/* args */)
{

}

/**
 * @brief Adds a 6D pose to the path (using ros atm, maybe switch to eigen)
 * 
 * @param x       x - pose
 * @param y       y - pose
 * @param z       z - pose
 * @param roll
 * @param pitch
 * @param yaw
 */
void Path::addPoseFromEuler(float x, float y, float z, float roll, float pitch, float yaw) 
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

    //merge into pose msg
    pose.quat = q;
    pose.pos = point;

    poses.push_back(pose);
}

