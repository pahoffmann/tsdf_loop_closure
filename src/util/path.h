#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Path
{
private:
    std::vector<geometry_msgs::Pose> poses;
public:
    Path();
    void addPoseFromEuler(float x, float y, float z, float eulerX, float eulerY, float eulerZ);
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
 * @param eulerX  roll
 * @param eulerY  pitch
 * @param eulerZ  yawn
 */
void Path::addPoseFromEuler(float x, float y, float z, float eulerX, float eulerY, float eulerZ) 
{
    geometry_msgs::Pose pose;

    // get quaternion from rpy
    tf2::Quaternion quat;
    quat.setEuler(eulerX, eulerY, eulerZ);
    
    // build point
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;

    //merge into pose msg
    pose.orientation = tf2::toMsg(quat);
    pose.position = point;

    poses.push_back(pose);
}

