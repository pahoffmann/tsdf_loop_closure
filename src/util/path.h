#pragma once

#include <vector>
#include "point.h"
#include <string>
#include "read_json.h"

class Path
{
private:
    std::vector<Pose> poses;

public:
    Path();
    void addPoseFromEuler(float x, float y, float z, float roll, float pitch, float yaw);
    void fromJSON(std::string filename);
};

/**
 * @brief constructor, currently empty
 *
 */
Path::Path(/* args */)
{
}

void Path::fromJSON(std::string filename)
{
    PATH::json_to_path(filename, poses);
}
