#pragma once

#include <vector>
#include "../util/point.h"
#include <string>
#include "../util/read_json.h"

class Path
{
private:
    std::vector<Pose> poses;

public:
    Path();
    void addPoseFromEuler(float x, float y, float z, float roll, float pitch, float yaw);
    void fromJSON(std::string filename);
};
