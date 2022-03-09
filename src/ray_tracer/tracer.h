#pragma once

#include <iostream>
#include <math.h>
#include <cuda_profiler_api.h>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "../util/point.h"
#include <loop_closure/LoopClosureConfig.h>
#include "../map/local_map.h"

namespace CudaTracing {
 int helloWorld();
 void updateRays(std::vector<Eigen::Vector3f> *rays, Pose *start_pose, loop_closure::LoopClosureConfig *config);
};

