#include "ray_tracer.h"

RayTracer::RayTracer() {
    // default
}

RayTracer::RayTracer(loop_closure::RayTracerConfig* new_config, std::shared_ptr<LocalMap> local_map_in, Pose* start_pose) 
{
    rt_config = new_config;
    local_map_ptr_ = local_map_in;
    current_pose = start_pose;
}

void RayTracer::start() {
    // we casually ignore calls to this function, when 
    if(rt_config == NULL || current_pose == NULL)
    {
        return;
    }
}

void RayTracer::initRays() {
    
    
}

void RayTracer::updateRays() {

}

visualization_msgs::Marker* RayTracer::get_ros_marker() {
    return &marker;
}


