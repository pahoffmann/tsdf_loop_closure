#include "path_exploration.h"

path_exploration::path_exploration(std::shared_ptr<LocalMap> l_map_ptr, std::shared_ptr<GlobalMap> g_map_ptr, Path *path_ptr, RayTracer *tracer_ptr)
{
    ray_tracer = tracer_ptr;
    local_map_ptr = l_map_ptr;
    global_map_ptr = g_map_ptr;
    path = path_ptr;
}

path_exploration::~path_exploration()
{
}

int path_exploration::explore(int num_randoms, int max_num_selects)
{
    // global map always begins at (0,0,0)
    Eigen::Vector3i start_pos(0, 0, 0);

    return 0;
}

std::string path_exploration::hash_from_point(Eigen::Vector3i pos)
{
    return "(" + std::to_string(pos.x()) + ")-(" + std::to_string(pos.y()) + ")-(" + std::to_string(pos.z()) + ")";
}

void path_exploration::fill_pq() {

    // initial
    dijkstra_vertex start;
    start.chunk_pos = Vector3i(0,0,0);
    start.distance = 0;
    start.previous = NULL;

    bool more_inserted = true;
    
    auto chunks = global_map_ptr->all_chunk_poses();

    while(more_inserted) {
        more_inserted = false;
    }
}

void path_exploration::dijsktra() {
    // before doing anything, we need to fill the prio queue :)
    std::cout << "Filling pq" << std::endl;
    fill_pq();
}