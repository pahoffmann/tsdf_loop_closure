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

void path_exploration::fill_pq()
{

    // initial
    dijkstra_vertex start;
    start.chunk_pos = Vector3i(0, 0, 0);
    start.distance = 0;
    start.previous = NULL;

    bool more_inserted = true;

    auto chunks = global_map_ptr->all_chunk_poses();

    for (auto chunk : chunks)
    {
        dijkstra_vertex *vertex = new dijkstra_vertex();
        vertex->chunk_pos = chunk;
        pq.push(vertex);
    }
}

/**
 * @brief 
 * TODO: a path needs to be tracable here...
 * 
 */
void path_exploration::dijsktra()
{
    // before doing anything, we need to fill the prio queue :)
    std::cout << "Filling pq" << std::endl;
    // fill_pq();
    int num_chunks = global_map_ptr->num_chunks();

    std::map<std::string, int> distance_map;

    //auto chunks = global_map_ptr->all_chunk_poses(local_map_ptr->get_size());
    auto chunks = global_map_ptr->all_chunk_poses();

    // fill distance map
    for (auto chunk : chunks)
    {
        distance_map[global_map_ptr->tag_from_chunk_pos(chunk)] = std::numeric_limits<int>::infinity();
    }

    std::cout << "Chunks in the dm: " << distance_map.size() << std::endl;

    // initial
    dijkstra_vertex *start = new dijkstra_vertex();
    start->chunk_pos = Vector3i(0, 0, 0);
    start->distance = 0;
    start->previous = NULL;

    pq.push(start);
    distance_map[global_map_ptr->tag_from_chunk_pos(start->chunk_pos)] = 0; // set start val to 0

    while (!pq.empty())
    {
        auto u = pq.top();
        pq.pop();

        std::vector<Vector3i> adj_vertices = global_map_ptr->get_adjacent_chunks(u->chunk_pos);

        for(auto adj : adj_vertices) {
            
            // only if cunk exists in dm
            if(distance_map.find(global_map_ptr->tag_from_chunk_pos(adj)) == distance_map.end()) {
                continue;
            }

            auto tmp_dist = distance_map[global_map_ptr->tag_from_chunk_pos(adj)];
            auto dist_cur = distance_map[global_map_ptr->tag_from_chunk_pos(u->chunk_pos)];

            if(tmp_dist > dist_cur + 1) {
                distance_map[global_map_ptr->tag_from_chunk_pos(adj)] = dist_cur + 1;
                dijkstra_vertex *new_vertex = new dijkstra_vertex();
                new_vertex->chunk_pos = adj;
                new_vertex->distance = dist_cur + 1;
                new_vertex->previous = u;
                
                pq.push(new_vertex);
            }
        }
    }

    int max = 0;
    Vector3i max_chunk(0, 0, 0);

    for(auto const &data : distance_map) {
        std::cout << "Data: " << data.first << " | " << data.second << std::endl;
        if(data.second > max) {
            max = data.second;
            std::string tmp = data.first;
            max_chunk =  global_map_ptr->chunk_pos_from_tag(tmp);
        }
    }

    std::cout << "Max dist: " << max << " | Max dist chunk: " << std::endl << max_chunk << std::endl;

    // now perform the basic dijkstra algorithm.
}