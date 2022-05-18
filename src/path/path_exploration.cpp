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

/**
 * @brief
 * TODO: a path needs to be tracable here...
 *
 */
void path_exploration::dijsktra()
{
    // before doing anything, we need to fill the prio queue :)
    // std::cout << "Filling pq" << std::endl;
    // fill_pq();
    int num_chunks = global_map_ptr->num_chunks();

    // distance map storing the current distance and the previous Vector (shortest path)
    std::map<std::string, std::pair<Vector3i, float>> distance_map;

    auto chunks = global_map_ptr->all_chunk_poses(local_map_ptr->get_size());
    // auto chunks = global_map_ptr->all_chunk_poses();

    std::vector<float> hit_miss_percentages;
    int max_index = 0;

    // checkout hit miss percentages for weighting
    for (auto chunk : chunks)
    {
        Vector3f tmp = chunk.cast<float>() * CHUNK_SIZE * (MAP_RESOLUTION / 1000.0f);
        Pose *pose = new Pose();
        pose->pos = tmp;
        pose->quat = Quaternionf::Identity();
        ray_tracer->update_pose(pose);
        float ret = ray_tracer->start(1);
        hit_miss_percentages.push_back(ret);

        // update max index
        if (ret > hit_miss_percentages[max_index])
        {
            max_index = hit_miss_percentages.size() - 1;
        }

        std::cout << "Hit/Not Percentage in path exploration: " << ret << std::endl;
    }

    std::cout << "Max value: " << hit_miss_percentages[max_index] << std::endl;

    for (float &val : hit_miss_percentages)
    {
        val = hit_miss_percentages[max_index] / val;
    }

    for (float &val : hit_miss_percentages)
    {
        std::cout << "Value in hit miss: " << val << std::endl;
    }

    // default previous Vector used to backtrack path has just max ints as values.
    Vector3i default_prev(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max());

    // fill distance map, no previous poses set
    for (auto chunk : chunks)
    {
        distance_map[global_map_ptr->tag_from_chunk_pos(chunk)] = std::make_pair(default_prev, std::numeric_limits<float>::max());
    }

    std::cout << "Chunks in the dm: " << distance_map.size() << std::endl;

    // initial vertex
    dijkstra_vertex *start = new dijkstra_vertex();

    pq.push(start);
    distance_map[global_map_ptr->tag_from_chunk_pos(start->chunk_pos)] = std::make_pair(Vector3i(0, 0, 0), 0.0f); // set start val to 0

    while (!pq.empty())
    {
        // get top value and pop it afterwards.
        auto u = pq.top();
        pq.pop();

        std::vector<Vector3i> adj_vertices = global_map_ptr->get_adjacent_chunks(u->chunk_pos);

        for (auto adj : adj_vertices)
        {
            // only if cunk exists in dm

            auto map_idx = distance_map.find(global_map_ptr->tag_from_chunk_pos(adj));

            if (map_idx == distance_map.end())
            {
                continue;
            }

            std::ptrdiff_t index = std::distance(distance_map.begin(), map_idx);

            // get distance values of current and adjacent chunk
            auto tmp_dist = distance_map[global_map_ptr->tag_from_chunk_pos(adj)].second;
            auto dist_cur = distance_map[global_map_ptr->tag_from_chunk_pos(u->chunk_pos)].second;

            // if the updated dist is smaller than the underlying one, we update it. + 1 distance is default
            if (tmp_dist > dist_cur + hit_miss_percentages[index])
            {
                distance_map[global_map_ptr->tag_from_chunk_pos(adj)].second = dist_cur + 1;
                distance_map[global_map_ptr->tag_from_chunk_pos(adj)].first = u->chunk_pos;
                dijkstra_vertex *new_vertex = new dijkstra_vertex(adj, dist_cur + 1);

                pq.push(new_vertex);
            }
        }
    }

    // find max valued chunk in distance map (furthest away)
    int max = 0;
    Vector3i max_chunk(0, 0, 0);

    for (auto const &data : distance_map)
    {
        // std::cout << "Data: " << data.first << " | " << data.second.second << std::endl;
        if (data.second.second > max)
        {
            max = data.second.second;
            std::string tmp = data.first;
            max_chunk = global_map_ptr->chunk_pos_from_tag(tmp);
        }
    }

    // extract path from max valued chunk to (0, 0, 0)
    std::cout << "[PathExploration] Start Path extraction" << std::endl;
    std::vector<Vector3f> path_arr;
    Vector3i current = max_chunk;
    Vector3i previous;

    // backtrack the path from furhtest chunk to start chunk (0, 0, 0)
    while (current != Vector3i(0, 0, 0))
    {
        previous = distance_map[global_map_ptr->tag_from_chunk_pos(current)].first;

        path_arr.push_back(current.cast<float>() * CHUNK_SIZE * (MAP_RESOLUTION / 1000.0f));

        current = previous;
    }

    // finally: the start pos
    path_arr.push_back(previous.cast<float>() * CHUNK_SIZE * (MAP_RESOLUTION / 1000.0f));

    // reverse the vector to make it a usable path
    std::reverse(path_arr.begin(), path_arr.end());

    estimate_more_poses(path_arr, 5);

    for (auto pose : path_arr)
    {
        Pose temp;
        temp.pos = pose;
        temp.quat = Eigen::Quaternionf::Identity();

        path->add_pose(temp);
    }

    std::cout << "Max dist: " << max << " | Max dist chunk: " << std::endl
              << max_chunk << std::endl;
}

void path_exploration::estimate_more_poses(std::vector<Vector3f> &path, int estimates)
{
    for (int i = 0; i < path.size() - 1; i += (estimates + 1))
    {
        auto current = path[i];
        auto next = path[i + 1];
        
        Vector3f diff = (next - current) / (estimates + 1);

        for (int j = 1; j <= estimates; j++)
        {
            Vector3f new_pose = current + j * diff;
            path.insert(path.begin() + i + j, new_pose);
        }
    }
}