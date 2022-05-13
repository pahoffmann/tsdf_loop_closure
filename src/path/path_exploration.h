#include "path.h"
#include "../map/local_map.h"
#include "../map/global_map.h"
#include "../ray_tracer/ray_tracer.h"

#include <queue>
#include <algorithm>

class path_exploration
{
private:
    Path *path;
    std::shared_ptr<GlobalMap> global_map_ptr;
    std::shared_ptr<LocalMap> local_map_ptr;
    RayTracer *ray_tracer;

    // struct used to store 
    struct dijkstra_vertex
    {
        Vector3i chunk_pos;
        float distance = std::numeric_limits<float>::infinity();

        dijkstra_vertex() {
            chunk_pos = Vector3i(0,0,0);
        }

        dijkstra_vertex(Vector3i chunk_pos_, float distance_) {
            chunk_pos = chunk_pos_;
            distance = distance_;
        }
    };

    // used to compare two dijkstra vertices, for the min priority queue
    struct compare : public std::binary_function<dijkstra_vertex*, dijkstra_vertex*, bool>  
    {
        bool operator()(const dijkstra_vertex *l, const dijkstra_vertex *r)
        {
            return l->distance < r->distance;
        }
    };

    // priority queue used for dijkstra
    std::priority_queue<dijkstra_vertex*, std::vector<dijkstra_vertex*>, compare> pq;


    /**
     * @brief Creates a string hash from a 3d integer point
     *
     * @return std::string
     */
    std::string hash_from_point(Eigen::Vector3i);

public:
    path_exploration(std::shared_ptr<LocalMap> l_map_ptr, std::shared_ptr<GlobalMap> g_map_ptr, Path *path_ptr, RayTracer *tracer_ptr);
    ~path_exploration();

    /**
     * @brief This function does some basic statistics to explore a possible path for the current globalmap, when there is no path given
     *        Used for debugging purposes, when there is no path, to ensure that the association strategies actually work
     *        Uses a very greedy method, which places random poses inside the boundaries of the global map and checks, if
     *
     * @TODO: implement this maybe...
     * @param num_randoms
     * @param max_num_selects
     * @return int
     */
    int explore(int num_randoms, int max_num_selects);

    /**
     * @brief Performs a dijkstra operation to find the (longest shortest) path
     * 
     */
    void dijsktra();
};
