#include "path.h"
#include "../map/local_map.h"
#include "../map/global_map.h"
#include "../ray_tracer/ray_tracer.h"

class path_exploration
{
private:
    Path *path;
    std::shared_ptr<GlobalMap> global_map_ptr;
    std::shared_ptr<LocalMap> local_map_ptr;
    RayTracer *ray_tracer;

    // to keep track of the minimum distances needed to go to chunk xyz
    std::map<std::string, std::pair<int, Eigen::Vector3i>> distance_map;

    /**
     * @brief used to recursively explore the 3d space
     *
     * @param distance_map used to check, if the current path is any good.
     * @param cnt used to track the number of positions
     * @return Vector3i
     */
    Vector3i explore_recursive(Vector3i new_pos, int cnt = 0);

    /**
     * @brief Creates a string hash from a 3d integer point
     * 
     * @return std::string 
     */
    std::string hash_from_point(Eigen::Vector3i);

public:
    path_exploration(std::shared_ptr<LocalMap> l_map_ptr, Path *path_ptr, RayTracer *tracer_ptr);
    ~path_exploration();

    /**
     * @brief This function does some basic statistics to explore a possible path for the current globalmap, when there is no path given
     *        Used for debugging purposes, when there is no path, to ensure that the association strategies actually work
     *        Uses a very greedy method, which places random poses inside the boundaries of the global map and checks, if
     *
     * @param num_randoms
     * @param max_num_selects
     * @return int
     */
    int explore(int num_randoms, int max_num_selects);
};
