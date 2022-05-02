#include "path_exploration.h"

path_exploration::path_exploration(std::shared_ptr<LocalMap> l_map_ptr, Path *path_ptr, RayTracer *tracer_ptr)
{
    ray_tracer = tracer_ptr;
    local_map_ptr = l_map_ptr;
    path = path_ptr;
}

path_exploration::~path_exploration()
{
}

int path_exploration::explore(int num_randoms, int max_num_selects)
{
    // global map always begins at (0,0,0)
    Eigen::Vector3i start_pos(0, 0, 0);

    // the previous pos of the first pos is the pos itself, recursion anchor
    distance_map[hash_from_point(start_pos)] = std::make_pair(0, start_pos);

    explore_recursive(start_pos, 0);
    return 0;
}

//FIXME: this does not work like this, there needs to be another way to do this
// shortest longest path, waht a mindfuck
Vector3i path_exploration::explore_recursive(Vector3i new_pos, int cnt = 0)
{

    // recursion anchor, if the local map is not fully occupied, e.g. some chunks don't exist
    if (!local_map_ptr->is_full_occupied(new_pos))
    {
        return new_pos;
    }

    int cnt1, cnt2, cnt3, cnt4, cnt5, cnt6, cnt7, cnt8, cnt9, cnt10;
    cnt1 = cnt2 = cnt3 = cnt4 = cnt5 = cnt6 = cnt7 = cnt8 = cnt9 = cnt10 = cnt;

    // iterate through 3d space, using manhattan distance

    // oben
    Vector3i above(0, 0, 1);

    // unten
    Vector3i below(0, 0, -1);

    // links
    Vector3i left(-1, 0, 0);

    // rechts
    Vector3i right(1, 0, 0);

    // vorne
    Vector3i front(0, 1, 0);

    // hinten
    Vector3i back(0, -1, 0);
}

std::string path_exploration::hash_from_point(Eigen::Vector3i pos)
{
    return "(" + std::to_string(pos.x()) + ")-(" + std::to_string(pos.y()) + ")-(" + std::to_string(pos.z()) + ")";
}