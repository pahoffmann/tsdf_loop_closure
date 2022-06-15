#include "path.h"

/**
 * @brief constructor, currently empty
 *
 */
Path::Path(RayTracer *tracer)
{
    ray_tracer = tracer;
}

void Path::fromJSON(std::string filename)
{
    PATH::json_to_path(filename, poses);
}

/**
 * @todo TODO: test this code piece
 */
std::pair<int, int> Path::find_loop_greedy(int start_idx, float max_dist, float min_traveled)
{
    // error identification
    if (start_idx > get_length())
    {
        return std::make_pair(-1, -1);
    }

    float dist_traveled = 0.0f;

    // a vector storing the distance from the start pose to the pose with index (i - start_idx)
    std::vector<float> distance_vec(get_length());

    float dist = 0.0f;

    Pose *current = at(start_idx);
    Pose *next;

    for (int i = start_idx; i < get_length(); i++)
    {
        next = at(i);

        float dist_cur_next = (next->pos - current->pos).norm();

        std::cout << "distance between " << *current << " and " << *next << ": " << dist_cur_next << std::endl;

        if (i > 0)
        {
            distance_vec[i - start_idx] = distance_vec[(i - start_idx) - 1] + dist_cur_next;
        }
        else
        {
            distance_vec[i - start_idx] = dist_cur_next;
        }

        current = next;

        std::cout << "Distance [" << i << "] :" << distance_vec[i - start_idx] << std::endl;
    }

    // now that we have calculated the distances, lezgo

    // store the indices of the loop closure candidates
    int index_i = -1;
    int index_j = -1;

    for (int i = start_idx; i < get_length(); i++)
    {
        for (int j = i + 1; j < get_length(); j++)
        {
            float distance = distance_vec[j] - distance_vec[i];

            std::cout << "Current distance in loop closure detection" << distance << std::endl;

            // not enough distance traveled
            if (distance < min_traveled)
            {
                std::cout << "[Path - Find loop (greedy)] skipping, because distance travled is not enough" << std::endl;
                continue;
            }
            // calc the direct "air distance" between the two candidates
            float direct_distance = (at(j)->pos - at(i)->pos).norm();

            std::cout << "Direct dist: " << direct_distance << std::endl;

            // if this distance is bigger than the one specified, we also continue with the next candidates
            if (direct_distance > max_dist)
            {
                continue;
            }

            // if we reach this point we have (as it seems) a candidate. now we need to check the visibility criteria (e.g. : is pos i visible from pos j ?)
            // this should usually be the case.

            if (!ray_tracer->is_visible(*(at(i)), *(at(j))))
            {
                continue;
            }
            else
            {
                index_i = i;
                index_j = j;
            }
        }
    }

    return std::make_pair(index_i, index_j);
}