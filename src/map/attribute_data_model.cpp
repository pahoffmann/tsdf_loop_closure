#include <loop_closure/map/attribute_data_model.h>

attribute_data_model::attribute_data_model(int res, float size_x, float size_y, float size_z, float max_dist, int max_w, int t)
    : map_resolution(res), map_size_x(size_x), map_size_y(size_y), map_size_z(size_z), max_distance(max_dist), max_weight(max_w), tau(t)

{
    // do something
}
