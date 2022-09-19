/**
 * @file attribute_data_model.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief
 * @version 0.1
 * @date 2022-07-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

class attribute_data_model
{
private:
    int map_resolution;
    float map_size_x;
    float map_size_y;
    float map_size_z;
    float max_distance;
    int max_weight;
    int tau;

public:
    /**
     * @brief Construct a new attribute data model object
     *
     * @param res
     * @param size_x
     * @param size_y
     * @param size_z
     * @param max_dist
     * @param max_w
     * @param t
     */
    attribute_data_model(int res, float size_x, float size_y, float size_z, float max_dist, int max_w, int t);
    
    attribute_data_model() = default;
    ~attribute_data_model() = default;

    inline int get_map_resolution() const
    {
        return map_resolution;
    };

    inline int get_map_size_x() const
    {
        return map_size_x;
    };

    inline int get_map_size_y() const
    {
        return map_size_y;
    };

    inline int get_map_size_z() const
    {
        return map_size_z;
    };

    inline int get_max_distance() const
    {
        return max_distance;
    };

    inline int get_max_weight() const
    {
        return max_weight;
    };

    inline int get_tau() const
    {
        return tau;
    };
};