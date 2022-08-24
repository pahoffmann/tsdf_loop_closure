#pragma once

/**
 * @file options_reader.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief This class functions as a config wrapper for the loop closure node, using boost program options
 * @version 0.1
 * @date 2022-04-06
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <vector>
#include <string>
#include <boost/program_options.hpp>
#include <iostream>
#include <sstream>

namespace po = boost::program_options;

class lc_options_reader
{
private:
    std::string _cfg_filename = "";
    po::options_description *desc;
    po::variables_map *vm;

    // option values
    int hor_res;
    int vert_res;
    int opening_degree;
    double step_size;
    double ray_size;
    std::string map_file_name;
    std::string poses_file_name;
    std::string base_file_name;
    int path_method;

    void create_options();

    void print_options();

public:
    lc_options_reader();
    ~lc_options_reader();

    /**
     * @brief will read the options from cmdline, store them in appropriate objects from boost
     *
     * @param argc
     * @param argv
     * @return int   status, which indicates if everything worked as proposed
     *               [0] - OK, proceed
     *               [1] - ERROR
     *               [2] - HELP Indicator
     */
    int read_options(int argc, char **argv);

    // TODO: write getters which wrap around the boost program options and make the config-values accessible in an easy fashion

    
    /**
     * @brief Get the hor res object
     * 
     * @return int 
     */
    inline int get_hor_res() {
        return hor_res;
    }
    
    /**
     * @brief Get the vert res object
     * 
     * @return int 
     */
    inline int get_vert_res() {
        return vert_res;
    }
    
    /**
     * @brief Get the opening degree object
     * 
     * @return int 
     */
    inline int get_opening_degree() {
        return opening_degree;
    }
    
    /**
     * @brief Get the step size object
     * 
     * @return double 
     */
    inline double get_step_size() {
        return step_size;
    }
    
    /**
     * @brief Get the ray size object
     * 
     * @return double 
     */
    inline double get_ray_size() {
        return ray_size;
    }
    
    /**
     * @brief Get the map file name object
     * 
     * @return std::string 
     */
    inline std::string get_map_file_name() {
        return map_file_name;
    }
    
    /**
     * @brief Get the poses file name object
     * 
     * @return std::string 
     */
    inline std::string get_poses_file_name() {
        return poses_file_name;
    }
    
    /**
     * @brief Get the base file name object
     * 
     * @return std::string 
     */
    inline std::string get_base_file_name() {
        return base_file_name;
    }

    inline int get_path_method() {
        return path_method;
    }
};
