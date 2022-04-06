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
    int side_length_xy;
    int side_length_z;
    int opening_degree;
    double step_size;
    double ray_size;
    std::string map_file_name;
    std::string poses_file_name;
    std::string base_file_name;

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
};
