#pragma once

/**
 * @file algorithm.h
 * @author Patrick Hoffmann
 */

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>

/**
 * @brief compare function used by the sort_vector method, currently only supposed to be used for hdf5 tag comparison
 * 
 * @param s1 
 * @param s2 
 * @return true 
 * @return false 
 */
static bool compare_int_strings(std::string s1, std::string s2) {
    if(s1.rfind("/", 0) == 0) {
        std::cout << "Before: " << s1 << std::endl;
        s1 = s1.substr(1);

        std::cout << "After: " << s1 << std::endl;
    }
    else if(s2.rfind("/", 0) == 0) {
        std::cout << "Before: " << s2 << std::endl;
        s2 = s2.substr(1);

        std::cout << "After: " << s2 << std::endl;
    }

    return std::stoi(s1) < std::stoi(s2);
}

/**
 * @brief sorts a vector of string values, if is_integer_values is set to true, it assumes, that there are integer strings
 *        method is used to sort vectors of hdf5 tags, which usally also contain a leading "/", therefore this method should only be used
 *        for this use case at the moment, though - should be working for other use cases as well
 * 
 * @param data 
 * @param is_integer_values 
 */
static void sort_vector(std::vector<std::string> &data, bool is_integer_values = true)
{
    if(is_integer_values) {
        // sort with compare func
        std::sort(data.begin(), data.end(), compare_int_strings);
    }
    else 
    {
        // basic sorting
        std::sort(data.begin(), data.end());
    }
}