#pragma once

#include <cmath>
#include <limits>
#include <random>
#include <utility>

/**
 * @brief method used to generate normal distrubuted numbers defined by mu and sigma
 * 
 * @param mu 
 * @param sigma 
 * @return std::pair<double, double> 
 */
static std::pair<double, double> generateGaussianNoise(double mu, double sigma)
{
    constexpr double epsilon = std::numeric_limits<double>::epsilon();
    constexpr double two_pi = 2.0 * M_PI;

    //initialize the random uniform number generator (runif) in a range 0 to 1
    static std::mt19937 rng(std::random_device{}()); // Standard mersenne_twister_engine seeded with rd()
    static std::uniform_real_distribution<> runif(0.0, 1.0);

    //create two random numbers, make sure u1 is greater than epsilon
    double u1, u2;
    do
    {
        u1 = runif(rng);
    }
    while (u1 <= epsilon);
    u2 = runif(rng);

    //compute z0 and z1
    auto mag = sigma * sqrt(-2.0 * log(u1));
    auto z0  = mag * cos(two_pi * u2) + mu;
    auto z1  = mag * sin(two_pi * u2) + mu;

    return std::make_pair(z0, z1);
}

/**
 * @brief Generates a uniform distributed number between lower_boundary and upper_boundary
 * 
 * @param lower_boundary 
 * @param upper_boundary 
 * @return double 
 */
static double generateRandomNumber(double lower_boundary, double upper_boundary)
{
    //initialize the random uniform number generator (runif) in a range 0 to 1
    static std::mt19937 rng(std::random_device{}()); // Standard mersenne_twister_engine seeded with rd()
    static std::uniform_real_distribution<> runif(lower_boundary, upper_boundary);

    return runif(rng);
}