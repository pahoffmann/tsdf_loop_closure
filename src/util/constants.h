#pragma once

/**
 * @file constants.h
 * @author Malte Hillmann
 */

#include "hdf5_constants.h"

/// Resolution of the Map in Millimeter per Cell.
constexpr int MAP_RESOLUTION = 64;

/// Resolution of the Weights. A weight of 1.0f is represented as WEIGHT_RESOLUTION
constexpr int WEIGHT_RESOLUTION = 32;

/**
 * @brief Resolution of calculations (Matrices, division, ...)
 *
 * This Value is the equivalent of 1.0f, and its size determines the accuracy of anything
 * after the decimal point.
 */
constexpr int MATRIX_RESOLUTION = 1 << 15; // == pow(2, 15)

/**
 * @brief The number of times that the tsdf kernel runs in parallel
 * 
 * NOTE: Changing this number requires adapting all places marked with
 * // MARKER: TSDF SPLIT
 */
constexpr int TSDF_SPLIT_FACTOR = 4;

constexpr int CHUNK_SHIFT = 6;

/// Side length of the cube-shaped chunks (2^CHUNK_SHIFT).
constexpr int CHUNK_SIZE = 1 << CHUNK_SHIFT;

// TODO: the following constants need to be evaluated based on whether they are correct
//       might be better saved in the global map hdf5

constexpr int TSDF_BASE_VALUE = 600;

constexpr int TSDF_BASE_WEIGHT = 0;
