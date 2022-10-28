#pragma once

/**
 * @file tsdf_hw.h
 * @author Marcel Flottmann
 * @date 2021-01-14
 */

#include <cinttypes>
#include <loop_closure/util/constants.h>

/**
 * @brief Hardware representation of a TSDF entry in the map. Consists of a TSDF value and a current weight for the update procedure
 * 
 */
struct TSDFEntryHW
{
    using ValueType = int16_t;
    using WeightType = int16_t;
    using IndexType = int16_t;
    using IntersectionType = int16_t;

    ValueType value;
    WeightType weight;
    IndexType pose_index;
    IntersectionType intersection;
};
