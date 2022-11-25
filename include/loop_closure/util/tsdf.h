#pragma once

/**
 * @file tsdf.h
 * @author Marcel Flottmann
 * @author Patrick Hoffmann
 * @date 2021-01-13
 */

#include <loop_closure/util/tsdf_hw.h>

/**
 * @brief Interface for a TSDF entry in hardware.
 *        Entry can accessed via the complete datatype or the hardware representation,
 *        which constists of a value and a weight
 */
class TSDFEntry
{
public:
    using RawType = uint64_t;
    using ValueType = TSDFEntryHW::ValueType;
    using WeightType = TSDFEntryHW::WeightType;
    using IntersectionType = TSDFEntryHW::IntersectionType;
    using IndexType = TSDFEntryHW::IntersectionType;

    // currently used to store information on the type of intersection
    enum IntersectStatus
    {
        NO_INT,  // 0
        INT,     // 1
        INT_NEG, // 2
        INT_ZERO // 3
    };

private:
    /// Internally managed datatype
    union
    {
        RawType raw;
        TSDFEntryHW tsdf;
    } data;
    static_assert(sizeof(RawType) == sizeof(TSDFEntryHW)); // raw and TSDF types must be of the same size

public:
    /**
     * @brief Initialize the entry through a weight and a value
     */
    TSDFEntry(ValueType value, WeightType weight, IndexType pose_index = -1, IntersectionType int_type = 0)
    {
        data.tsdf.value = value;
        data.tsdf.weight = weight;
        data.tsdf.intersection = int_type;
        data.tsdf.pose_index = pose_index;
    }

    /**
     * @brief Initialize the entry through the raw data
     */
    explicit TSDFEntry(RawType raw)
    {
        data.raw = raw;
    }

    // Default behaviour for copy and move

    TSDFEntry() = default;
    TSDFEntry(const TSDFEntry &) = default;
    TSDFEntry(TSDFEntry &&) = default;
    ~TSDFEntry() = default;
    TSDFEntry &operator=(const TSDFEntry &) = default;
    TSDFEntry &operator=(TSDFEntry &&) = default;

    /**
     * @brief Are the two TSDF entries equal?
     */
    bool operator==(const TSDFEntry &rhs) const
    {
        return raw() == rhs.raw();
    }

    /**
     * @brief Get the complete data as raw type
     */
    RawType raw() const
    {
        return data.raw;
    }

    /**
     * @brief Set the TSDF entry through the raw type
     */
    void raw(RawType val)
    {
        data.raw = val;
    }

    /**
     * @brief Get the value from the TSDF entry
     */
    ValueType value() const
    {
        return data.tsdf.value;
    }

    /**
     * @brief Set th value of the TSDF entry
     */
    void value(ValueType val)
    {
        data.tsdf.value = val;
    }

    /**
     * @brief Get the weight of the TSDF entry
     */
    WeightType weight() const
    {
        return data.tsdf.weight;
    }

    /**
     * @brief Set the weight of the TSDF entry
     */
    void weight(WeightType val)
    {
        data.tsdf.weight = val;
    }

    /**
     * @brief get the pose index of the TSDF Entry
     * 
     * @return IndexType 
     */
    IndexType pose_index() const
    {
        return data.tsdf.pose_index;
    }

    /**
     * @brief Set the pose index of the TSDF entry
     */
    void pose_index(IndexType val)
    {
        data.tsdf.pose_index = val;
    }

    /**
     * @brief returns the intersect of the current tsdf entry
     * 
     * @return IntersectionType 
     */
    IntersectionType intersect() const
    {
        return data.tsdf.intersection;
    }

    /**
     * @brief set the intersect of the current tsdf entry
     * 
     * @param int_type 
     */
    void intersect(IntersectionType int_type)
    {
        data.tsdf.intersection = int_type;
    }
};

// static_assert(sizeof(TSDFEntry) == sizeof(TSDFEntryHW));          // HW and SW types must be of the same size
