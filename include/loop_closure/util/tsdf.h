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
    using RawType = uint32_t;
    using ValueType = TSDFEntryHW::ValueType;
    using WeightType = TSDFEntryHW::WeightType;

        // currently used to store information on the type of intersection
    enum IntersectStatus {
        NO_INT,
        INT,
        INT_NEG,
        INT_ZERO
    };

private:


    /// Internally managed datatype
    union
    {
        RawType raw;
        TSDFEntryHW tsdf;
    } data;
    static_assert(sizeof(RawType) == sizeof(TSDFEntryHW));            // raw and TSDF types must be of the same size

    IntersectStatus intersect_data;

public:

    /**
     * @brief Initialize the entry through a weight and a value
     */
    TSDFEntry(ValueType value, WeightType weight)
    {
        data.tsdf.value = value;
        data.tsdf.weight = weight;
        intersect_data = IntersectStatus::NO_INT;
    }

    /**
     * @brief Initialize the entry through the raw data
     */
    explicit TSDFEntry(RawType raw)
    {
        data.raw = raw;
        intersect_data = IntersectStatus::NO_INT;
    }

    // Default behaviour for copy and move

    TSDFEntry() = default;
    TSDFEntry(const TSDFEntry&) = default;
    TSDFEntry(TSDFEntry&&) = default;
    ~TSDFEntry() = default;
    TSDFEntry& operator=(const TSDFEntry&) = default;
    TSDFEntry& operator=(TSDFEntry&&) = default;

    /**
     * @brief Are the two TSDF entries equal? 
     */
    bool operator==(const TSDFEntry& rhs) const
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

    IntersectStatus getIntersect() const
    {
        return intersect_data;
    }

    void setIntersect(IntersectStatus val)
    {
        intersect_data = val;
    }
};

//static_assert(sizeof(TSDFEntry) == sizeof(TSDFEntryHW));          // HW and SW types must be of the same size

