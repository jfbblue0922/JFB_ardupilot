#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#ifndef AP_RANGEFINDER_JRE_SERIAL_ENABLED
#define AP_RANGEFINDER_JRE_SERIAL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#if AP_RANGEFINDER_JRE_SERIAL_ENABLED

#define DATA_LENGTH 16

class AP_RangeFinder_JRE_Serial : public AP_RangeFinder_Backend_Serial
{

public:
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return new AP_RangeFinder_JRE_Serial(_state, _params);
    }

protected:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
    {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    bool get_signal_quality_pct(uint8_t &quality_pct) const override
    {
        quality_pct = no_signal ? 0 : 100;
        return true;
    }

private:

    // get a reading
    bool get_reading(float &reading_m) override;

    uint8_t read_num;
    uint8_t read_buff[DATA_LENGTH * 8];
    uint8_t read_buff_idx;
    uint8_t data_buff[DATA_LENGTH];
    uint8_t data_buff_idx;

    bool no_signal;
};
#endif  // AP_RANGEFINDER_JRE_SERIAL_ENABLED
