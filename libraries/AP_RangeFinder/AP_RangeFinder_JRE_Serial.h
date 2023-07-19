#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#ifndef AP_RANGEFINDER_JRE_SERIAL_ENABLED
#define AP_RANGEFINDER_JRE_SERIAL_ENABLED 1
#endif

#define AP_RANGEFINDER_JRE_SERIAL_COMMAND_ENABLED
#undef AP_RANGEFINDER_JRE_SERIAL_COMMAND_ENABLED

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

    // baudrate used during object construction:
    uint32_t initial_baudrate(uint8_t serial_instance) const override
    {
        return 460800;
    }

#ifdef AP_RANGEFINDER_JRE_SERIAL_COMMAND_ENABLED
    bool set_writeing(uint16_t cmd, float param1, float param2) override;
#endif

private:

    // get a reading
    bool get_reading(uint16_t &reading_cm) override;

    uint8_t read_num;
    uint8_t read_buff[DATA_LENGTH * 8];
    uint8_t read_buff_idx;
    uint8_t data_buff[DATA_LENGTH];
    uint8_t data_buff_idx;

    bool no_signal;
};
#endif  // AP_RANGEFINDER_JRE_SERIAL_ENABLED
