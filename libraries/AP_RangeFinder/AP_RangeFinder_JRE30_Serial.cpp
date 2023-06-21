/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_JRE30_Serial.h"

#if AP_RANGEFINDER_JRE30_SERIAL_ENABLED

#include <GCS_MAVLink/GCS.h>

#define FRAME_HEADER_1   0x52  // 'R'
#define FRAME_HEADER_2   0x41  // 'A'

#define CCITT_INITPARA   0xffff
#define CCITT_POLYNOMIAL 0x8408

/* CRC16CCITT */
static uint16_t CalcCRC16CCITT(uint8_t *cbuffer, uint16_t csize)
{
	uint16_t crc = CCITT_INITPARA;
	for (int i = 0; i < csize; i++) {
		crc ^= *cbuffer++;
		for (int j = 0; j < 8; j++) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= CCITT_POLYNOMIAL;
            } else {
                crc >>= 1;
            }
		}
	}
	crc = crc ^ 0xffff;
	return crc;
}

void AP_RangeFinder_JRE30_Serial::clear_data_buff()
{
    data_buff_idx = 0;
    for (uint16_t i=0; i<ARRAY_SIZE(data_buff); i++) {
        data_buff[i] = 0;
    }
}

bool AP_RangeFinder_JRE30_Serial::get_reading(uint16_t &reading_cm)
{
    // uart instance check
    if (uart == nullptr) {
        return false;  // not update
    }

    // buffer read
    const uint8_t num_read = uart->read(read_buff, ARRAY_SIZE(read_buff));

    // OUTPUT RAW DATA 32BYTES
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "------------------- : %d", num_read);
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "1: %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x", 
    //read_buff[0], read_buff[1], read_buff[2], read_buff[3], read_buff[4], read_buff[5], read_buff[6], read_buff[7], 
    //read_buff[8], read_buff[9], read_buff[10], read_buff[11], read_buff[12], read_buff[13], read_buff[14], read_buff[15]
    //);
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "2: %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x", 
    //read_buff[16], read_buff[17], read_buff[18], read_buff[19], read_buff[20], read_buff[21], read_buff[22], read_buff[23], 
    //read_buff[24], read_buff[25], read_buff[26], read_buff[27], read_buff[28], read_buff[29], read_buff[30], read_buff[31]
    //);

    read_buff_idx = 0; // read_buff start data index
    while (read_buff_idx < num_read) {
        // header data 1 not set
        if (data_buff[0] != FRAME_HEADER_1) {
            for (; read_buff_idx<num_read; read_buff_idx++) {
                // header data found judge
                if (read_buff[read_buff_idx] == FRAME_HEADER_1) {
                    data_buff[data_buff_idx++] = read_buff[read_buff_idx];
                    read_buff_idx++; // next read_buff
                    break;
                }
            }
            // header data not found. next frame.
            if (read_buff_idx == num_read) {
                return false;
            }
        }

        // header data 2 not set
        if (data_buff[1] != FRAME_HEADER_2) {  // header data 2 not set
            for (; read_buff_idx<num_read; read_buff_idx++) {
                // header data found judge
                if (read_buff[read_buff_idx] == FRAME_HEADER_2) {
                    data_buff[data_buff_idx++] = read_buff[read_buff_idx];
                    read_buff_idx++; // next read_buff
                    break;
                }
            }
            // header data not found. next frame.
            if (read_buff_idx == num_read) {
                return false;
            }
        }

        // data set
        data_buff[data_buff_idx++] = read_buff[read_buff_idx++];

        // crc check
        if (data_buff_idx >= ARRAY_SIZE(data_buff)) {  // 1 data set complete
            uint16_t crc = CalcCRC16CCITT(data_buff, ARRAY_SIZE(data_buff) - 2);
            if (((crc>>8 & 0xff) == data_buff[15]) && ((crc & 0xff) == data_buff[14])) {
                //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x", 
                //data_buff[0], data_buff[1], data_buff[2], data_buff[3], data_buff[4], data_buff[5], data_buff[6], data_buff[7], 
                //data_buff[8], data_buff[9], data_buff[10], data_buff[11], data_buff[12], data_buff[13], data_buff[14], data_buff[15]
                //);
                // UPDATE DATA
                reading_cm = (data_buff[4] * 256 + data_buff[5]);
                state.distance_cm = reading_cm;
                state.last_reading_ms = AP_HAL::millis();
                update_status();
                // data clear
                clear_data_buff();
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, ">>> CRC ERROR");
            }
        } 
    }

    return false;
}

bool AP_RangeFinder_JRE30_Serial::set_writeing(uint16_t cmd, float param1, float param2)
{
    if ((int8_t)param1 != 1) {
        return false;
    }

    switch (cmd) {
    case MAV_CMD_ZEROSET_SET_JRE30:
        uart->write('z');
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MAV_CMD_ZEROSET_SET_JRE30 : ");
        break;

    case MAV_CMD_ZEROSET_CLR_JRE30:
        uart->write('0');
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MAV_CMD_ZEROSET_CLR_JRE30 : ");
        break;

    case MAV_CMD_GAINSET_JRE30:
        uart->write('O');
        uart->write(param2);
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MAV_CMD_GAINSET_JRE30 : %d", (int8_t)param2);
        break;

    default:
        break;
    }

    return true;
}
#endif  // AP_RANGEFINDER_JRE30_SERIAL_ENABLED
