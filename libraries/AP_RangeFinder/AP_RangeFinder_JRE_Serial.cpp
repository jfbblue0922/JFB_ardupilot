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

#include "AP_RangeFinder_JRE_Serial.h"

#if AP_RANGEFINDER_JRE_SERIAL_ENABLED

#include <GCS_MAVLink/GCS.h>

#define FRAME_HEADER_1   0x52  // 'R'
#define FRAME_HEADER_2   0x41  // 'A'

#define DIST_MAX_CM 50000

#define CCITT_INITPARA   0xffff
#define CCITT_POLYNOMIAL 0x8408
#define CCITT_OUTPARA    0xffff

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
    crc = crc ^ CCITT_OUTPARA;
    return crc;
}

bool AP_RangeFinder_JRE_Serial::get_reading(float &reading_m)
{
    // uart instance check
    if (uart == nullptr) {
        return false;  // not update
    }
    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }

    uint16_t count = 0;
    float reading_cm;
    // max distance the sensor can reliably measure - read from parameters
    const int16_t distance_cm_max = max_distance_cm();

    // buffer read
    const ssize_t num_read = uart->read(read_buff, ARRAY_SIZE(read_buff));
    read_buff_idx = 0; // read_buff start data index
    while (read_buff_idx < num_read) {

        if (data_buff_idx == 0) { // header first byte check
            // header data search
            for (; read_buff_idx<num_read; read_buff_idx++) {
                if (read_buff[read_buff_idx] == FRAME_HEADER_1) {
                    data_buff[0] = FRAME_HEADER_1;
                    data_buff_idx = 1; // next data_buff
                    if (read_buff_idx >= num_read - 1) { // read end byte
                        if (count > 0) {
                            return true;
                        }
                        return false;      // next packet to header second byte
                    } else {
                        if (read_buff[read_buff_idx + 1] == FRAME_HEADER_2) {
                            data_buff[1] = FRAME_HEADER_2;
                            data_buff_idx = 2;    // next data_buff
                            read_buff_idx += 2;   // next read_buff
                            break;         // next data set
                        } else {
                            data_buff_idx = 0;    // data index clear
                        }
                    }
                }
            }

        } else if (data_buff_idx == 1) { // header second byte check
            if (read_buff[read_buff_idx] == FRAME_HEADER_2) {
                data_buff[1] = FRAME_HEADER_2;
                data_buff_idx = 2;    // next data_buff
            } else {
                data_buff_idx = 0;    // data index clear
            }
            read_buff_idx++;      // next read_buff

        } else { // data set
            if (data_buff_idx >= ARRAY_SIZE(data_buff)) {  // 1 data set complete
                // crc check
                uint16_t crc = CalcCRC16CCITT(data_buff, ARRAY_SIZE(data_buff) - 2);
                if ((((crc>>8) & 0xff) == data_buff[15]) && ((crc & 0xff) == data_buff[14])) {
                    // status check
                    if (data_buff[13] & 0x02) { // NTRK
                        no_signal = true;
                        reading_m = MIN(MAX(DIST_MAX_CM, distance_cm_max), UINT16_MAX) * 0.01f;
                    } else { // UPDATE DATA
                        no_signal = false;
                        reading_cm = data_buff[4] * 256 + data_buff[5];
                        if (reading_cm < distance_cm_max) {
                            reading_m = reading_cm * 0.01f;
                        } else {
                            reading_m = distance_cm_max * 0.01f;
                        }
                    }
                    count++;
                }
                data_buff_idx = 0; // data index clear
            } else {
                data_buff[data_buff_idx++] = read_buff[read_buff_idx++];
            }
        }
    }

    if (count > 0) {
        return true;
    }
    return false;
}
#endif  // AP_RANGEFINDER_JRE_SERIAL_ENABLED
