// Skyways
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
/*
Driver for reading UART data from connected Anello EVK
*/

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_AnelloEVK.h"

#if HAL_EXTERNAL_AHRS_ENABLED

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

/**
 * @brief Header struct used for all Anello packets
 *
 * see:https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#binary-data-output-messages
 */
struct PACKED Anello_pkt_header {
    uint8_t preamble;
    uint16_t length;
    uint16_t msg_type;
};

/**
 * @brief Stuct containing fields and respective lengths for an IMU message.
 *
 * see:
 * https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#imu
 */
struct PACKED IMU_pkt {
    Anello_pkt_header header {0xD3, 0x01D0, 0xFDA1};
    uint64_t mcu_time;
    int64_t odr_time;
    int32_t accel[3];
    int32_t gyro[3];
    int32_t fog_z;
    int16_t odr;
    int16_t temp;
    int8_t crc[3];

} imu_packet;

int8_t imu_pkt_length = sizeof(imu_packet);

AP_ExternalAHRS_AnelloEVK::AP_ExternalAHRS_AnelloEVK(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
        return;
    }

    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    pkt_buf = new uint8_t[buf_size];
    last_imu_pkt = new IMU_pkt;


    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_AnelloEVK::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Failed to allocate ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AnelloEVK ExternalAHRS initialised");
}

/*
  check the UART for more data
  returns true if the function should be called again straight away
*/
bool AP_ExternalAHRS_AnelloEVK::check_uart()
{
    if (!port_open) {
        return false;
    }
    WITH_SEMAPHORE(state.sem);

    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }
    if (pkt_offset < buf_size) {
        ssize_t nread = uart->read(&pkt_buf[pkt_offset], MIN(n, unsigned(buf_size-pkt_offset)));
        if (nread <= 0) {
            return false;
        }
        pkt_offset += nread;
    }

    if (pkt_buf[0] != 0xD3) {
        goto reset;
    }

    bool match_imu_header;
    match_imu_header = (0 == memcmp(&pkt_buf[0], &imu_packet.header, MIN(sizeof(imu_packet.header), unsigned(pkt_offset-1))));

    if (!match_imu_header) {
        goto reset;
    }

    if (match_imu_header && pkt_offset >= imu_pkt_length) {

        // Check if this is a valid packet
        // Calculate the crc-2 of the packet
        uint32_t crc = crc_crc24(&pkt_buf[0], imu_pkt_length-3);

        // Calculate the received CRC
        uint32_t supplied_crc =  pkt_buf[imu_pkt_length-3] << 16 | pkt_buf[imu_pkt_length-4] << 8 | pkt_buf[imu_pkt_length-5];

        // Process packet if they are equal
        if (crc == supplied_crc ) {
            process_imu_packet(&pkt_buf[imu_pkt_length]);
            memmove(&pkt_buf[0], &pkt_buf[imu_pkt_length], pkt_offset-imu_pkt_length);
            pkt_offset -= imu_pkt_length;
        } else {
            goto reset;
        }
    } else {
            goto reset;
    }
    return true;

reset:
    uint8_t *p = (uint8_t *)memchr(&pkt_buf[1], (char)0xD3, pkt_offset-1);
    if (p) {
        uint8_t newlen = pkt_offset - (p - pkt_buf);
        memmove(&pkt_buf[0], p, newlen);
        pkt_offset = newlen;
    } else {
        pkt_offset = 0;
    }
    return true;
}

/*
  process imu packet
 */
void AP_ExternalAHRS_AnelloEVK::process_imu_packet(const uint8_t *b)
{
    const struct IMU_pkt &pkt1 = *(struct IMU_pkt *)b;

    last_imu_pkt_ms = AP_HAL::millis();
    *last_imu_pkt = pkt1;

    {
        WITH_SEMAPHORE(state.sem);
        state.accel = Vector3f{float(pkt1.accel[0]), float(pkt1.accel[1]), float(pkt1.accel[2])};
        state.gyro = Vector3f{float(pkt1.gyro[0]), float(pkt1.gyro[1]), float(pkt1.fog_z)};
    }

    {
        AP_ExternalAHRS::ins_data_message_t ins;

        ins.accel = state.accel;
        ins.gyro = state.gyro;
        ins.temperature = pkt1.temp;

        AP::ins().handle_external(ins);
    }

}

void AP_ExternalAHRS_AnelloEVK::update_thread(void)
{
    if (!port_open) {
        port_open = true;
        uart->begin(baudrate, 1024, 0);
    }

    while (true) {
        if (!check_uart()) {
        hal.scheduler->delay(1);
        }
    }
}


// get serial port number for the uart
int8_t AP_ExternalAHRS_AnelloEVK::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

bool AP_ExternalAHRS_AnelloEVK::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return (now - last_imu_pkt_ms < 40);
}

bool AP_ExternalAHRS_AnelloEVK::initialised(void) const
{
    return last_imu_pkt_ms != 0;
}

bool AP_ExternalAHRS_AnelloEVK::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "AnelloEVK unhealthy");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_AnelloEVK::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (last_imu_pkt_ms != 0 ) {
        status.flags.initalized = 1;
    }
    if (healthy() && last_imu_pkt_ms != 0) {
        status.flags.attitude = 1;
        status.flags.vert_vel = 1;
        status.flags.vert_pos = 1;

    }
}

void AP_ExternalAHRS_AnelloEVK::send_status_report(mavlink_channel_t chan) const
{
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    // send message
    // const struct AnelloPacket_IMU_pkt &pkt = *(struct AnelloPacket_IMU_pkt *)last_imu_pkt;

    // const float vel_gate = 4; // represents hz value data is posted at
    // const float pos_gate = 4; // represents hz value data is posted at
    // const float hgt_gate = 4; // represents hz value data is posted at
    // const float mag_var = 0; //we may need to change this to be like the other gates, set to 0 because mag is ignored by the ins filter in vectornav
    mavlink_msg_ekf_status_report_send(chan, flags,0,0,0,0,0,0);
}


#endif // HAL_EXTERNAL_AHRS_ENABLED

