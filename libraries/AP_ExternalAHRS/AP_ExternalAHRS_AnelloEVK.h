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
  support for serial connected AHRS systems
 */

#pragma once

#include "AP_ExternalAHRS_backend.h"

#ifndef HAL_EXTERNAL_AHRS_ANELLO_EVK_ENABLED
#define HAL_EXTERNAL_AHRS_ANELLO_EVK_ENABLED HAL_EXTERNAL_AHRS_ENABLED
#endif

#if HAL_EXTERNAL_AHRS_ANELLO_EVK_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_ExternalAHRS_AnelloEVK: public AP_ExternalAHRS_backend
{
public:

    AP_ExternalAHRS_AnelloEVK(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;

    // check for new data
    void update() override {
        build_packet();
    };

private:
    // Useful ASCII encoded characters
    const char COMMA_DELIMITER = ',';
    const char END_CHECKSUM = '\r';// "CR"
    const char END_DATA = '*';
    const char PKT_IDENTIFIER = '#';

    // ASCII Encoded Message Descriptors
    // see Anello ref: https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#ascii-data-output-messages
    const char* GPS_HEADER = "APGPS";
    const char* GP2_HEADER = "APGP2";
    const char* IMU_HEADER = "APIMU";
    const char* INS_HEADER = "APINS";

    AP_HAL::UARTDriver *uart;
    bool port_open = false;
    HAL_Semaphore sem;
    int8_t port_num;
    uint32_t baudrate;
    uint32_t last_filter_pkt;
    uint32_t last_gps_pkt;
    uint32_t last_ins_pkt;
    uint pkt_counter = 0;

    enum class ParseState {
        WaitingFor_PktIdentifier,
        WaitingFor_MsgDescriptor,
        WaitingFor_Data,
        WaitingFor_Checksum
    };

    enum class PacketType {
        IMU,
        GPS,
        INS,
        UNKNOWN,
    };

    enum class IMUPacketField {
        TIME_ms,
        ACCEL_X_g,
        ACCEL_Y_g,
        ACCEL_Z_g,
        GYRO_X_dps,
        GYRO_Y_dps,
        GYRO_Z_dps,
        OPT_GYRO_Z_dps,
        ODOM_mps,
        ODOM_TIME_ms,
        TEMP_degC,
    };

    enum class GNSSFixType {
        NO_GPS,
        GPS_3D_FIX,
        GPS_FIX_3D_DGPS,
        GPS_OK_FIX_3D_RTK_FLOAT,
        GPS_OK_FIX_3D_RTK_FIXED,
    };

    struct Msg {
        PacketType msg_type;
        ParseState state;
        char header_type[5];
        char payload[200];
        char checksum[2];
        uint8_t running_checksum;
        uint8_t length;
    } message_in;

    struct {
        uint16_t week;
        uint32_t tow_ms;
        float hdop;
        float vdop;
        uint8_t num_sats;
        float speed_accuracy;
        int32_t msl_altitude;
        float horizontal_position_accuracy;
        float vertical_position_accuracy;
    } gnss_data;

    struct {
        uint16_t week;
        uint32_t tow_ms;
        int32_t lon;
        int32_t lat;
        int32_t hae_altitude;
        float ned_velocity_north;
        float ned_velocity_east;
        float ned_velocity_down;
        uint8_t fix_type;
    } filter_data;

    bool classify_packet(Msg &msg);
    bool valid_packet(Msg &msg);
    std::vector<double> parse_packet(char *payload);
    void build_packet();
    void handle_filter(std::vector<double> &payload);
    void handle_gnss(std::vector<double> &payload);
    void handle_imu(std::vector<double> &packet);
    void handle_packet(Msg &packet);
    void post_filter();
    void post_imu();
    void update_thread();
    int i = 0;


};

#endif // HAL_EXTERNAL_AHRS_ENABLED
