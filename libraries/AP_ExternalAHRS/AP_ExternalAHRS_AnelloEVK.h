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

#pragma once

#include "AP_ExternalAHRS_backend.h"

// Only do this if External AHRS is Enabled
#if HAL_EXTERNAL_AHRS_ENABLED

// Includes
#include <GCS_MAVLink/GCS_MAVLink.h>

/**
 * @brief AnelloEVK implementation using the already existing External AHRS
 * pipeline
 */
class AP_ExternalAHRS_AnelloEVK: public AP_ExternalAHRS_backend

{
public:

    // Constructor
    AP_ExternalAHRS_AnelloEVK(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // Accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    int8_t get_port(void) const override;  // get serial port number, -1 for not enabled
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(mavlink_channel_t chan) const override;

    // check for new data
    void update() override {
        build_packet();
    };

private:

    /**
     * @brief Simple state for packet parsing.
     *
     * Which state of packet parsing we are in determines which part of the packet we are reading.
     *
     */
    enum class ParseState {
        WaitingFor_Preamble,        ///< Looking for packet start descriptor
        WaitingFor_PayloadLength,   ///< Looking for descriptor indicating number of bits of data
        Reading_Data,               ///< Reading bits that are considered data
        Checking_CRC                ///< Making sure this is a valid packet
    };

    /**
     * @brief Stuct to hold an Anello EVK packet.
     *
     * An AnelloEVK payload packet can be a maximum of 64 bytes, see:
     * https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#binary-data-output-messages
     */
    struct AnelloEVK_Packet {
        uint8_t preamble;               ///< 0xD3 indicating start of Anello Packet
        uint8_t data_length_bytes[2];   ///< 2 bytes (000000) + 10 bits for data length
        uint8_t payload[64];            ///< data
        uint8_t CRC[3];                 ///< 3 byte CRC
        uint8_t data_packet_length;     ///< This is an added field for convience.
    };

    /**
     * @brief Stuct containing information required for correct parsing of a UART "message"
     */
    struct {
        AnelloEVK_Packet packet;    ///< The parsed Anello packet
        ParseState state;           ///< The state of parsing
        uint8_t index;              ///< Convience variable to help byte counting in different parsing states
    } message_in;

    struct {
        Vector3f accel;
        Vector3f gyro;
        Vector3f mag;
        Quaternion quat;
        float pressure;
    } imu_data;

    struct {
        uint16_t week;
        uint32_t tow_ms;
        GPS_FIX_TYPE fix_type;
        uint8_t satellites;
        float horizontal_position_accuracy;
        float vertical_position_accuracy;
        float hdop;
        float vdop;
        int32_t lon;
        int32_t lat;
        int32_t msl_altitude;
        float ned_velocity_north;
        float ned_velocity_east;
        float ned_velocity_down;
        float speed_accuracy;
    } gnss_data;

    struct {
        uint16_t state;
        uint16_t mode;
        uint16_t flags;
    } filter_status;

    struct {
        uint16_t week;
        uint32_t tow_ms;
        float horizontal_position_accuracy;
        float vertical_position_accuracy;
        int32_t lon;
        int32_t lat;
        int32_t hae_altitude;
        float ned_velocity_north;
        float ned_velocity_east;
        float ned_velocity_down;
        float speed_accuracy;
    } ins_data;

    AP_HAL::UARTDriver *uart;        ///< Pointer to the uart accessor of the serial manager
    bool port_open = false;          ///< Flag to know if to read data from port
    const uint8_t GPS_MSG = 0xA2;    ///< Byte #2's value of the data section of the packet indicating GPS data
    const uint8_t IMU_MSG = 0xA1;    ///< Byte #2's value of the data section of the packet indicating IMU data
    const uint8_t INS_MSG = 0xA3;    ///< Byte #2's value of the data section of the packet indicating INS data
    const uint8_t PREAMBLE = 0xD3;   ///< Value which Anello EVK sends before sending packet
    HAL_Semaphore sem;               ///< ???
    int8_t port_num;                 ///< which port the device is connected too
    uint32_t baudrate;               ///< bits/s of the UART communication
    uint32_t last_filter_pkt;
    uint32_t last_gps_pkt;
    uint32_t last_ins_pkt;

    bool valid_packet(const AnelloEVK_Packet &packet) const;
    double extract_double(const uint8_t* data, uint8_t offset) const;
    float extract_float(const uint8_t* data, uint8_t offset) const;
    Quaternion populate_quaternion(const uint8_t* data, uint8_t offset) const;
    uint8_t data_packet_length(const AnelloEVK_Packet & packet) const;
    Vector3f populate_vector3f(const uint8_t* data, uint8_t offset) const;
    void build_packet();
    void handle_filter(const AnelloEVK_Packet &packet);
    void handle_gnss(const AnelloEVK_Packet &packet);
    void handle_imu(const AnelloEVK_Packet &packet);
    void handle_packet(const AnelloEVK_Packet &packet);
    void post_filter() const;
    void post_gnss() const;
    void post_imu() const;

    /**
     * @brief Background thread running, parsing the packet.
     *
     */
    void update_thread();

};

#endif //HAL_EXTERNAL_AHRS_ENABLED
