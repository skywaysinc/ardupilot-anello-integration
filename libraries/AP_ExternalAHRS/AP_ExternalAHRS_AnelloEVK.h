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
 *
 * This implementation follows the implementation for the VectorNav driver.
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
        check_uart();
    };

private:

    AP_HAL::UARTDriver *uart;        ///< Pointer to the uart accessor of the serial manager
    bool port_open = false;          ///< Flag to know if to read data from port
    HAL_Semaphore sem;               ///< ???
    int8_t port_num;                 ///< which port the device is connected too
    struct IMU_pkt *last_imu_pkt;
    uint16_t buf_size = 70;          ///< largest anello packet is 70 bytes
    uint16_t pkt_offset;
    uint32_t baudrate;               ///< bits/s of the UART communication
    uint32_t last_filter_pkt;
    uint32_t last_imu_pkt_ms;
    uint8_t *pkt_buf;
    bool check_uart();
    void process_imu_packet(const uint8_t *b);



    /**
     * @brief Background thread running, parsing the packet.
     *
     */
    void update_thread();

};

#endif //HAL_EXTERNAL_AHRS_ENABLED
