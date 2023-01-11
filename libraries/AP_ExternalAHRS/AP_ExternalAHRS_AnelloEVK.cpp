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
  support for Anello EVK serially connected AHRS Systems
 */

#include "AP_ExternalAHRS_AnelloEVK.h"

#if HAL_EXTERNAL_AHRS_ANELLO_EVK_ENABLED

#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <string>

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
    TYPE_3D_FIX = 3,
    TYPE_2D_FIX = 2,
    TIME_ONLY = 5,
    NONE = 0,
    INVALID = 4
};

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_AnelloEVK::AP_ExternalAHRS_AnelloEVK(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_AnelloEVK::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Failed to allocate ExternalAHRS update thread");
    }

    hal.scheduler->delay(5000);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AnelloEVK ExternalAHRS initialised");
}

void AP_ExternalAHRS_AnelloEVK::update_thread(void)
{
    if (!port_open) {
        port_open = true;
        uart->begin(baudrate);
    }

    while (true) {
        build_packet();
        hal.scheduler->delay_microseconds(100);
    }
}

// Builds packets by looking at each individual byte, once a full packet has been read in it checks the checksum then handles the packet.
void AP_ExternalAHRS_AnelloEVK::build_packet()
{
    WITH_SEMAPHORE(sem);
    uint32_t nbytes = MIN(uart->available(), 2048u);
    while (nbytes--> 0) {
        const int16_t b = uart->read();

        // If the byte does not make sense, do nothing.
        if (b < 0) {
            return;
        }

        // Simple state machine for packet collection and parsing.
        switch (message_in.state) {
            case ParseState::WaitingFor_PktIdentifier:
                if (b == PKT_IDENTIFIER) {
                    // Start looking for what type of message we can expect
                    message_in.state = ParseState::WaitingFor_MsgDescriptor;
                    // Clear the container for the data of the message.
                    message_in.payload.clear();
                    message_in.checksum.clear();
                }
                break;

            case ParseState::WaitingFor_MsgDescriptor:
                // If we get a comma, then we are done receiving the message descriptor
                if (b == COMMA_DELIMITER) {
                    if(!classify_packet(message_in)) {
                        // not a valid message, go back to waiting for identifier
                        message_in.state = ParseState::WaitingFor_PktIdentifier;
                    } else {
                        // Continue recording data now that we know it is one of the expected messages
                        message_in.state = ParseState::WaitingFor_Data;
                    }
                }
                // Record data. The checksum runs over the message descriptor section also.
                message_in.payload.push_back(b);
                break;

            case ParseState::WaitingFor_Data:
                // If we have not receive the ASCII character representing the end of data,
                // continue recording data.
                if (b != END_DATA) {
                    message_in.payload.push_back(b);
                } else {
                    // If we got the "*"", record the checksum to check for data integrity.
                    message_in.state = ParseState::WaitingFor_Checksum;
                }
                break;

            case ParseState::WaitingFor_Checksum:
                // We are still waiting for checksum data if we have not seen the "CR" character.
                // Therefore still record data.
                if (b!= END_CHECKSUM) {
                    message_in.checksum.push_back(b);
                } else {
                    //If we got the "CR", check the checksums.
                    if(valid_packet(message_in)) {
                        handle_packet(message_in);
                    }
                    // Now that we got the end of the packet, and have handled the message if it needs handling,
                    // go back to waiting for data.
                    message_in.state = ParseState::WaitingFor_PktIdentifier;
                    message_in.payload.clear();
                    message_in.checksum.clear();
                }
                break;
        }
    }
}

// returns true if the packet header bytes matches an expected message type.
bool AP_ExternalAHRS_AnelloEVK::classify_packet(Msg &msg) {
    // Pull out header section (5 bytes) from vector of received bytes.
    std::vector<uint8_t> msg_header = {msg.payload.begin(), msg.payload.begin()+5};

    // Try to classify by comparing received header to declared expected headers
    if (msg_header == IMU_HEADER) {
        msg.msg_type = PacketType::IMU;
    } else if (msg_header == GPS_HEADER) {
        msg.msg_type = PacketType::GPS;
    } else if (msg_header == INS_HEADER) {
        msg.msg_type = PacketType::INS;
    } else {
        // If not matches declare unknown and return false.
        msg.msg_type = PacketType::UNKNOWN;
        return false;
    }
    return true;
}

// returns true if the XOR checksum for the packet is valid, else false.
bool AP_ExternalAHRS_AnelloEVK::valid_packet(const Msg &msg) const
{
    uint8_t checksum = 16 * char_to_hex(msg.checksum[0]) + char_to_hex(msg.checksum[1]);

    // Calculate the expected CRC
    // Simple XOR, see:
    // https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#ascii-data-output-messages
    uint8_t crc = 0;
    for (auto i : msg.payload) {
        crc ^= i;
    }

    return crc == checksum;
}

// Calls the correct functions based on the packet descriptor of the packet
void AP_ExternalAHRS_AnelloEVK::handle_packet(const Msg &packet) {

    std::vector<float> parsed_values = parse_packet(packet.payload);

    switch (packet.msg_type) {
    case PacketType::IMU:
        handle_imu(parsed_values);
        break;
    case PacketType::GPS:
        handle_gnss(parsed_values);
        break;
    case PacketType::INS:
        handle_filter(parsed_values);
        post_filter();
        break;
    case PacketType::UNKNOWN:
        break;
    }
}

// Parses the csv payload to a vector of floats.
std::vector<float> AP_ExternalAHRS_AnelloEVK::parse_packet(const std::vector<uint8_t> &payload) const {

    std::string token;
    std::vector<float> result;

    for (int i = 0; i < payload.size(); i++) {
        if (payload[i] == COMMA_DELIMITER) {
            result.push_back(atof(token.c_str()));
            token.clear();
        } else {
            token += payload[i];
        }

    }
    return result;
}

// Collects data from an imu packet into `imu_data`
void AP_ExternalAHRS_AnelloEVK::handle_imu(const std::vector<float> &payload) {


    last_ins_pkt = AP_HAL::millis();
    {
        WITH_SEMAPHORE(state.sem);
        state.accel = Vector3f{payload[2], payload[3], payload[4]} * GRAVITY_MSS; // convert from g to m/s/s
        state.gyro = Vector3f{payload[5], payload[6], payload[8]} * DEG_TO_RAD; // use FOG gyro for z and convert to rads/s
        state.have_quaternion = false;
    }

    {
        AP_ExternalAHRS::ins_data_message_t ins {
            accel: state.accel,
            gyro: state.gyro,
            temperature: payload[11],
        };
        AP::ins().handle_external(ins);
    }

    // @LoggerMessage: EAH1
    // @Description: External AHRS data
    // @Field: TimeUS: Time since system startup
    // @Field: AX: x acceleration
    // @Field: AY: y acceleration
    // @Field: AZ: z acceleration
    // @Field: WX: x angular velocity
    // @Field: WY: y angular velocity
    // @Field: WZ: z angular velocity
    // @Field: OG_WZ: z angular velocity measured by FOG
    // @Field: ODO: Scaled Composite Odometer Value
    // @Field: ODO_TIME: Timestamp of Odometer Reading
    // @Field: T: Temperature
    AP::logger().WriteStreaming("EAH1", "TimeUS,AX,AY,AZ,WX,WY,WZ,OG_WZ,ODO,ODO_TIME,T",
                       "soooEEEEnsO", "C00000000C0",
                       "Qffffffffff",
                       AP_HAL::micros64(),
                       payload[2]*GRAVITY_MSS,payload[3]*GRAVITY_MSS,payload[4]*GRAVITY_MSS,
                       payload[5], payload[6], payload[7],payload[8],
                       payload[9], payload[10], payload[11]);
}

// Collects data from a gnss packet into `gnss_data`
void AP_ExternalAHRS_AnelloEVK::handle_gnss(const std::vector<float> &payload)
{
    last_gps_pkt = AP_HAL::millis();

    gnss_data.tow_ms = payload[2] / 1e6; // Convert seconds to ms
    gnss_data.week = payload[2]/604800;

    switch ((GNSSFixType) payload[15]) {
        case(GNSSFixType::TIME_ONLY):
        case(GNSSFixType::NONE):
        case(GNSSFixType::INVALID):
            gnss_data.fix_type = GPS_FIX_TYPE_NO_FIX;
            break;
        case (GNSSFixType::TYPE_2D_FIX):
            gnss_data.fix_type = GPS_FIX_TYPE_2D_FIX;
            break;
        case (GNSSFixType::TYPE_3D_FIX):
            gnss_data.fix_type = GPS_FIX_TYPE_3D_FIX;
            break;
    };

        gnss_data.satellites = payload[13];

        gnss_data.lon = payload[3];
        gnss_data.lat = payload[4];
        gnss_data.msl_altitude = payload[6];


        gnss_data.horizontal_position_accuracy =  payload[9];
        gnss_data.vertical_position_accuracy = payload[10];

        gnss_data.speed_accuracy = payload[14];
}

void AP_ExternalAHRS_AnelloEVK::handle_filter(const std::vector<float> &payload)
{
    last_filter_pkt = AP_HAL::millis();

    filter_data.tow_ms = payload[2] / 1e6; // Convert seconds to ms
    filter_data.week = payload[2]/604800;

    filter_data.lat = payload[4];
    filter_data.lon = payload[5];
    filter_data.hae_altitude = payload[6];

    filter_data.ned_velocity_north = payload[7];
    filter_data.ned_velocity_east = payload[8];
    filter_data.ned_velocity_down = payload[9];

    filter_status.state = payload[3];
    filter_status.mode = payload[13];

    gnss_data.hdop = payload [11];
    gnss_data.hdop = payload [11];
}

void AP_ExternalAHRS_AnelloEVK::post_filter() const
{
    {
        WITH_SEMAPHORE(state.sem);
        state.velocity = Vector3f{filter_data.ned_velocity_north, filter_data.ned_velocity_east, filter_data.ned_velocity_down};
        state.have_velocity = true;

        state.location = Location{filter_data.lat, filter_data.lon, gnss_data.msl_altitude, Location::AltFrame::ABSOLUTE};
        state.have_location = true;
    }

    AP_ExternalAHRS::gps_data_message_t gps {
        gps_week: filter_data.week,
        ms_tow: filter_data.tow_ms,
        fix_type: (uint8_t) gnss_data.fix_type,
        satellites_in_view: gnss_data.satellites,

        horizontal_pos_accuracy: gnss_data.horizontal_position_accuracy,
        vertical_pos_accuracy: gnss_data.vertical_position_accuracy,
        horizontal_vel_accuracy: gnss_data.speed_accuracy,

        hdop: gnss_data.hdop,
        vdop: gnss_data.vdop,

        longitude: filter_data.lon,
        latitude: filter_data.lat,
        msl_altitude: gnss_data.msl_altitude,

        ned_vel_north: filter_data.ned_velocity_north,
        ned_vel_east: filter_data.ned_velocity_east,
        ned_vel_down: filter_data.ned_velocity_down,
    };

    if (gps.fix_type >= 3 && !state.have_origin) {
        WITH_SEMAPHORE(state.sem);
        state.origin = Location{int32_t(filter_data.lat),
                                int32_t(filter_data.lon),
                                int32_t(gnss_data.msl_altitude),
                                Location::AltFrame::ABSOLUTE};
        state.have_origin = true;
    }

    AP::gps().handle_external(gps);
}

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
    return (now - last_ins_pkt < 40 && now - last_gps_pkt < 500 && now - last_filter_pkt < 500);
}

bool AP_ExternalAHRS_AnelloEVK::initialised(void) const
{
    return last_ins_pkt != 0 && last_gps_pkt != 0 && last_filter_pkt != 0;
}

bool AP_ExternalAHRS_AnelloEVK::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "AnelloEVK unhealthy");
        return false;
    }
    if (gnss_data.fix_type < 3) {
        hal.util->snprintf(failure_msg, failure_msg_len, "AnelloEVK no GPS lock");
        return false;
    }
    if (filter_status.state != 0x02) {
        hal.util->snprintf(failure_msg, failure_msg_len, "AnelloEVK filter not running");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_AnelloEVK::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (last_ins_pkt != 0 && last_gps_pkt != 0) {
        status.flags.initalized = 1;
    }
    if (healthy() && last_ins_pkt != 0) {
        status.flags.attitude = 1;
        status.flags.vert_vel = 1;
        status.flags.vert_pos = 1;

        if (gnss_data.fix_type >= 3) {
            status.flags.horiz_vel = 1;
            status.flags.horiz_pos_rel = 1;
            status.flags.horiz_pos_abs = 1;
            status.flags.pred_horiz_pos_rel = 1;
            status.flags.pred_horiz_pos_abs = 1;
            status.flags.using_gps = 1;
        }
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
    const float vel_gate = 4; // represents hz value data is posted at
    const float pos_gate = 4; // represents hz value data is posted at
    const float hgt_gate = 4; // represents hz value data is posted at
    const float mag_var = 0; //we may need to change this to be like the other gates, set to 0 because mag is ignored by the ins filter in anello
    mavlink_msg_ekf_status_report_send(chan, flags,
                                       gnss_data.speed_accuracy/vel_gate, gnss_data.horizontal_position_accuracy/pos_gate, gnss_data.vertical_position_accuracy/hgt_gate,
                                       mag_var, 0, 0);

}

#endif // HAL_EXTERNAL_AHRS_ENABLED
