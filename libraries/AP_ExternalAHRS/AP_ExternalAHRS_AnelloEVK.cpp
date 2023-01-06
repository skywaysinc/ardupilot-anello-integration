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

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_AnelloEVK.h"
#if HAL_EXTERNAL_AHRS_ANELLO_EVK_ENABLED
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include <string>

enum class DescriptorSet {
    BaseCommand = 0x01,
    DMCommand = 0x0C,
    SystemCommand = 0x7F,
    IMUData = 0x80,
    GNSSData = 0x81,
    EstimationData = 0x82
};

enum class INSPacketField {
    ACCEL = 0x04,
    GYRO = 0x05,
    QUAT = 0x0A,
    MAG = 0x06,
    PRESSURE = 0x17
};

enum class GNSSPacketField {
    LLH_POSITION = 0x03,
    NED_VELOCITY = 0x05,
    DOP_DATA = 0x07,
    GPS_TIME = 0x09,
    FIX_INFO = 0x0B
};

enum class GNSSFixType {
    FIX_3D = 0x00,
    FIX_2D = 0x01,
    TIME_ONLY = 0x02,
    NONE = 0x03,
    INVALID = 0x04
};

enum class FilterPacketField {
    FILTER_STATUS = 0x10,
    GPS_TIME = 0x11,
    LLH_POSITION = 0x01,
    NED_VELOCITY = 0x02
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
                        //TODO: handle_packet(message_in);
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
     // Convert checksum ASCII encoded bytes to characters
    char char1 = char(msg.checksum[0]);
    char char2 = char(msg.checksum[1]);

    // Combine characters in string so it can be changed into an intergers
    std::string char3{char1,char2};
    uint16_t crc_st = std::stoi(char3, 0, 16);

    // Calculate the expected CRC
    // Simple XOR, see:
    // https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#ascii-data-output-messages
    uint16_t crc = 0;
    for (auto i : msg.payload) {
        crc ^= i;
    }

    return crc == crc_st;
}

// Calls the correct functions based on the packet descriptor of the packet
void AP_ExternalAHRS_AnelloEVK::handle_packet(const Msg &packet)
{
    switch ((DescriptorSet) packet.header[2]) {
    case DescriptorSet::IMUData:
        handle_imu(packet);
        post_imu();
        break;
    case DescriptorSet::GNSSData:
        handle_gnss(packet);
        break;
    case DescriptorSet::EstimationData:
        handle_filter(packet);
        post_filter();
        break;
    case DescriptorSet::BaseCommand:
    case DescriptorSet::DMCommand:
    case DescriptorSet::SystemCommand:
        break;
    }
}

// Collects data from an imu packet into `imu_data`
void AP_ExternalAHRS_AnelloEVK::handle_imu(const Msg &packet)
{
    last_ins_pkt = AP_HAL::millis();

    // Iterate through fields of varying lengths in INS packet
    for (uint8_t i = 0; i < packet.header[3]; i +=  packet.payload[i]) {
        switch ((INSPacketField) packet.payload[i+1]) {
        // Scaled Ambient Pressure
        case INSPacketField::PRESSURE: {
            imu_data.pressure = extract_float(packet.payload, i+2) * 100; // Convert millibar to pascals
            break;
        }
        // Scaled Magnetometer Vector
        case INSPacketField::MAG: {
            imu_data.mag = populate_vector3f(packet.payload, i+2) * 1000; // Convert gauss to milligauss
            break;
        }
        // Scaled Accelerometer Vector
        case INSPacketField::ACCEL: {
            imu_data.accel = populate_vector3f(packet.payload, i+2) * GRAVITY_MSS; // Convert g's to m/s^2
            break;
        }
        // Scaled Gyro Vector
        case INSPacketField::GYRO: {
            imu_data.gyro = populate_vector3f(packet.payload, i+2);
            break;
        }
        // Quaternion
        case INSPacketField::QUAT: {
            imu_data.quat = populate_quaternion(packet.payload, i+2);
            break;
        }
        }
    }
}

// Posts data from an imu packet to `state` and `handle_external` methods
void AP_ExternalAHRS_AnelloEVK::post_imu() const
{
    {
        WITH_SEMAPHORE(state.sem);
        state.accel = imu_data.accel;
        state.gyro = imu_data.gyro;

        state.quat = imu_data.quat;
        state.have_quaternion = true;
    }

    {
        AP_ExternalAHRS::ins_data_message_t ins {
            accel: imu_data.accel,
            gyro: imu_data.gyro,
            temperature: -300
        };
        AP::ins().handle_external(ins);
    }

    {
        AP_ExternalAHRS::mag_data_message_t mag {
            field: imu_data.mag
        };
        AP::compass().handle_external(mag);
    }

    {
        const AP_ExternalAHRS::baro_data_message_t baro {
            instance: 0,
            pressure_pa: imu_data.pressure,
            // setting temp to 25 effectively disables barometer temperature calibrations - these are already performed by AnelloEVK
            temperature: 25,
        };
        AP::baro().handle_external(baro);
    }
}

// Collects data from a gnss packet into `gnss_data`
void AP_ExternalAHRS_AnelloEVK::handle_gnss(const Msg &packet)
{
    last_gps_pkt = AP_HAL::millis();

    // Iterate through fields of varying lengths in GNSS packet
    for (uint8_t i = 0; i < packet.header[3]; i += packet.payload[i]) {
        switch ((GNSSPacketField) packet.payload[i+1]) {
        // GPS Time
        case GNSSPacketField::GPS_TIME: {
            gnss_data.tow_ms = extract_double(packet.payload, i+2) * 1000; // Convert seconds to ms
            gnss_data.week = be16toh_ptr(&packet.payload[i+10]);
            break;
        }
        // GNSS Fix Information
        case GNSSPacketField::FIX_INFO: {
            switch ((GNSSFixType) packet.payload[i+2]) {
            case (GNSSFixType::FIX_3D): {
                gnss_data.fix_type = GPS_FIX_TYPE_3D_FIX;
                break;
            }
            case (GNSSFixType::FIX_2D): {
                gnss_data.fix_type = GPS_FIX_TYPE_2D_FIX;
                break;
            }
            case (GNSSFixType::TIME_ONLY):
            case (GNSSFixType::NONE): {
                gnss_data.fix_type = GPS_FIX_TYPE_NO_FIX;
                break;
            }
            default:
            case (GNSSFixType::INVALID): {
                gnss_data.fix_type = GPS_FIX_TYPE_NO_GPS;
                break;
            }
            }

            gnss_data.satellites = packet.payload[i+3];
            break;
        }
        // LLH Position
        case GNSSPacketField::LLH_POSITION: {
            gnss_data.lat = extract_double(packet.payload, i+2) * 1.0e7; // Decimal degrees to degrees
            gnss_data.lon = extract_double(packet.payload, i+10) * 1.0e7;
            gnss_data.msl_altitude = extract_double(packet.payload, i+26) * 1.0e2; // Meters to cm
            gnss_data.horizontal_position_accuracy = extract_float(packet.payload, i+34);
            gnss_data.vertical_position_accuracy = extract_float(packet.payload, i+38);
            break;
        }
        // DOP Data
        case GNSSPacketField::DOP_DATA: {
            gnss_data.hdop = extract_float(packet.payload, i+10);
            gnss_data.vdop = extract_float(packet.payload, i+14);
            break;
        }
        // NED Velocity
        case GNSSPacketField::NED_VELOCITY: {
            gnss_data.ned_velocity_north = extract_float(packet.payload, i+2);
            gnss_data.ned_velocity_east = extract_float(packet.payload, i+6);
            gnss_data.ned_velocity_down = extract_float(packet.payload, i+10);
            gnss_data.speed_accuracy = extract_float(packet.payload, i+26);
            break;
        }
        }
    }
}

void AP_ExternalAHRS_AnelloEVK::handle_filter(const Msg &packet)
{
    last_filter_pkt = AP_HAL::millis();

    // Iterate through fields of varying lengths in filter packet
    for (uint8_t i = 0; i < packet.header[3]; i += packet.payload[i]) {
        switch ((FilterPacketField) packet.payload[i+1]) {
        // GPS Timestamp
        case FilterPacketField::GPS_TIME: {
            filter_data.tow_ms = extract_double(packet.payload, i+2) * 1000; // Convert seconds to ms
            filter_data.week = be16toh_ptr(&packet.payload[i+10]);
            break;
        }
        // LLH Position
        case FilterPacketField::LLH_POSITION: {
            filter_data.lat = extract_double(packet.payload, i+2) * 1.0e7; // Decimal degrees to degrees
            filter_data.lon = extract_double(packet.payload, i+10) * 1.0e7;
            filter_data.hae_altitude = extract_double(packet.payload, i+26) * 1.0e2; // Meters to cm
            break;
        }
        // NED Velocity
        case FilterPacketField::NED_VELOCITY: {
            filter_data.ned_velocity_north = extract_float(packet.payload, i+2);
            filter_data.ned_velocity_east = extract_float(packet.payload, i+6);
            filter_data.ned_velocity_down = extract_float(packet.payload, i+10);
            break;
        }
        // Filter Status
        case FilterPacketField::FILTER_STATUS: {
            filter_status.state = be16toh_ptr(&packet.payload[i+2]);
            filter_status.mode = be16toh_ptr(&packet.payload[i+4]);
            filter_status.flags = be16toh_ptr(&packet.payload[i+6]);
            break;
        }
        }
    }
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
    const float mag_var = 0; //we may need to change this to be like the other gates, set to 0 because mag is ignored by the ins filter in vectornav
    mavlink_msg_ekf_status_report_send(chan, flags,
                                       gnss_data.speed_accuracy/vel_gate, gnss_data.horizontal_position_accuracy/pos_gate, gnss_data.vertical_position_accuracy/hgt_gate,
                                       mag_var, 0, 0);

}

Vector3f AP_ExternalAHRS_AnelloEVK::populate_vector3f(const uint8_t *data, uint8_t offset) const
{
    return Vector3f {
        extract_float(data, offset),
        extract_float(data, offset+4),
        extract_float(data, offset+8)
    };
}

Quaternion AP_ExternalAHRS_AnelloEVK::populate_quaternion(const uint8_t *data, uint8_t offset) const
{
    return Quaternion {
        extract_float(data, offset),
        extract_float(data, offset+4),
        extract_float(data, offset+8),
        extract_float(data, offset+12)
    };
}

float AP_ExternalAHRS_AnelloEVK::extract_float(const uint8_t *data, uint8_t offset) const
{
    uint32_t tmp = be32toh_ptr(&data[offset]);

    return *reinterpret_cast<float*>(&tmp);
}

double AP_ExternalAHRS_AnelloEVK::extract_double(const uint8_t *data, uint8_t offset) const
{
    uint64_t tmp = be64toh_ptr(&data[offset]);

    return *reinterpret_cast<double*>(&tmp);
}

#endif // HAL_EXTERNAL_AHRS_ENABLED

