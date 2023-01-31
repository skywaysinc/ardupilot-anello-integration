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
  Support for Anello EVK serially connected AHRS Systems
 */


#include "AP_ExternalAHRS_AnelloEVK.h"

#if HAL_EXTERNAL_AHRS_ANELLO_EVK_ENABLED

#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

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
        hal.scheduler->delay_microseconds(5);
    }
}

// Builds packets by looking at each individual byte, once a full packet has been read in it checks
// the checksum then handles the packet.
// see: https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#ascii-data-output-messages
void AP_ExternalAHRS_AnelloEVK::build_packet()
{
    WITH_SEMAPHORE(sem);
    uint32_t nbytes = MIN(uart->available(), 2048u);
    while (nbytes--> 0) {
        const char b = uart->read();

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
                    message_in.payload[0] = 0;
                    message_in.checksum[0] = 0;
                    message_in.running_checksum = 0;
                    message_in.header_type[0] = 0;
                    pkt_counter = 0;
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
                    pkt_counter = 0;
                    message_in.running_checksum ^= b;
                    break;
                }
                // Record data. The checksum runs over the message descriptor section also.
                message_in.running_checksum ^= b;
                message_in.header_type[pkt_counter++] = b;
                break;

            case ParseState::WaitingFor_Data:
                // If we have not receive the ASCII character representing the end of data,
                // continue recording data.
                if (b != END_DATA) {
                    message_in.payload[pkt_counter++] = b;
                    message_in.running_checksum ^= b;
                    message_in.length = pkt_counter;
                } else {
                    // If we got the "*"", record the checksum to check for data integrity.
                    message_in.state = ParseState::WaitingFor_Checksum;
                    pkt_counter = 0;
                }
                break;

            case ParseState::WaitingFor_Checksum:
                // We are still waiting for checksum data if we have not seen the "CR" character.
                // Therefore still record data.
                if (b!= END_CHECKSUM) {
                    message_in.checksum[pkt_counter++] = b;
                } else {
                    //If we got the "CR", check the checksums.
                    if(valid_packet(message_in)) {
                        handle_packet(message_in);
                    }

                    // Now that we got the end of the packet, and have handled the message if it needs handling,
                    // go back to waiting for data.
                    message_in.state = ParseState::WaitingFor_PktIdentifier;
                    message_in.payload[0] = 0;
                    message_in.checksum[0] = 0;
                    message_in.running_checksum = 0;
                }
                break;
        }
    }
}

// returns true if the packet header bytes matches an expected message type.
bool AP_ExternalAHRS_AnelloEVK::classify_packet(Msg &msg) {

    // Try to classify by comparing received header to declared expected headers
    if (strcmp(msg.header_type,IMU_HEADER)) {
        msg.msg_type = PacketType::IMU;
    } else if (strcmp(msg.header_type,GPS_HEADER) || strcmp(msg.header_type,GP2_HEADER)) {
        msg.msg_type = PacketType::GPS;
    } else if (strcmp(msg.header_type,INS_HEADER)) {
        msg.msg_type = PacketType::INS;
    } else {
        // If not matches declare unknown and return false.
        msg.msg_type = PacketType::UNKNOWN;
        return false;
    }
    return true;
}

// returns true if the XOR checksum for the packet is valid, else false.
bool AP_ExternalAHRS_AnelloEVK::valid_packet(Msg &msg)
{
    uint16_t checksum = 16 * char_to_hex(msg.checksum[0]) + char_to_hex(msg.checksum[1]);

    // Calculate the expected CRC
    // Simple XOR, see:
    // https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#ascii-data-output-messages
    return checksum == msg.running_checksum;
}

// Calls the correct functions based on the packet descriptor of the packet
void AP_ExternalAHRS_AnelloEVK::handle_packet(Msg &packet) {

    std::vector<double> parsed_values = parse_packet(packet.payload);

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
std::vector<double> AP_ExternalAHRS_AnelloEVK::parse_packet(char payload[]) {

    std::vector<double> result;

    char *saveptr = nullptr;
    for (char *pname = strtok_r(payload, ",", &saveptr);
        pname != nullptr;
        pname = strtok_r(nullptr, ",", &saveptr)) {
        result.push_back(atof(pname));
    }
    return result;
}

// Collects data from an imu packet into `imu_data`
// Ref: https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#apimu-message
void AP_ExternalAHRS_AnelloEVK::handle_imu(std::vector<double> &payload) {

    last_ins_pkt = AP_HAL::millis();
    {
        WITH_SEMAPHORE(state.sem);
        state.accel = Vector3f{static_cast<float>(payload[2]), static_cast<float>(payload[3]), static_cast<float>(payload[4])} * GRAVITY_MSS; // convert from g to m/s/s
        state.gyro = Vector3f{static_cast<float>(payload[5]), static_cast<float>(payload[6]), static_cast<float>(payload[8])} * DEG_TO_RAD; // use FOG gyro for z and convert to rads/s
        state.have_quaternion = false;
    }

    {
        AP_ExternalAHRS::ins_data_message_t ins {
            accel: state.accel,
            gyro: state.gyro,
            temperature: static_cast<float>(payload[11]),
        };

        AP::ins().handle_external(ins);
    }


    // @LoggerMessage: EAH1
    // @Description: External AHRS IMU data
    // @Field: TimeUS: Time since system startup
    // @Field: AX: x acceleration
    // @Field: AY: y acceleration
    // @Field: AZ: z acceleration
    // @Field: WX: x angular velocity
    // @Field: WY: y angular velocity
    // @Field: WZ: z angular velocity
    // @Field: OG_WZ: z angular velocity measured by FOG
    // @Field: TempC: Temperature
    AP::logger().WriteStreaming("EAH1", "TimeUS,AX,AY,AZ,WX,WY,WZ,OG_WZ,T",
                       "soooEEEEO", "C00000000",
                       "Qdddddddd",
                       AP_HAL::micros64(),
                       payload[2]*GRAVITY_MSS, payload[3]*GRAVITY_MSS, payload[4]*GRAVITY_MSS,
                       payload[5]*DEG_TO_RAD, payload[6]*DEG_TO_RAD, payload[7]*DEG_TO_RAD, payload[8]*DEG_TO_RAD,
                       payload[11]);
}

// Collects data from a gnss packet into `gnss_data`
// see: https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#apgps-message
void AP_ExternalAHRS_AnelloEVK::handle_gnss(std::vector<double> &payload)
{
    last_gps_pkt = AP_HAL::millis();


    gnss_data.week = static_cast<uint16_t>(payload[2] / (AP_MSEC_PER_WEEK * 1e6));
    gnss_data.tow_ms = static_cast<uint32_t>(payload[2] / 1e6 - gnss_data.week * AP_MSEC_PER_WEEK);

    gnss_data.num_sats = static_cast<uint8_t>(payload[13]);
    gnss_data.speed_accuracy = static_cast<float>(payload[14]);
    gnss_data.horizontal_position_accuracy = static_cast<float>(payload[9]);
    gnss_data.vertical_position_accuracy = static_cast<float>(payload[10]);


    // Anello only forwards a Position dilution, not separate for vertical and horizontal.
    gnss_data.hdop = static_cast<float>(payload [11]);
    gnss_data.vdop = gnss_data.hdop;

    gnss_data.msl_altitude = static_cast<int32_t>(payload[6]) * 100; // convert to cm.



    // @LoggerMessage: EAH2
    // @Description: External AHRS gps data
    // @Field: TimeUS: Time since system startup
    // @Field: GMS: GPS Time of week in ms
    // @Field: GWk: GPS Week
    // @Field: Lat: Latitude in deg
    // @Field: Lng: Longitude in deg
    // @Field: MSL: Height above mean sea level m
    // @Field: Spd: GPS speed in m/s
    // @Field: Hdg: GNSS Heading
    // @Field: Status: Fix type -> 0: No Fix, 2: 2D Fix, 3: 3D Fix, 5: Time Only
    // @Field: NSats: No of satellites
    // @Field: RTK: RTK status

    AP::logger().WriteStreaming("EAH2", "TimeUS,GMS,GWk,Lat,Lng,MSL,Spd,Hdg,Status,NSats,RTK",
                        "ss-DUmnh---", "CC000000000",
                        "QiHdddddddd",
                        AP_HAL::micros64(),
                        gnss_data.tow_ms,gnss_data.week,
                        payload[3],payload[4],payload[6],payload[7], payload[8],
                        payload[12],payload[13],payload[16]);

}

void AP_ExternalAHRS_AnelloEVK::handle_filter(std::vector<double> &payload)
{
    last_filter_pkt = AP_HAL::millis();

    switch ((GNSSFixType) payload[3]) {
        case(GNSSFixType::NO_GPS):
            filter_data.fix_type = GPS_FIX_TYPE_NO_GPS;
            break;
        case(GNSSFixType::GPS_3D_FIX):
            filter_data.fix_type = GPS_FIX_TYPE_3D_FIX;
            break;
        case(GNSSFixType::GPS_FIX_3D_DGPS):
            filter_data.fix_type = GPS_FIX_TYPE_DGPS;
            break;
        case (GNSSFixType::GPS_OK_FIX_3D_RTK_FLOAT):
            filter_data.fix_type = GPS_FIX_TYPE_RTK_FLOAT;
            break;
        case (GNSSFixType::GPS_OK_FIX_3D_RTK_FIXED):
            filter_data.fix_type = GPS_FIX_TYPE_RTK_FIXED;
            break;
    };

    filter_data.week = static_cast<uint16_t>(payload[2] / (AP_MSEC_PER_WEEK * 1000000));
    filter_data.tow_ms = static_cast<uint32_t>(payload[2] / 1000000 - filter_data.week * AP_MSEC_PER_WEEK);

    filter_data.lat = static_cast<int32_t>(payload[4]*1.0e7);
    filter_data.lon = static_cast<int32_t>(payload[5]*1.0e7);
    filter_data.hae_altitude = static_cast<int32_t>(payload[6]);

    filter_data.ned_velocity_north = static_cast<float>(payload[7]);
    filter_data.ned_velocity_east = static_cast<float>(payload[8]);
    filter_data.ned_velocity_down = static_cast<float>(payload[9]);

    // @LoggerMessage: EAH3
    // @Description: External AHRS Filter data
    // @Field: TimeUS: Time since system startup
    // @Field: GT: GPS Time in ns
    // @Field: Stat: Status
    // @Field: Lat: Latitude in deg
    // @Field: Lng: Longitude in deg
    // @Field: Alt: Height above ellipsoid
    // @Field: VN: x velocity
    // @Field: VE: y velocity
    // @Field: VZ: z velocity
    // @Field: Rll: Roll Angle, rotation about body frame X
    // @Field: Pit: Pitch Angle, rotation about body frame Y
    // @Field: Yaw: Heading Angle, rotation about body frame Z
    // @Field: ZUPT: 0: Moving, 1: Stationary
    AP::logger().WriteStreaming("EAH3", "TimeUS,GT,Stat,Lat,Lng,Alt,VN,VE,Vz,Rll,Pit,Yaw,ZUPT",
                       "ss-DUmnnnddd-", "CC00000000000",
                       "Qdddddddddddd",
                       AP_HAL::micros64(), payload[2], payload[3],
                       payload[4],payload[5], payload[6],
                       payload[7],payload[8], payload[9],
                       payload[10],payload[11], payload[12],
                       payload[13]);

}

void AP_ExternalAHRS_AnelloEVK::post_filter()
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
        fix_type: filter_data.fix_type,
        satellites_in_view: gnss_data.num_sats,

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
    if (filter_data.fix_type < 3) {
        hal.util->snprintf(failure_msg, failure_msg_len, "AnelloEVK no GPS lock");
        return false;
    }

    return true;
}

#endif // HAL_EXTERNAL_AHRS_ENABLED
