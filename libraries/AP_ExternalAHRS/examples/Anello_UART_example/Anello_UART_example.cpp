/*
*  Access and process UART data from Anello EVK wired to TELEM2
*
*  Anello EVK data is currently ASCII encoding. See here for details:
*  https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#ascii-data-output-messages
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <string>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// ASCII Encoded Message Descriptors
const char COMMA_DELIMITER = ',';
const char END_CHECKSUM = '\n';// "CR"
const char END_DATA = '*';
const char PKT_IDENTIFIER = '#';


// ASCII Encoded Message Descriptors
// see Anello ref: https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#ascii-data-output-messages
const char* GPS_HEADER = "APGPS";
const char* GP2_HEADER = "APGP2";
const char* IMU_HEADER = "APIMU";
const char* INS_HEADER = "APINS";

uint pkt_counter = 0;

/**
 * @brief Convience class enumerating states for a simple parsing state machine.
 *
 */
enum class ParseState {
    WaitingFor_PktIdentifier,
    WaitingFor_MsgDescriptor,
    WaitingFor_Data,
    WaitingFor_Checksum
};

/**
 * @brief Convience class enumerating the expected types of messages from EVK.
 *
 */
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


/**
 * @brief Structure to hold attributes for the complete data packet sent from EVK.
 *
 */
struct Msg {
    PacketType msg_type;
    ParseState state;
    char header_type[5];
    char payload[100];
    char checksum[2];
    uint8_t running_checksum;
    uint8_t length;
} message_in;


// Declarations of functions
bool classify_packet(Msg &msg);
bool valid_packet(Msg &msg);
char read_uart(AP_HAL::UARTDriver *uart, const char *name);
std::vector<double> parse_packet(char payload[]);
void build_packet();
void handle_filter(const std::vector<double> &payload);
void handle_gnss(const std::vector<double> &payload);
void handle_imu(const std::vector<double> &packet);
void handle_packet(Msg &packet);
void update_thread();

void loop();
void setup();
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name);
static void write_uart(AP_HAL::UARTDriver *uart, const char *name, const char *s);
void write_uart(AP_HAL::UARTDriver *uart, const char *name, const Vector3f &vec);
void write_uart(AP_HAL::UARTDriver *uart, const char *name, const float &val);

/**
 * @brief Determine if the message corresponds to one of the expected types.
 *
 * @param msg Msg struct that includes vector of bytes received over UART
 * @return true If the first 5 payload bytes match an expected header.
 * @return false If the first 5 payload bytes do not match expected header.
 */
bool classify_packet(Msg &msg) {

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

/**
 * @brief Reads a byte of data from UART port
 *
 * @param uart Pointer to the UART driver
 * @param name Pointer to the name fo the UART.
 * @return char The byte of data read from UART.
 */
char read_uart(AP_HAL::UARTDriver *uart, const char *name) {
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return 0;
    }

    // Read one byte of data on from UART.
    char b = uart->read();
    return b;
}

/**
 * @brief Set the up uart object
 *
 * @param uart Pointer to the UART driver
 * @param name Pointer to the name fo the UART.
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name) {
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(921600);
}

/**
 * @brief Send an encoded string over UART.
 *
 * @param uart Pointer to UART instance to send string.
 * @param name Pointer to name of the UART.
 * @param s Pointer to string of bytes to send over UART.
 */
static void write_uart(AP_HAL::UARTDriver *uart, const char *name, const char *s) {
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->print(s);
}

/**
 * @brief Send an encoded vector of floats over UART.
 *
 * @param uart Pointer to UART instance to send data over.
 * @param name Pointer to name of the UART.
 * @param vec Reference to vector of 3 floats to send over UART.
 */
void write_uart(AP_HAL::UARTDriver *uart, const char *name, const Vector3f &vec) {
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }

    for (int i = 0; i <= vec.length(); i++) {
            uart->printf("vec[%i]: %f", i,vec[i]);
    }
}

/**
 * @brief Send an encoded vector of floats over UART.
 *
 * @param uart Pointer to UART instance to send data over.
 * @param name Pointer to name of the UART.
 * @param val Reference to float to send over UART.
 */
void write_uart(AP_HAL::UARTDriver *uart, const char *name, const float &val) {
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }

    uart->printf("float: %f", val);
}


/**
 * @brief Take vector of char values read from UART and turn them into appropriate float values.
 *
 * @param msg The vector of characters transmitted over UART
 * @return std::vector<float> A vector of floats converted from UART message
 */
// Parses the csv payload to a vector of floats.
std::vector<double> parse_packet(char payload[]) {

    std::vector<double> result;

    char *saveptr = nullptr;
    for (char *pname = strtok_r(payload, ",", &saveptr);
        pname != nullptr;
        pname = strtok_r(nullptr, ",", &saveptr)) {
        result[pkt_counter++] = atof(pname);
    }
    return result;
}


void handle_gnss(const std::vector<double> &msg) {
    write_uart(hal.serial(0), "SERIAL0", "handled gnss msg\n");

}

/**
 * @brief Takes a vector of imu values creates a Vector3f forwarding to AP's imu field.
 *
 * @param imu_msg the vector of floats containing imu values.
 */
void handle_imu(const std::vector<double> &imu_msg) {
    //auto accel = Vector3f{imu_msg[2], imu_msg[3], imu_msg[4]};
    //auto gyro = Vector3f{imu_msg[5], imu_msg[6], imu_msg[8]};

    write_uart(hal.serial(0), "SERIAL0", "handled imu msg\n");
    write_uart(hal.serial(0), "SERIAL0", imu_msg[11]);
}

void handle_filter(const std::vector<double> &msg) {
    write_uart(hal.serial(0), "SERIAL0", "handled filter msg\n");

}

// returns true if the XOR checksum for the packet is valid, else false.
bool valid_packet(Msg &msg)
{
    uint8_t checksum = 16 * char_to_hex(msg.checksum[0]) + char_to_hex(msg.checksum[1]);

    // Calculate the expected CRC
    // Simple XOR, see:
    // https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#ascii-data-output-messages
    return checksum == msg.running_checksum;
}



/**
 * @brief Parses the message received over UART based on what type of message.
 *
 * @param msg The message received over UART.
 */
void handle_packet(Msg &packet) {

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
        break;
    case PacketType::UNKNOWN:
        break;
    }
}

void loop(void) {
    // Read byte from Telem 2.
    static char b;
    b = read_uart(hal.serial(2), "SERIAL2");

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
                    pkt_counter = 0;
                }
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

    // also do a raw printf() on some platforms, which prints to the
    // debug console
#if HAL_OS_POSIX_IO
    ::printf("Hello on debug console at %.3f seconds\n", (double)(AP_HAL::millis() * 0.001f));
#endif

    hal.scheduler->delay(50);
}

void setup(void) {
    // Ensure that the uart can be initialized
    hal.scheduler->delay(1000);

    // Setup UARTs
    setup_uart(hal.serial(0), "SERIAL0");  // console
    setup_uart(hal.serial(2), "SERIAL2");  // telemetry 2
}

AP_HAL_MAIN();
