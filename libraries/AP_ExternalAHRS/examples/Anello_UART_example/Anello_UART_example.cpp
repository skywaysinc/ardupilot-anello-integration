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
const std::vector<char> GPS_HEADER {0x41, 0x50, 0x47, 0x50, 0x53}; // "APGPS"
const std::vector<char> GP2_HEADER {0x41, 0x50, 0x47, 0x50, 0x32}; // "APGP2"
const std::vector<char> IMU_HEADER {0x41, 0x50, 0x49, 0x4D, 0x55}; // "APIMU"
const std::vector<char> INS_HEADER {0x41, 0x50, 0x49, 0x4E, 0x53}; // "APINS"

// Useful ASCII encoded characters
const char COMMA_DELIMITER = 0x2C; // ","
const char END_CHECKSUM = 0x0D; // "CR"
const char END_DATA = 0x2A; // "*"
const char PKT_IDENTIFIER = 0x23; // "#"

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

/**
 * @brief Structure to hold attributes for the complete data packet sent from EVK.
 *
 */
struct Msg {
    PacketType msg_type;
    ParseState state;
    std::vector<char> payload;
    std::vector<char> checksum;
} message_in;


// Declarations of functions
bool classify_msg(Msg &msg);
bool validate_msg(const Msg &msg);
char read_uart(AP_HAL::UARTDriver *uart, const char *name);
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name);
static void write_uart(AP_HAL::UARTDriver *uart, const char *name, const char *s);
std::vector<float> parse_msg (const std::vector<char> &msg);
void handle_imu(const std::vector<float> &msg);
void handle_msg(const Msg &msg);
void loop();
void setup();
void write_uart(AP_HAL::UARTDriver *uart, const char *name, const Vector3f &vec);

/**
 * @brief Determine if the message corresponds to one of the expected types.
 *
 * @param msg Msg struct that includes vector of bytes received over UART
 * @return true If the first 5 payload bytes match an expected header.
 * @return false If the first 5 payload bytes do not match expected header.
 */
bool classify_msg(Msg &msg) {
    // Pull out header section (5 bytes) from vector of received bytes.
    std::vector<char> msg_header = {msg.payload.begin(), msg.payload.begin()+5};

    // Try to classify by comparing received header to declared expected headers
    if (msg_header == IMU_HEADER) {
        msg.msg_type = PacketType::IMU;
    } else if (msg_header == GPS_HEADER || msg_header == GP2_HEADER) {
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

/**
 * @brief Runs simple XOR check to determine if there were any errors in transmission.
 *
 * @param msg Vector or received bytes
 * @return true If the received checksum matches the calculated checksum
 * @return false If the received checksum does not match the calculated checksum
 */
bool validate_msg(const Msg &msg) {

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
std::vector<float> parse_msg(const std::vector<char> &msg) {

    std::string field;
    std::vector<float> parsed_values;

    for (int i=0; i <= msg.size(); i++) {
        if (msg[i] == COMMA_DELIMITER || i == msg.size()) {
            write_uart(hal.serial(0), "SERIAL0", field.c_str());
            write_uart(hal.serial(0), "SERIAL0", "\n");

            parsed_values.push_back(atof(field.c_str()));
            field.clear();

        } else {
            field += msg[i];
        }
    }
    return parsed_values;
}

void handle_gnss(const std::vector<float> &msg) {
    write_uart(hal.serial(0), "SERIAL0", "handled gnss msg\n");

}

/**
 * @brief Takes a vector of imu values creates a Vector3f forwarding to AP's imu field.
 *
 * @param imu_msg the vector of floats containing imu values.
 */
void handle_imu(const std::vector<float> &imu_msg) {
    //auto accel = Vector3f{imu_msg[2], imu_msg[3], imu_msg[4]};
    //auto gyro = Vector3f{imu_msg[5], imu_msg[6], imu_msg[8]};

    write_uart(hal.serial(0), "SERIAL0", "handled imu msg\n");
}

/**
 * @brief Parses the message received over UART based on what type of message.
 *
 * @param msg The message received over UART.
 */
void handle_msg(const Msg &msg) {

    std::vector<float> parsed_values = parse_msg(msg.payload);


    switch (msg.msg_type) {
        case PacketType::IMU:
            handle_imu(parsed_values);
            break;
        case PacketType::GPS:
            handle_gnss(parsed_values);
            break;
        case PacketType::INS:
            //handle_filter(packet);
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

    if (b == PKT_IDENTIFIER) {
        message_in.state = ParseState::WaitingFor_PktIdentifier;
        message_in.payload.clear();
        message_in.checksum.clear();
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
                if(!classify_msg(message_in)) {
                    // not a valid message, go back to waiting for identifier
                    message_in.state = ParseState::WaitingFor_PktIdentifier;
                    write_uart(hal.serial(0), "SERIAL0", " Not sure what it is\n");
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
                message_in.state = ParseState::WaitingFor_Checksum;
            }
            break;

        case ParseState::WaitingFor_Checksum:
            // We are still waiting for checksum data if we have not seen the "CR" character.
            // Therefore still record data.
            if (b!= END_CHECKSUM) {
                message_in.checksum.push_back(b);
            } else {
                // If we got the "CRgyro", check the checksums.
                if(validate_msg(message_in)) {
                    write_uart(hal.serial(0), "SERIAL0", "Valid msg\n");
                    handle_msg(message_in);
                } else {
                    write_uart(hal.serial(0), "SERIAL0", "Invalid msg\n");
                };
                // Now that we got the end of the packet, and have handled the message if it needs handling,
                // go back to waiting for data.
                message_in.state = ParseState::WaitingFor_PktIdentifier;
                message_in.payload.clear();
                message_in.checksum.clear();
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
