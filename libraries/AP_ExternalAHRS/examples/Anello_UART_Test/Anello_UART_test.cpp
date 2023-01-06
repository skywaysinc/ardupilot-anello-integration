/*
*  Access and process UART data from Anello EVK wired to TELEM2
*
*  Anello EVK data is currently ASCII encoding. See here for details:
*  https://docs-a1.readthedocs.io/en/latest/communication_messaging.html#ascii-data-output-messages
*/


#include <AP_HAL/AP_HAL.h>
#include <array>
#include <vector>
#include <string>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// ASCII Encoded Message Descriptors
const std::vector<uint8_t> GPS_HEADER {0x41, 0x50, 0x47, 0x50, 0x53}; // "APGPS"
const std::vector<uint8_t> IMU_HEADER {0x41, 0x50, 0x49, 0x4D, 0x55}; // "APIMU"
const std::vector<uint8_t> INS_HEADER {0x41, 0x50, 0x49, 0x4E, 0x53}; // "APGPS"

// Useful ASCII encoded characters
const uint8_t COMMA_DELIMITER = 0x2C; // ","
const uint8_t END_CHECKSUM = 0x0D; // "CR"
const uint8_t END_DATA = 0x2A; // "*"
const uint8_t PKT_IDENTIFIER = 0x23; // "#"

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
    std::vector<uint8_t> payload;
    std::vector<uint8_t> checksum;
} message_in;


// Declarations of functions
bool classify_msg(Msg &msg);
bool validate_msg(const Msg &msg);
static int16_t read_uart(AP_HAL::UARTDriver *uart, const char *name);
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name);
static void write_uart(AP_HAL::UARTDriver *uart, const char *name, const char *s);
static void write_uart(AP_HAL::UARTDriver *uart, const char *name, const int16_t *b);
void loop();
void setup();

/**
 * @brief Determine if the message corresponds to one of the expected types.
 *
 * @param msg Msg struct that includes vector of bytes received over UART
 * @return true If the first 5 payload bytes match an expected header.
 * @return false If the first 5 payload bytes do not match expected header.
 */
bool classify_msg(Msg &msg) {
    // Pull out header section (5 bytes) from vector of received bytes.
    std::vector<uint8_t> msg_header = {msg.payload.begin(), msg.payload.begin()+5};

    // Try to classify by comparing received header to declared expected headers
    if (msg_header == IMU_HEADER) {
        msg.msg_type = PacketType::IMU;
        write_uart(hal.serial(0), "SERIAL0", "It's an imu\n");
    } else if (msg_header == GPS_HEADER) {
        msg.msg_type = PacketType::GPS;
        write_uart(hal.serial(0), "SERIAL0", "It's a GPS\n");
    } else if (msg_header == INS_HEADER) {
        msg.msg_type = PacketType::INS;
        write_uart(hal.serial(0), "SERIAL0", "It's an ins\n");
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

/**
 * @brief Reads a byte of data from UART port
 *
 * @param uart Pointer to the UART driver
 * @param name Pointer to the name fo the UART.
 * @return int16_t The byte of data read from UART.
 */
static int16_t read_uart(AP_HAL::UARTDriver *uart, const char *name) {
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return 0;
    }

    // Read one byte of data on from UART.
    int16_t b = uart->read();
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
 * @brief Print helpful message of the value of the received byte.
 *
 * @param uart Pointer to UART instance to send string.
 * @param name Pointer to name of the UART.
 * @param b Pointer to byte to write.
 */
static void write_uart(AP_HAL::UARTDriver *uart, const char *name, const int16_t *b) {
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->printf("Read byte is: %i \n", *b);
}

/**
 * @brief Send an encoded string over UART.
 *
 * @param uart Pointer to UART instance to send string.
 * @param name Pointer to name of the UART.
 * @param s Pointer to string of bytes to write.
 */
static void write_uart(AP_HAL::UARTDriver *uart, const char *name, const char *s) {
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->print(s);
}

void loop(void) {
    // Read byte from Telem 2.
    static int16_t b;
    b = read_uart(hal.serial(2), "SERIAL2");

    // Ouput contents to console for debugging.
    write_uart(hal.serial(0), "SERIAL0", &b);

    // If the byte does not make sense, do nothing.
    if (b < 0) {
        return;
    }

    // Simple state machine for packet collection and parsing.
    switch (message_in.state) {
        case ParseState::WaitingFor_PktIdentifier:
            if (b == PKT_IDENTIFIER) {
                // Ouput contents to console for debugging.
                write_uart(hal.serial(0), "SERIAL0", "Got Pkt identifier\n");
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
                write_uart(hal.serial(0), "SERIAL0", "Got Msg Descriptor:");
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
                // If we got the "*"", record the checksum to check for data integrity.
                write_uart(hal.serial(0), "SERIAL0", "Finished reading msg\n");
                message_in.state = ParseState::WaitingFor_Checksum;
            }
            break;

        case ParseState::WaitingFor_Checksum:
            // We are still waiting for checksum data if we have not seen the "CR" character.
            // Therefore still record data.
            if (b!= END_CHECKSUM) {
                message_in.checksum.push_back(b);
            } else {
                // If we got the "CR", check the checksums.
                write_uart(hal.serial(0), "SERIAL0", "Finished reading check sum\n");
                if(validate_msg(message_in)) {
                    write_uart(hal.serial(0), "SERIAL0", "Valid msg\n");
                    //TODO: handle_packet(message_in);
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

    hal.scheduler->delay(100);
}

void setup(void) {
    // Ensure that the uart can be initialized
    hal.scheduler->delay(1000);

    // Setup UARTs
    setup_uart(hal.serial(0), "SERIAL0");  // console
    setup_uart(hal.serial(2), "SERIAL2");  // telemetry 2
}

AP_HAL_MAIN();
