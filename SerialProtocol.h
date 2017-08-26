#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

/**
 Marlin serial protocol handler.
 This is a new communication protocol implementation which is not backwards compatible
 with the old MarlinV1 protocol used by many repraps and the Ultimaker Original and Ultimaker2.

 The decision to change this protocol was made because the checksum and resend implementation of the MarlinV1 protocol where not robust enough.
 */

/*** Protocol: Description ***
 Host: The system talking to Marlin.
 Slave: Marlin.
 The communication protocol is not mirrored, meaning that the host to slave communication is not the same protocol as the slave to host communication.
 This is because the slave is limited in memory and CPU. And does not have the facilities to keep buffers for re-sending.
 Also the current slave implementation has no concept of a "message" towards the host, but sends data in seperate pieces.
 Changing this would be a serious effort due to the  errors being send asynchronized and thus can be in-between of a normal message.
 */

/*** Protocol: Implementation ***

 *** Host to slave **
 The host to slave* protocol has the following elements:
 * Start of packet
 * Sequence number
 * Data length
 * Payload
 * CRC16

 Start of packet: 0xFF, chosen to fix any possible framing errors.
 Sequence number: 0-254, increased for each message, used for error detection and resend recovery, since 0xFF 0xFF can be received at start of serial comms
 Data length: 1-MAX_CMD_SIZE, size of the payload.
 Payload: squence of bytes, only ASCII. Between 0x1F and 0x7F. The Data length field indicates the amount of payload data.
 CRC16: A CRC16-CCITT calculated over the sequence number, data length and payload data.

 *** Slave to host ***
 ASCII line based protocol for legacy reasons.
 Special commands:
 "ok N[0-254] P[0-16]\n" - Message received with sequence number N, and there is P room in the planner buffer for G0/G1 commands.
 "Resend: [0-254]\n"     - Message to indicate that a protocol error happened, and the next expected sequence number to receive.
 "ProtoErr: [ascii]\n"   - Actuall error message to indicate the type of protocol error, for debugging.
 "Error: [ascii]\n"      - System error.
 "LOG: [ascii]\n"        - Log the message in the system log.
 Any other data is part of the reply of a latest command without an "ok" until the "ok" message is received.
 */

#include "Configuration_adv.h"

#include <assert.h>
#include <inttypes.h>

class SerialProtocol
{
public:
    /** Defines the NetworkPacket to receive and possibly send! */
    typedef struct
    {
        uint8_t start_of_packet;
        uint8_t sequence_number;
        uint8_t payload_size;
        uint8_t payload[MAX_CMD_SIZE];
        uint16_t crc;
    } network_packet_t;

    /** @brief Define function pointer callback as a function taking a network_packet_t and returning a boolean value
     *  std::function from <functional> is not available!
     */
    typedef bool (*callback_handle_packet_t)(SerialProtocol::network_packet_t packet);


    /** @brief Read a packet, a character at a time
     *  @param callback When a packet is complete (and without errors), it's passed on to this callback.
     */
    static void readCommand(callback_handle_packet_t callback);

    /** @brief Will put out the "Resend: <sequence_number>" message onto the serial line */
    static void requestResend();

    /** @brief Cyclic increase of the sequence number
     */
    static inline void increaseSequenceNumber()
    {
        sequence_number = (sequence_number + 1) % MAX_SEQUENCE_NUMBER;
    }

private:
    /** @brief The sequence number of the packet to be received */
    static uint8_t sequence_number;

    /** constexpr is not allowed */
    static const uint8_t MAX_SEQUENCE_NUMBER = 255;
    static const uint8_t HEADER_BYTE         = 0xFF;

    /** Allowable characters (included), starting with space to ~ */
    static const uint8_t MINIMUM_ALLOWED_PAYLOAD_CHARACTER = 0x1F + 1;
    static const uint8_t MAXIMUM_ALLOWED_PAYLOAD_CHARACTER = 0x7F - 1;

    /** Possible states */
    static const uint8_t WAIT_FOR_HEADER_BYTE     = 0;
    static const uint8_t WAIT_FOR_SEQUENCE_NUMBER = 1;
    static const uint8_t WAIT_FOR_PAYLOAD_SIZE    = 2;
    static const uint8_t WAIT_FOR_PAYLOAD_OR_CRC  = 3;
    static const uint8_t WAIT_FOR_FINAL_CRC_BYTE  = 4;
};

#endif//SERIAL_PROTOCOL_H