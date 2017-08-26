#include "SerialProtocol.h"
// Required to get the correct MSerial stuff - should be moved to something different, as well as the SERIAL_... macros
#include "Marlin.h"
#include "crc16.h"

uint8_t SerialProtocol::sequence_number = 0;

void SerialProtocol::readCommand(callback_handle_packet_t callback)
{
    /* Local static variables to keep track of the reading  of bytes to fill the packet */
    static SerialProtocol::network_packet_t packet;
    static uint8_t                          state = 0;
    static uint8_t                          payload_index   = 0;

    while (MYSERIAL.available() > 0)
    {
        int serial_input = MYSERIAL.read();
        if (serial_input == -1)
        {
            return;
        }
        unsigned char serial_char = (unsigned char)serial_input;

        /* What byte do we expect? */
        switch (state)
        {
            case WAIT_FOR_HEADER_BYTE :
            {
                if (serial_char == HEADER_BYTE)
                {
                    packet.start_of_packet = serial_char;
                    state++;
                }
                break;
            }
            case WAIT_FOR_SEQUENCE_NUMBER:
            {
                if (serial_char == HEADER_BYTE)
                {
                    // When starting communication, it's possible to receive an extra 0xFF... This would then wrongly be identified as the start of the packet.
                    // So this break here is in place so this will be eaten as the start of the packet, expecting the next packet to be the sequence number.
                    break;
                }
                if (serial_char != sequence_number)
                {
                    // Wrong expected sequence number, ignore complete packet, ask for resend of the expected sequence number
                    SERIAL_PROTOCOL_ERROR_START;
                    SERIAL_PROTOCOL_ERRORPGM(MSG_ERR_UNEXPECTED_SEQUENCE_NUMBER);
                    SERIAL_PROTOCOL_ERROR((unsigned int)serial_char);
                    SERIAL_PROTOCOL_ERROR(",");
                    SERIAL_PROTOCOL_BYTE_AS_NR_LN(sequence_number);
                    requestResend();
                    state = WAIT_FOR_HEADER_BYTE;
                }
                else
                {
                    packet.sequence_number = serial_char;
                    state++;
                }
                break;
            }
            case WAIT_FOR_PAYLOAD_SIZE:
            {
                if (serial_char >= MAX_CMD_SIZE)
                {
                    // Payload too big too handle
                    SERIAL_PROTOCOL_ERROR_START;
                    SERIAL_PROTOCOL_ERRORPGM(MSG_ERR_PAYLOAD_SIZE_TOO_BIG);
                    SERIAL_PROTOCOL_BYTE_AS_NR_LN(sequence_number);
                    requestResend();
                    state = WAIT_FOR_HEADER_BYTE;
                }
                else
                {
                    packet.payload_size = serial_char;
                    payload_index = 0;
                    state++;
                }
                break;
            }
            case WAIT_FOR_PAYLOAD_OR_CRC:
            {
                if (payload_index < packet.payload_size)
                {
                    if ((serial_char >= MINIMUM_ALLOWED_PAYLOAD_CHARACTER) && (serial_char <= MAXIMUM_ALLOWED_PAYLOAD_CHARACTER))
                    {
                        packet.payload[payload_index++] = serial_char;
                    }
                    else
                    {
                        SERIAL_PROTOCOL_ERROR_START;
                        SERIAL_PROTOCOL_ERRORPGM(MSG_ERR_PAYLOAD_ILLEGAL_CHARACTER);
                        SERIAL_PROTOCOL_F(payload_index, DEC);
                        SERIAL_PROTOCOL_ERRORLN("");
                        requestResend();
                        state = WAIT_FOR_HEADER_BYTE;
                    }
                }
                else
                {
                    packet.crc = uint16_t(serial_char) << 8;
                    state++;
                }
                break;
            }
            // After CRC byte 2, verify packet and pass it on if it's correct.
            case WAIT_FOR_FINAL_CRC_BYTE:
            {
                packet.crc += uint16_t(serial_char);

                Crc16 packet_crc(&packet.sequence_number, 1);
                packet_crc.update(&packet.payload_size, 1);
                packet_crc.update(packet.payload, packet.payload_size);

                if (packet.crc != packet_crc.getCrc())
                {
                    // Corrupted packet
                    SERIAL_PROTOCOL_ERROR_START;
                    SERIAL_PROTOCOL_ERRORPGM(MSG_ERR_CRC_MISMATCH);
                    SERIAL_PROTOCOL_BYTE_AS_NR_LN(sequence_number);
                    requestResend();
                }
                else
                {
                    // Hand off packet
                    if (callback(packet))
                    {
                        increaseSequenceNumber();
                    }
                    else
                    {
                        SERIAL_PROTOCOL_ERROR_START;
                        SERIAL_PROTOCOL_ERRORPGM(MSG_ERR_PACKET_UNHANDLED);
                        SERIAL_PROTOCOL_BYTE_AS_NR_LN(sequence_number);
                        requestResend();
                    }
                }
                // Yes, fall through, in both cases we expect to get a new packet
            }
            // Unknown case, or finished, wait for new packet in the next loop
            default:
            {
                // Wait for new start of packet
                state = WAIT_FOR_HEADER_BYTE;
                // Even if there is still serial input to process...
                return;
            }
        }
    }
}

void SerialProtocol::requestResend()
{
    SERIAL_PROTOCOLPGM(MSG_RESEND);
    SERIAL_PROTOCOL_BYTE_AS_NR_LN(sequence_number);
    SERIAL_PROTOCOLLNPGM(MSG_OK);
    MSerial.flush();
}

