#ifndef CRC16_H
#define CRC16_H

#include <stdlib.h>
#include <inttypes.h>

/** @brief Based on the Crc articles of M. Barr
 * Implements an effecient CRC calculation mechanism
 */
class Crc16
{
public:
    Crc16(const uint8_t* message=NULL, const size_t nr_of_bytes=0);

    void update(const uint8_t* message, const size_t nr_of_bytes);

    inline uint16_t getCrc() const
    {
        return result;
    }

private:
    uint16_t result;
};

#endif//CRC16_H