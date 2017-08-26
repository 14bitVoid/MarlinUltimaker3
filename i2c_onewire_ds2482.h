#ifndef I2C_ONEWIRE_DS2482_H
#define I2C_ONEWIRE_DS2482_H

#include <stdint.h>

/**
    Functions for the DS2482 i2c to 1wire convertor.
    These functions are synchronized, and thus take a longer time to execute. So they should not be used while printing,
    as they will block other important update loops.
*/

/// Setup and configure the i2cOneWrite convertors.
void i2cOneWireInit();
/// Generate a 1wire reset signal. Return true when a presence detect signal has been seen.
bool i2cOneWireReset(uint8_t index);
/// Write a 8bit byte to the 1wire device
void i2cOneWireWrite(uint8_t index, uint8_t data);
/// Read a 8bit byte from the 1 wire device
uint8_t i2cOneWireRead(uint8_t index);

#endif//I2C_ONEWIRE_DS2482_H
