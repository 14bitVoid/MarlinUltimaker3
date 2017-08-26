#ifndef FLOW_AS5048B
#define FLOW_AS5048B

#include <stdint.h>

/*!
 * \brief        Initialization of sensor AS5048B (e.g. set zero position).
 *
 * \param n      number of the sensor.
 */
void flowAS5048BInit(uint8_t n);

/*!
 * \brief        Send read angle command to sensor AS5048B.
 *
 * \param n      number of the sensor.
 */
void flowAS5048BStart(uint8_t n);

/*!
 * \brief        Read angle value of sensor AS5048B.
 *
 * \param n      number of the sensor.
 * \param value  angle value read
 */
bool flowAS5048BDone(uint8_t n, uint16_t &value);

#endif /* FLOW_AS5048B */

