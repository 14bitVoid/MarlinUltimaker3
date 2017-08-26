#ifndef MARLIN_FLOW
#define MARLIN_FLOW

#include <stdint.h>

extern uint16_t flow_sensor_raw[EXTRUDERS];
extern long flow_sensor_e_position[EXTRUDERS];
extern bool flow_sensor_enabled;

/*!
 * \brief    Initalize flow position.
 */
void initFlowPosition(uint8_t n);

/*!
 * \brief    Echo error if E position changes and sensor angle does not change.
 *
 * \param n  number of the sensor.
 */
void checkFlowPosition(uint8_t n);

#endif /* MARLIN_FLOW */

