#ifndef FAN_DRIVER_H
#define FAN_DRIVER_H

#include <stdint.h>

void initFans();
/**
 Set the current fan speed for the side fans that cool the print.
 @param fan_speed, PWM output for the fans, in the range 0 to 255.
 */
void setCoolingFanSpeed(uint8_t fan_speed);
/**
 Get the currently active fan speed for the cooling fan.
 @return current cooling fan speed in the range 0 to 255.
 */
uint8_t getCoolingFanSpeed();

/**
 Set the fan speed for the hotend cooling fan. This fan cools the top of the hotend to prevent heat-creep.
 @param fan_speed, PWM output for the fans, in the range 0 to 255.
 */
void setHotendCoolingFanSpeed(uint8_t fan_speed);

#endif//FAN_DRIVER_H
