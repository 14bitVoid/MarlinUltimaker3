#include "fan_driver.h"
#include "pca9635_driver.h"
#include "Marlin.h"

// UM3: Front fan connected to output 4 of the PCA9635
#define FAN_FRONT  3   /* Front location */

// UM3: Side fans connected to output 5 and 6 of the PCA9635
#define FAN_LEFT   4   /* Left location */
#define FAN_RIGHT  5   /* Right location */

static uint8_t current_fan_speed;

void initFans()
{
    //For the UM2 the head fan is connected to PJ6, which does not have an Arduino PIN definition. So use direct register access.
    DDRJ |= _BV(6);

#if defined(FAN_PIN) && (FAN_PIN > -1)
    SET_OUTPUT(FAN_PIN);
    #ifdef FAST_PWM_FAN
    setPwmFrequency(FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
#endif
}

/** @brief Sets the fan speed
 *  @param fan_location Identify which fan
 *  @param fan_speed The speed of the fan
 */
void setFanSpeed(uint8_t fan_location, uint8_t fan_speed)
{
    if (fan_location != FAN_LEFT &&
        fan_location != FAN_RIGHT &&
        fan_location != FAN_FRONT)
    {
        return;
    }

    setupPCA9635output(fan_location, 255 - fan_speed);
    executePCA9635output();
}

void setCoolingFanSpeed(uint8_t fan_speed)
{
    setFanSpeed(FAN_LEFT, fan_speed);
    setFanSpeed(FAN_RIGHT, fan_speed);
#if FAN_PIN > -1
    analogWrite(FAN_PIN, fan_speed);
#endif
    current_fan_speed = fan_speed;
}

uint8_t getCoolingFanSpeed()
{
    return current_fan_speed;
}

void setHotendCoolingFanSpeed(uint8_t fan_speed)
{
    setFanSpeed(FAN_FRONT, fan_speed);

    //For the UM2 the head fan is connected to PJ6, which does not have an Arduino PIN definition. So use direct register access.
    if (fan_speed)
        PORTJ |= _BV(6);
    else
        PORTJ &=~_BV(6);
}
