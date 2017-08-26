#ifndef TEMPERATURE_PID_H
#define TEMPERATURE_PID_H

#include <stdint.h>
#include "Configuration.h"

/**
    PID controller for temperature.
    Uniform implementation for an advanced PID controller.
    For standard PID controller information, look at https://en.wikipedia.org/wiki/PID_controller
    Runs at a fixed time interval controlled by the define PID_dT.
    Has the following standard control features:
    * Kp = proportional, part of standard PID controllers.
    * Ki = integral, part of standard PID controllers.
    * Kd = derivative, part of standard PID controllers.
    It has the following additional control features not found in standard PID controllers
    * Kff = Feed-forward. This uses the actual setpoint to generate a certain amount of output.
    * Kpcf = Print cooling fan feed-forward. Uses the print fan speed to add a certain amount of output.
    * Ke = Extrusion feed forward. Uses the amount of extrusion to adjust the heater output.
    
    Additional features are:
    * Functional range, if the setpoint differs more then this value a simple "on/off" logic is used.
    * Control range, outside of this control range the output is always off.
*/
class TemperaturePID
{
public:
    TemperaturePID();
    
    /**
     Update the PID controller. Call when a new temperature sample is ready.
     Implementation assumes this is called every PID_dT.
     @param: current_temperature, current temperature sample in degree celsius.
     @return: pwm value for the output in the range 0 to 255.
     */
    uint8_t update(float current_temperature);

    /**
     Set the target temperature to a new value.
     @param: temperature in degree celsius.
     */
    FORCE_INLINE void setTargetTemperature(float temperature)
    {
        target_temperature = temperature;
    }
    
    /**
     Get the target temperature
     @return: target temperature in degree celsius.
     */
    FORCE_INLINE float getTargetTemperature()
    {
        return target_temperature;
    }

    /**
        Modify the Kff factor.
        output = Kff * target_temperature
        @param: Kff, new Kff factor.
     */
    FORCE_INLINE void setKff(float Kff)
    {
        this->Kff = Kff;
    }

    /**
        Modify the Kp factor.
        output = Kp * (current_temperature - target_temperature)
        @param: Kp, new Kp factor.
     */
    FORCE_INLINE void setKp(float Kp)
    {
        this->Kp = Kp;
    }

    /**
        Modify the Ki factor.
        i_state += (current_temperature - target_temperature)
        output = Ki * i_state
        @param: Ki, new Ki factor.
     */
    FORCE_INLINE void setKi(float Ki)
    {
        this->Ki = Ki * PID_dT;
    }

    /**
        Set the maximum value for the integral. Limiting the amount of output the integral can provide.
        i_state = constrain(i_state, -Ki_max, Ki_max)
        @param: Ki_max, new maximum i_state
     */
    FORCE_INLINE void setKiMax(float Ki_max)
    {
        this->Ki_max = Ki_max;
    }

    /**
        Modify the Kd factor.
        output = Kd * (previous_temperature - current_temperature)
        @param: Kd, new Kd factor.
     */
    FORCE_INLINE void setKd(float Kd)
    {
        this->Kd = Kd / PID_dT;
    }

    /**
        Modify the Kpcf factor, Kpcf of 1.0 = 100% fan => 100% PWM output.
        output = Kpcf * current_fan_speed
        @param: Kpcf, new Kpcf factor.
     */
    FORCE_INLINE void setKpcf(float Kpcf)
    {
        this->Kpcf = Kpcf;
    }

    /**
        Modify the Ke factor, Ke of 1.0 = 1mm extrusion in 1 second => 100% heater output.
        output = Ke * current_extrusion
        @param: Ke, new Ke factor.
        @param: extruder_index
     */
    FORCE_INLINE void setKe(float Ke, uint8_t extruder_index)
    {
        //Calculate the Ke value so we can directly multiply it with the amount of extrusion. For this we divide it by our time base, and multiply by the maximum PWM output.
        this->Ke = Ke / PID_dT * 255.0f;
        this->Ke_extruder_index = extruder_index;
    }
    
    /**
        Set the functional range for this PID controller.
        Outside of this range from the target temperature the controller does a simple on/off logic.
        @param: functional_range, range of this controller, measured from the target temperature in degree celsius.
    */
    FORCE_INLINE void setFunctionalRange(float functional_range)
    {
        this->functional_range = functional_range;
    }
    
    /**
        Set the control range for this PID controller. If the current temperature is outside of this control range the PID controller always returns "off"
        @param: min_input, minimal measured temperature for this controller to function.
        @param: max_input, maximum measured temperature for this controller to function.
    */
    FORCE_INLINE void setControlRange(float min_input, float max_input)
    {
        this->min_input = min_input;
        this->max_input = max_input;
    }
    
    
    /**
        When Marlin changes the E position, the PID controllers need to be updated of this fact.
        Else the Ke tracking will fail. This will reset the maximum E position so the Ke factor keeps working after an E reset.
     */
    void updateExtrusionPosition();

    /**
        Reset the extrusion position.
        This will make the PID controller think that the filament is at the end of the hotend at the current E position, with the given offset;
     */
    void resetExtrusionPosition(float offset);
private:
    /** PID configuration values. */
    float Kff;               /*< Basic feed forward factor configuration. */
    float Kp;                /*< Proportional factor configuration. */
    float Ki;                /*< Integral factor configuration. */
    float Kd;                /*< Derivative factor configuration. */
    float Ki_max;            /*< Maximum value for the integral buildup, to prevent large overshoot due to integral buildup. */
    float Kpcf;              /*< Print cooling fan feed forward factor. Multiplied with the current fan speed to compensate for the cooling fan drawing away heat from the hotends. */
    float Ke;                /*< Extrusion feed forward factor. If the current active extruder equals the extruder linked to this controller then this factor is multiplied by the amount of mm E movement that is done */
    uint8_t Ke_extruder_index;/*< Linked extruder for the Ke factor. 255 if not set. */
    float functional_range;  /*< Maximum difference from the current temperature and the target temperature in which the controller does the advanced controll loop. */
    float min_input;         /*< Minimum input temperature for the PID controller to function. */
    float max_input;         /*< Maximum input temperature for the PID controller to function. */
    uint8_t max_output;      /*< Maximum PWM output that will be returned by this controller, defaults to 255 */
    uint8_t min_output;      /*< Mimimum PWM output that will be returned by this controller, defaults to 0 */
    float target_temperature;/*< Target temperature for the PID controller in degree celsius */
    
    /** PID runtime values */
    float i_state;         /*< State of the integral, updated every update call, and added to the output. */
    float d_state;         /*< State of the derivative, updated every update call, and slightly smoothed, to prevent noise in the derivative. */
    float previous_sample; /*< Previous temperature sample, used to calculate the derivative */
    float previous_e_position; /*< Last known E position. Used when a G92 is issues to correctly adjust the previous_maximum_e_position */
    float previous_maximum_e_position; /*< Maximum known e position. Used to make sure the Ke factor is only applied to extrusion. */
    
    /**
        Reset the internal state of the PID controller.
        This is called when the PID controller decides that it has no work to do, and thus internal tracked values need to be reset to prevent old build up state
        to propate to the next time the PID controller does work.
    */
    void resetState();
};

#endif//TEMPERATURE_PID_H
