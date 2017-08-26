#include "temperaturePID.h"
#include "fan_driver.h"
#include "stepper.h"

TemperaturePID::TemperaturePID()
{
    Kff = 0.0;
    Kp = 0.0;
    Ki = 0.0;
    Kd = 0.0;
    Ki_max = 255;
    Kpcf = 0.0;
    Ke = 0.0;
    Ke_extruder_index = 255;
    functional_range = DEFAULT_PID_FUNCTIONAL_RANGE;
    max_output = 255;
    min_output = 0;
    target_temperature = 0.0;
    
    i_state = 0.0;
    d_state = 0.0;
    previous_sample = 0.0;
}

uint8_t TemperaturePID::update(const float current_temperature)
{
    float error = target_temperature - current_temperature;
    float e_position = float(st_get_position(E_AXIS)) / axis_steps_per_unit[E_AXIS];
    uint8_t result;
    
    if (error > functional_range)
    {
        resetState();
        result = max_output;
    }
    else if (error < -functional_range || target_temperature == 0)
    {
        previous_sample = current_temperature;
        result = min_output;
    }
    else
    {
        float p_term = Kp * error;
        i_state += error * Ki;
        i_state = constrain(i_state, -Ki_max, Ki_max);

        d_state = (1.0 - K1) * (Kd * (current_temperature - previous_sample)) + (K1 * d_state);
        previous_sample = current_temperature;
        float ff_term = Kff * (target_temperature - FEED_FORWARD_MINIMAL_TEMPERATURE);
        float pcf_term = Kpcf * getCoolingFanSpeed();
        float e_term = 0.0;
        
        //If the current active extruder is the extruder that we are tracking, then apply the extrusion to our Ke value.
        // Note that there is a slight inconsistency here, in theory the planner could be executing a block with a different extruder then the "active_extruder".
        // In practice this doesn't happen, and if it happens then this is just a tiny hickup for 1 update cycle, limiting the effect.
        if (active_extruder == Ke_extruder_index)
        {
            e_term = max(0.0, (e_position - previous_maximum_e_position) * Ke);
        }
        result = constrain(ff_term + pcf_term + e_term + p_term + i_state - d_state, min_output, max_output);
    }
    if (current_temperature < min_input || current_temperature > max_input)
    {
        result = min_output;
    }
    previous_e_position = e_position;
    if (e_position > previous_maximum_e_position)
        previous_maximum_e_position = e_position;
    previous_sample = current_temperature;
    return result;
}

void TemperaturePID::updateExtrusionPosition()
{
    //Reset the maximum extrusion position to the current E position.
    float e_position = float(st_get_position(E_AXIS)) / axis_steps_per_unit[E_AXIS];
    previous_maximum_e_position = previous_maximum_e_position - previous_e_position + e_position;
    previous_e_position = e_position;
}

void TemperaturePID::resetExtrusionPosition(float offset)
{
    previous_maximum_e_position = previous_e_position + offset;
}

void TemperaturePID::resetState()
{
    i_state = 0.0;
    d_state = 0.0;
}
