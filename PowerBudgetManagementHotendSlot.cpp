#include "PowerBudgetManagementHotendSlot.h"

PowerBudgetManagementHotendSlot::PowerBudgetManagementHotendSlot()
{
    // all defaults set here are meant to not cause errors or the heaters or bed being turned on until valid values are sent by griffin.
    requested_output = 0;
    max_power = 0;
    current_power_usage = 0;
    resistance = 1;
    voltage = 0;
    calculated_max_output = 0;
    calculated_power_usage = 0;
}

uint8_t PowerBudgetManagementHotendSlot::getActualHeaterOutput()
{
    uint8_t output = min(calculated_max_output, requested_output);
    current_power_usage = calculated_power_usage * output / PID_MAX;
    return output;
}
