#include "PowerBudgetManagementBed.h"

PowerBudgetManagementBed::PowerBudgetManagementBed()
{
    // all defaults set here are meant to not cause errors or the heaters or bed being turned on until valid values are sent by griffin.
    requested_output = 0;
    remaining_power_budget = 0;
    voltage = 0;
    temperature = 0;
    nominal_resistance = 1;
    resistance_per_degree = 0;
}

uint8_t PowerBudgetManagementBed::getActualHeaterOutput()
{
    // The math here is very simple but just in case:
    // P = power in watts, U = tension in Volts, I = current in Ampere and R is resistance in Ohm.
    // P = U * I
    // R = U/I
    // I = U/R
    // P = U * U / R or P = U^2 / R
    // resistance is reasonably linearly dependent on temperature.
    // R = Rn + T * Ra
    // P = U^2 / (Rn + T * Ra)
    float full_on_power = voltage * voltage / (nominal_resistance + resistance_per_degree * (temperature - 60)); // nominal temperature is set at 60 degC
    return min(requested_output, constrain(PID_MAX * remaining_power_budget / full_on_power, 0, PID_MAX));// the constrained value is the calculated maximum output.
}
