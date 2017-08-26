#ifndef POWER_BUDGET_MANAGEMENT_BED_H
#define POWER_BUDGET_MANAGEMENT_BED_H

#include <stdint.h>
#include "Configuration.h"


/**
 * This class is used to track all values needed to calculate the power usage of a bed and limit the power usage.
 * This is done by calculating the approximate bed resistance for the bed at the current temperature then taking in to account the power supply voltage
 * the power the bed would use when turned on fully is calculated, this is then used to calculate the maximum allowed PWM duty-cycle which is outputed as 0-255 range value.
 */
class PowerBudgetManagementBed
{
public:
    PowerBudgetManagementBed();
    
    /**
     * This function takes the output of the PID system and stores it, after setting this the getActualHeaterOutput()
     * function can be used to get the possibly smaller value that is to be set the PWM duty-cycle of the heater.
     * @param output The Requested output.
     */
    FORCE_INLINE void setRequestedHeaterOutput(uint8_t output) { requested_output = output; };

    /**
     * Tells the bed how much power it is allowed to burn
     * @param power The power the bed is alowed to burn in watts
     */
    FORCE_INLINE void setRemainingPowerBudget(float power) { remaining_power_budget = power; };

    /**
     * @param bed_voltage The voltage used to power the bed.
     */
    FORCE_INLINE void setBedVoltage(float bed_voltage) { voltage = bed_voltage; };

    /**
     * @param nominal_bed_resistance The nominal bed resistance at 60 degC (60 degC since values measured at the low end proved inaccurate in measurements).
     */
    FORCE_INLINE void setNominalResistance(float nominal_resistance) { this->nominal_resistance = nominal_resistance; };

    /**
     * @param bed_resistance_per_degree the bed resistance changes linearly based on temperature, so to calculate the power used we need to calculate the resistance first based on the temperature of the bed.
     */
    FORCE_INLINE void setResistancePerDegree(float resistance_per_degree) { this->resistance_per_degree = resistance_per_degree; };

    /**
     * Set the bed temperature as it's resistance is somewhat temperature dependent.
     * @param temp The temperature in degrees Celsius.
     */
    FORCE_INLINE void setCurrentBedTemperature(float temperature) { this->temperature = temperature; };

    /**
     * @return The 0-255 output to be set the PWM duty-cycle of the heater.
     */
    uint8_t getActualHeaterOutput();
private:
    uint8_t requested_output;
    float remaining_power_budget;
    float voltage;
    float temperature;
    float nominal_resistance;
    float resistance_per_degree;
};

#endif /* POWER_BUDGET_MANAGEMENT_BED_H */
