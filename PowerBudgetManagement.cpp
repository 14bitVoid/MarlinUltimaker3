#include "PowerBudgetManagement.h"

PowerBudgetManagement::PowerBudgetManagement(uint8_t nr_of_heaters)
{
    this->nr_of_heaters = min(nr_of_heaters, EXTRUDERS);
    idle_power_consumption = 0;
    total_power_budget = 0;
}

uint8_t PowerBudgetManagement::getActualBedOutput()
{
    float remaining_power_budget = total_power_budget - idle_power_consumption;
    for (uint8_t i = 0; i < nr_of_heaters; i++)
    {
        remaining_power_budget -= slots[i].getCurrentPowerUsage();
    }
    bed.setRemainingPowerBudget(remaining_power_budget);
    
    return bed.getActualHeaterOutput();
}
