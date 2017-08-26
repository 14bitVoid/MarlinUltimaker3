#include <stdlib.h>

#include "temperature.h"
#include "Marlin.h"
#include "stepper.h"
#include "flow_AS5048B.h"
/*
#define SENSOR_WHEEL_DIAMETER 9.1
#define SENSOR_WHEEL_CIRCUM (SENSOR_WHEEL_DIAMETER * M_PI)
#define SENSOR_STEPS_PER_EXTRUSION_STEPS (axis_steps_per_unit[E_AXIS] / (1.0 / (SENSOR_WHEEL_CIRCUM / 16384)
*/

#define MAX_MILLIS_FLOW_POSITION        2000UL
//50 flow sensor steps equals 77.6 extrusion steps, add a bit of a safety margin for noise.
#define MIN_DELTA_FLOW_POSITION         100
#define MIN_DELTA_ANGLE                 50
#define MAX_COUNT_FILAMENT_FLOW_ERROR   2

struct flow_sensor_data
{
    bool          is_first_check_flow_position;
    bool          is_first_flow_position;
    bool          changing_flow_position;
    uint16_t      count_filament_flow_error;
    int16_t       angle;
    long          flow_position;
    unsigned long previous_millis;
};

static struct flow_sensor_data sd[NR_OF_FLOW_SENSORS];

uint16_t flow_sensor_raw[EXTRUDERS];
long flow_sensor_e_position[EXTRUDERS];
bool flow_sensor_enabled;

void initFlowPosition(uint8_t n)
{
//    SERIAL_ECHOPGM("initFlowPosition, sensor=");
//    MSerial.println(n, HEX);

    sd[n].is_first_check_flow_position = true;
    sd[n].is_first_flow_position       = true;
    sd[n].changing_flow_position       = false;
    sd[n].count_filament_flow_error    = 0;
    sd[n].angle                        = 0;
    sd[n].flow_position                = 0;
    sd[n].previous_millis              = 0;
}

static void checkFlowSensor(uint8_t n, bool flow_position_change_started)
{
    int16_t old_angle;

    if (flow_position_change_started)
    {
        sd[n].angle                     = flow_sensor_raw[n];
        old_angle                       = sd[n].angle - MIN_DELTA_ANGLE;
        sd[n].count_filament_flow_error = 0;
    }
    else
    {
        old_angle   = sd[n].angle;
        sd[n].angle = flow_sensor_raw[n];
    }

    if (abs(sd[n].angle - old_angle) < MIN_DELTA_ANGLE)
    {
        sd[n].count_filament_flow_error++;
        if (sd[n].count_filament_flow_error == MAX_COUNT_FILAMENT_FLOW_ERROR)
        {
            SERIAL_ECHOPGM("WARNING:FILAMENT_FLOW:");
            SERIAL_ECHOLN(uint16_t(n));
            sd[n].count_filament_flow_error = 0;
        }
    }
    else
    {
        sd[n].count_filament_flow_error = 0;
    }
}

void checkFlowPosition(uint8_t n)
{
    long old_flow_position;

    if (flowAS5048BDone(n, flow_sensor_raw[n]))
    {
        flow_sensor_e_position[n] = st_get_position(E_AXIS);
        flowAS5048BStart(n);
    }

    if (sd[n].is_first_check_flow_position)
    {
        sd[n].previous_millis              = millis();
        sd[n].is_first_check_flow_position = false;
    }
    if (millis() - sd[n].previous_millis > MAX_MILLIS_FLOW_POSITION)
    {
        if (sd[n].is_first_flow_position)
        {
            sd[n].flow_position          = st_get_position(E_AXIS);
            old_flow_position            = sd[n].flow_position;
            sd[n].is_first_flow_position = false;
        }
        else
        {
            old_flow_position   = sd[n].flow_position;
            sd[n].flow_position = st_get_position(E_AXIS);
        }
        if (abs(sd[n].flow_position - old_flow_position) > MIN_DELTA_FLOW_POSITION)
        {
            checkFlowSensor(n, !sd[n].changing_flow_position);
            sd[n].changing_flow_position = true;
        }
        else
        {
            sd[n].changing_flow_position = false;
        }
        sd[n].previous_millis = millis();
    }
}
