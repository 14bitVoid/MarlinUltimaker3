#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// This configuration file contains the basic settings.
// Advanced settings can be found in Configuration_adv.h
// BASIC SETTINGS: select your board type, temperature sensor type, axis scaling, and endstop configuration

// SERIAL_PORT selects which serial port should be used for communication with the host.
// This allows the connection of wireless adapters (for instance) to non-default port pins.
// Serial port 0 is still used by the Arduino bootloader regardless of this setting.
#define SERIAL_PORT 2

// This determines the communication speed of the printer
#define BAUDRATE 250000

// This defines the number of extruders
#define EXTRUDERS 2

// This defines the number of flow sensors
#define NR_OF_FLOW_SENSORS 2

// The following define selects which power supply you have.
// We need to set our line HIGH to keep our main relay on.
#define PS_ON_AWAKE  HIGH
#define PS_ON_ASLEEP LOW

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================
//
//// Temperature sensor settings:
// 0 is not used
// 20 is PT100 with INA826 amp in Ultiboard v2.0
// 21 is ADS101X with Ultiboard v2.x

#define TEMP_SENSOR_0 21
#define TEMP_SENSOR_1 21
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_BED 20

// The minimum temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define BED_MINTEMP 5

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 365
#define HEATER_1_MAXTEMP 365
#define HEATER_2_MAXTEMP 365
#define BED_MAXTEMP 200

// this magic value is the value the temperature is read out to be if no PT100 element is connected!
#define MAGIC_NO_PT100_CONNECTED_VALUE 700

//Check if the heater heats up MAX_HEATING_TEMPERATURE_INCREASE within MAX_HEATING_CHECK_MILLIS while the PID was at the maximum.
// If not, raise an error because most likely the heater is not heating up the temperature sensor. Indicating an issue in the system.
#define MAX_HEATING_TEMPERATURE_INCREASE 1
#define MAX_HEATING_CHECK_MILLIS (30 * 1000)

// PID settings:
#define PID_MAX 255 // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current

#define PID_OUTPUT_BY_PERCENTAGE(x) ((PID_MAX * x) / 100)
// this setting indicates the level of PWM output of a nozzle has to be before clamping the bed output becomes necessary (to avoid drawing too much power from the power supply.
#define THRESHOLD_OUTPUT_PER_CARTRIDGE_FOR_CLAMPING_BED_OUTPUT PID_OUTPUT_BY_PERCENTAGE(50)
// this setting is for the amount of clamping to be done when the cartridges are using too much power.
#define MAX_BED_OUTPUT_WHILE_CARTRIDGES_ARE_HEATING PID_OUTPUT_BY_PERCENTAGE(75)

//#define PID_DEBUG // Sends debug data to the serial port.
#define DEFAULT_PID_FUNCTIONAL_RANGE 25 // If the temperature difference between the target temperature and the actual temperature
                              // is more than PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
#define K1 0.99 //smoothing factor within the PID
#define PID_dT ((OVERSAMPLENR * 4.0)/(F_CPU / 64.0 / 256.0)) //sampling period of the temperature routine

//The 0 point for the feed forward factor of the PID controller.
#define FEED_FORWARD_MINIMAL_TEMPERATURE 35

//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit
//can be software-disabled for whatever purposes by
#define PREVENT_DANGEROUS_EXTRUDE
//if PREVENT_DANGEROUS_EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
#define PREVENT_LENGTHY_EXTRUDE

#define EXTRUDE_MINTEMP 170
#define EXTRUDE_MAXLENGTH 1000.0 //prevent extrusion of very large distances.

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// coarse Endstop Settings
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#ifndef ENDSTOPPULLUPS
  // fine Enstop settings: Individual Pullups. will be ignored if ENDSTOPPULLUPS is defined
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  //#define ENDSTOPPULLUP_ZMIN
#endif

#ifdef ENDSTOPPULLUPS
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif

// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
const bool X_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
const bool Y_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
const bool Z_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

#define INVERT_X_DIR true     // for Mendel set to false, for Orca set to true
#define INVERT_Y_DIR false    // for Mendel set to true, for Orca set to false
#define INVERT_Z_DIR true     // for Mendel set to false, for Orca set to true
#define INVERT_E0_DIR true    // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E1_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E2_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false

// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR 1
#define Z_HOME_DIR 1

// Travel limits after homing
#define X_MAX_POS 230
#define X_MIN_POS 0
#define Y_MAX_POS 230
#define Y_MIN_POS 0
#define Z_MAX_POS 240
#define Z_MIN_POS 0

//// MOVEMENT SETTINGS
#define HOMING_FEEDRATE {100*60, 100*60, 40*60, 0}  // set the homing speeds (mm/min)

// default settings

#define DEFAULT_AXIS_STEPS_PER_UNIT   {80.0,80.0,200,369}   // default steps per unit for Ultimaker UM3
#define DEFAULT_MAX_FEEDRATE          {300, 300, 40, 45}    // (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {9000,9000,100,10000} // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.

#define DEFAULT_ACCELERATION          3000      // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  3000      // X, Y, Z and E max acceleration in mm/s^2 for retracts

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK                20.0      // (mm/sec)
#define DEFAULT_ZJERK                 0.4       // (mm/sec)
#define DEFAULT_EJERK                 5.0       // (mm/sec)

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
//#define FAST_PWM_FAN

#define ENABLE_BED_LEVELING_PROBE
#ifdef ENABLE_BED_LEVELING_PROBE
  # define CONFIG_BED_LEVELING_CALIBRATION_DISTANCE 4
  # define CONFIG_BED_LEVELING_Z_MOVE_DISTANCE 10
  # define CONFIG_FALL_OFF_BED_LEVELING_HEIGHT 10.0 // height at which to start reducing the amount of Z-correction
  # define CONFIG_MAX_BED_LEVELING_HEIGHT 20.0      // height at which Z-correction is no longer applied.
  // The capacitive sensor code has 2 buffers. A linear line is plotted over each buffer, the point between
  // these lines is called the inflection point, the angle is the amount of inflection, when the angle is maximized it is assumed
  // to be at the height the nozzle touches the bed. These values are calibrated to work with the sample frequency, the speed of movement,
  // the distance moved and the size of integers used in calculation(overflow etc), please be careful.
  # define CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT1 100
  # define CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT2 100
  # define CONFIG_BED_LEVELING_Z_HEIGHT 7
  # define CONFIG_BED_LEVELING_PEAK_DET1 1200000UL
  # define CONFIG_BED_LEVELING_PEAK_DET2N 3     // numerator
  # define CONFIG_BED_LEVELING_PEAK_DET2D 4     // denominator
#endif//ENABLE_BED_LEVELING_PROBE

#define ENABLE_MONITOR_SERIAL_INPUT // Implement monitoring getting commands to be executed
#ifdef ENABLE_MONITOR_SERIAL_INPUT
  #define MONITOR_SERIAL_INPUT_TIMEOUT 5 * 60       // Timeout for 5 minutes: after this time of no communication received, marlin will stop the heating of hotends and bed.
  #define MONITOR_MINIMUM_BED_TEMPERATURE    50     // Minimum temperature threshold for marking the bed as hot
  #define MONITOR_MINIMUM_HOTEND_TEMPERATURE 50     // Minimum temperature threshold for marking the hotend as hot
#endif//ENABLE_MONITOR_SERIAL_INPUT

#include "Configuration_adv.h"
#include "thermistortables.h"

#endif // CONFIGURATION_H
