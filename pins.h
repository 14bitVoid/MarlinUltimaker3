#ifndef PINS_H
#define PINS_H

/*****************************************************************
* Ultiboard v2.0 pin assignment
******************************************************************/

#ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif

#define X_STEP_PIN 25
#define X_DIR_PIN 23
#define X_STOP_PIN 22
#define X_ENABLE_PIN 27

#define Y_STEP_PIN 32
#define Y_DIR_PIN 33
#define Y_STOP_PIN 26
#define Y_ENABLE_PIN 31

#define Z_STEP_PIN 35
#define Z_DIR_PIN 36
#define Z_STOP_PIN 29
#define Z_ENABLE_PIN 34

#define HEATER_BED_PIN 4
#define TEMP_BED_PIN 10

#define HEATER_0_PIN  2
#define TEMP_0_PIN 8

#define HEATER_1_PIN 3
#define TEMP_1_PIN 9

#define HEATER_2_PIN -1
#define TEMP_2_PIN -1

#define E0_STEP_PIN         42
#define E0_DIR_PIN          43
#define E0_ENABLE_PIN       37

#define E1_STEP_PIN         49
#define E1_DIR_PIN          47
#define E1_ENABLE_PIN       48

#define LED_PIN            8
#define FAN_PIN            7
#define PS_ON_PIN          24
#define SAFETY_TRIGGERED_PIN     28 //PIN to detect the safety circuit has triggered

// Analog PIN to measure the main voltage
#define MAIN_VOLTAGE_MEASURE_PIN 14
// Connected to 24V - 100k -|- 4k7  - GND = ADC ~221 on Ultimaker 2.0 board
#define MAIN_BOARD_ADC_V2_0 221
// Connected to 24V - 100k -|- 10k  - GND = ADC ~447 ADC on Ultimaker 2.x board
#define MAIN_BOARD_ADC_V2_x 447
// Connected to 24V - 100k -|- 14k7 - GND = ADC ~630 ADC on Ultimaker board revision I
#define MAIN_BOARD_ADC_REV_I 630

#define MOTOR_CURRENT_PWM_XY_PIN 44
#define MOTOR_CURRENT_PWM_Z_PIN 45
#define MOTOR_CURRENT_PWM_E_PIN 46
//Motor current PWM conversion, PWM value = MotorCurrentSetting * 255 / range
#define MOTOR_CURRENT_PWM_RANGE 2000



#ifdef X_STOP_PIN
  #if X_HOME_DIR < 0
    #define X_MIN_PIN X_STOP_PIN
    #define X_MAX_PIN -1
  #else
    #define X_MIN_PIN -1
    #define X_MAX_PIN X_STOP_PIN
  #endif
#endif

#ifdef Y_STOP_PIN
  #if Y_HOME_DIR < 0
    #define Y_MIN_PIN Y_STOP_PIN
    #define Y_MAX_PIN -1
  #else
    #define Y_MIN_PIN -1
    #define Y_MAX_PIN Y_STOP_PIN
  #endif
#endif

#ifdef Z_STOP_PIN
  #if Z_HOME_DIR < 0
    #define Z_MIN_PIN Z_STOP_PIN
    #define Z_MAX_PIN -1
  #else
    #define Z_MIN_PIN -1
    #define Z_MAX_PIN Z_STOP_PIN
  #endif
#endif

#endif
