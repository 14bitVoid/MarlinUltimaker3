// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>


#include "fastio.h"
#include "Configuration.h"
#include "pins.h"
#include "language.h"

#ifndef AT90USB
#define  HardwareSerial_h // trick to disable the standard HWserial
#endif

#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
  //Arduino < 1.0.0 does not define this, so we need to do it ourselfs
# define analogInputToDigitalPin(p) ((p) + A0)
#endif

#include "MarlinSerial.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"

#ifdef AT90USB
  #define MYSERIAL Serial
#else
  #define MYSERIAL MSerial
#endif

#define SERIAL_PROTOCOL(x) MYSERIAL.print(x);
#define SERIAL_PROTOCOL_F(x,y) MYSERIAL.print(x,y);
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(PSTR(x));
#define SERIAL_PROTOCOLLN(x) do {MYSERIAL.print(x);MYSERIAL.write('\n');} while(0)
#define SERIAL_PROTOCOLLNPGM(x) do{serialprintPGM(PSTR(x));MYSERIAL.write('\n');} while(0)
#define SERIAL_PROTOCOL_OK() do{serialprintPGM(PSTR(MSG_OK " N")); MYSERIAL.print(gcode_N[command_buffer_index_read]); serialprintPGM(PSTR(" P")); MYSERIAL.print(int(BLOCK_BUFFER_SIZE - movesplanned() - 1)); }while(0)
#define SERIAL_PROTOCOL_OK_LN() do{SERIAL_PROTOCOL_OK(); MYSERIAL.println(); }while(0)

const char errormagic[] PROGMEM ="\nError:";
const char perrormagic[] PROGMEM ="ProtoErr:";
const char echomagic[] PROGMEM ="echo:";
#define SERIAL_ERROR_START serialprintPGM(errormagic);
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_PROTOCOL_ERROR_START serialprintPGM(perrormagic);
#define SERIAL_PROTOCOL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_PROTOCOL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_PROTOCOL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_PROTOCOL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START serialprintPGM(echomagic);
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))

void serial_echopair_P(const char *s_P, float v);
void serial_echopair_P(const char *s_P, double v);
void serial_echopair_P(const char *s_P, unsigned long v);


//things to write to serial from Programmemory. saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MYSERIAL.write(ch);
    ch=pgm_read_byte(++str);
  }
}


void get_command();
void process_commands();

void manage_inactivity();

#if defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1
  #define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#else
  #define enable_x() ;
  #define disable_x() ;
#endif

#if defined(Y_ENABLE_PIN) && Y_ENABLE_PIN > -1
  #define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#else
  #define enable_y() ;
  #define disable_y() ;
#endif

#if defined(Z_ENABLE_PIN) && Z_ENABLE_PIN > -1
  #define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
  #define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
#else
  #define enable_z() ;
  #define disable_z() ;
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
  #define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e0()  /* nothing */
  #define disable_e0() /* nothing */
#endif

#if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
  #define enable_e1() WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e1() WRITE(E1_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e1()  /* nothing */
  #define disable_e1() /* nothing */
#endif

#if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
  #define enable_e2() WRITE(E2_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e2() WRITE(E2_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e2()  /* nothing */
  #define disable_e2() /* nothing */
#endif


#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E
enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};


void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
void prepare_move();
#define STOP_REASON_MAXTEMP(e)              (1 + e * 50)
#define STOP_REASON_MINTEMP(e)              (2 + e * 50)
#define STOP_REASON_MAXTEMP_BED             3
#define STOP_REASON_HEATER_ERROR(e)         (4 + e * 50)
#define STOP_REASON_Z_ENDSTOP_BROKEN_ERROR  5
#define STOP_REASON_Z_ENDSTOP_STUCK_ERROR   6
#define STOP_REASON_XY_ENDSTOP_BROKEN_ERROR 7
#define STOP_REASON_XY_ENDSTOP_STUCK_ERROR  8
#define STOP_REASON_I2C_HEAD_COMM_ERROR     9
#define STOP_REASON_I2C_COMM_ERROR          10
#define STOP_REASON_SAFETY_TRIGGER          11
#define STOP_REASON_GCODE                   12
#define STOP_REASON_SERIAL_INPUT_TIMEOUT    13
#define STOP_REASON_POSITION_ERROR          14
void Stop(uint8_t reasonNr);

bool IsStopped();
uint8_t StoppedReason();

void clear_command_queue();
void enquecommand(const char *cmd); //put an ascii command at the end of the current buffer.
void enquecommand_P(const char *cmd); //put an ascii command at the end of the current buffer, read from flash
bool is_command_queued();
uint8_t commands_queued();
void prepare_arc_move(char isclockwise);

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val);
#endif

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern int extrudemultiply; // Sets extrude multiply factor (in percent)
extern float current_position[NUM_AXIS] ;
extern float add_homing[3];
extern float min_pos[3];
extern float max_pos[3];
extern uint8_t target_fan_speed;

extern unsigned long starttime;
extern unsigned long stoptime;

// Handling multiple extruders pins
extern uint8_t active_extruder;
// handle extruder offset in steps to avoid floating point errors accumulating
// (and move handling to the planner)
extern long extruder_offset[NUM_AXIS-1][EXTRUDERS];

void pulseLeds();

#if EXTRUDERS > 3
  # error Unsupported number of extruders
#elif EXTRUDERS > 2
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2, v3 }
#elif EXTRUDERS > 1
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2 }
#else
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1 }
#endif
# define ARRAY_BY_EXTRUDERS_INIT(v) ARRAY_BY_EXTRUDERS(v, v, v)

#endif
