/*
  temperature.cpp - temperature control
  Part of Marlin

 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)
 */


#include "Marlin.h"
#include "temperature.h"
#include "watchdog.h"
#include "temperature_ADS101X.h"
#include "fan_driver.h"
#include "i2c_onewire_ds2482.h"
#include "PowerBudgetManagement.h"

//===========================================================================
//============================= public variables ============================
//===========================================================================
TemperaturePID hotend_pid[EXTRUDERS];
TemperaturePID heated_bed_pid;
PowerBudgetManagement pwr(EXTRUDERS);

int current_temperature_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS_INIT(0);
float current_temperature[EXTRUDERS] = ARRAY_BY_EXTRUDERS_INIT(0.0);
int current_temperature_bed_raw = 0;
float current_temperature_bed = 0.0;
bool allow_temperature_sensor_errors[EXTRUDERS] = ARRAY_BY_EXTRUDERS_INIT(true);
bool hotend_present[EXTRUDERS] = ARRAY_BY_EXTRUDERS_INIT(false);
//Heater output accumulators, this is used to have M105 report the average heating output sinds last report instead of just the "current" output which fluctuates too fast.
uint16_t heater_output_accumulator[EXTRUDERS] = ARRAY_BY_EXTRUDERS_INIT(0);
uint16_t heater_output_accumulator_counter[EXTRUDERS] = ARRAY_BY_EXTRUDERS_INIT(0);
uint16_t bed_heater_output_accumulator = 0;
uint16_t bed_heater_output_accumulator_counter = 0;

bool stop_heaters_pwm = false;

//===========================================================================
//============================ private variables ============================
//===========================================================================
static volatile bool temp_meas_ready = false;

// 60 degC is hot but usually not too hot to hold  and it is also not a expected ambient temperature.
#define HOTEND_HUMAN_TOUCHABLE_TEMPERATURE 60

// Init min and max temp with extreme values to prevent false errors during startup
static int minttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS(HEATER_0_RAW_LO_TEMP, HEATER_1_RAW_LO_TEMP, HEATER_2_RAW_LO_TEMP);
static int maxttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS(HEATER_0_RAW_HI_TEMP, HEATER_1_RAW_HI_TEMP, HEATER_2_RAW_HI_TEMP);
static uint8_t temp_error_cnt[EXTRUDERS] = ARRAY_BY_EXTRUDERS_INIT(0);
static uint8_t temp_error_bed_cnt = 0;

static void min_temp_error(uint8_t e);
static void max_temp_error(uint8_t e);

#ifdef BED_MAXTEMP
  static int bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
//static int bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP; /* No bed mintemp error implemented?!? */
#endif

static unsigned long max_heating_start_millis[EXTRUDERS] = ARRAY_BY_EXTRUDERS_INIT(0);
static float max_heating_start_temperature[EXTRUDERS] = ARRAY_BY_EXTRUDERS_INIT(0);

static void *heater_ttbl_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS( (void *)HEATER_0_TEMPTABLE, (void *)HEATER_1_TEMPTABLE, (void *)HEATER_2_TEMPTABLE );
static uint8_t heater_ttbllen_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN );

static float analog2temp(int raw, uint8_t e);
static float analog2tempBed(int raw);
static void updateTemperaturesFromRawValues();

#define ERROR_SETTLING_TIME_MILLIS 10
#define ERROR_COUNT_INCREASE 10
#define ERROR_COUNT_INCREASE_NOT_PRESENT 4
#define ERROR_COUNT_DECREASE 1
#define ERROR_COUNT_BED_TEMP_OVERFLOW_TRIGGER   20          // The number of bed temperature errors before a bed temperature overflow will be triggered

// Save the last pwm level of the heaters as the bed has to be adjusted (possibly) the moment another heater is set.
// Because manage_heaters() is not guaranteed to be called fast enough, save the last PWM level of the heaters ...
static uint8_t last_pwm_output[EXTRUDERS] = { 0 };



static void setHeaterOutput(int8_t heater_nr, uint8_t output);
//===========================================================================
//================================ Functions ================================
//===========================================================================

void setAllowHotendRemoval(uint8_t extruder, bool allow_removal)
{
    if (extruder >= EXTRUDERS)
        return;
    if (allow_removal)
    {
        if (current_temperature[extruder] > 70)
            return;
        if (hotend_pid[extruder].getTargetTemperature() != 0.0)
            return;
    }
    allow_temperature_sensor_errors[extruder] = allow_removal;
}

/// Safety feature for nozzle temperature.
/// When the output has been set to maximum for too long a time without the temperature increasing,
/// then most likely we have a hardware problem and should disable the heaters.
void nozzle_temperature_watchdog(int e)
{
    if (last_pwm_output[e] == PID_MAX)
    {
        if (current_temperature[e] - max_heating_start_temperature[e] > MAX_HEATING_TEMPERATURE_INCREASE)
        {
            max_heating_start_millis[e] = 0;
        }
        if (max_heating_start_millis[e] == 0)
        {
            max_heating_start_millis[e] = millis();
            max_heating_start_temperature[e] = current_temperature[e];
        }
        else if (millis() - max_heating_start_millis[e] > MAX_HEATING_CHECK_MILLIS)
        {
            //Did not heat up MAX_HEATING_TEMPERATURE_INCREASE in MAX_HEATING_CHECK_MILLIS while the PID was at the maximum.
            //Potential problems could be that the heater is not working, or the temperature sensor is not measuring what the heater is heating.
            disable_all_heaters();
            Stop(STOP_REASON_HEATER_ERROR(e));
        }
    }
    else
    {
        max_heating_start_millis[e] = 0;
    }
}

void manage_heater()
{
  if (!temp_meas_ready)
  {
    return;
  }

  updateTemperaturesFromRawValues();

  for (uint8_t e = 0; e < EXTRUDERS; e++)
  {
      // When heaters should not PWM, then disable the extruders power so this is available for the bed.
      if (stop_heaters_pwm)
      {
          pwr.setRequestedHeaterOutput(e, 0);
      }
      else
      {
          pwr.setRequestedHeaterOutput(e, hotend_pid[e].update(current_temperature[e]));
      }

      nozzle_temperature_watchdog(e);
  }
  pwr.setRequestedBedOutput(heated_bed_pid.update(current_temperature_bed));
  pwr.setCurrentBedTemperature(current_temperature_bed);
  for (uint8_t e = 0; e < EXTRUDERS; e++)
  {
      setHeaterOutput(e, pwr.getActualHeaterOutput(e));
  }
  setHeaterOutput(-1, pwr.getActualBedOutput());
  

  // which fan pins need to be turned on?
  uint8_t fanState = 0;
  for (uint8_t e = 0; e < EXTRUDERS; e++)
  {
    hotend_present[e] = (i2cOneWireReset(e) || current_temperature[e] != MAGIC_NO_PT100_CONNECTED_VALUE);

    // On reboot a hotend cartridge may be present and hot, if so enable it so the fan is turned on!
    // The rationale for not using EXTRUDER_AUTO_FAN_TEMPERATURE is that it is reasonable to expect 40 degC ambient temperature.
    // It would also be annoying to have temperature error warnings(reboot to clear) if you remove your cold hotends at ambient temp > 40 degC.
    if (hotend_present[e] && current_temperature[e] >= HOTEND_HUMAN_TOUCHABLE_TEMPERATURE + 2 )
        allow_temperature_sensor_errors[e] = false;

    if ((current_temperature[e] > EXTRUDER_AUTO_FAN_TEMPERATURE) && !allow_temperature_sensor_errors[e])
      fanState |= 1;
    if ((current_temperature[e] < EXTRUDER_AUTO_FAN_TEMPERATURE - 5) || allow_temperature_sensor_errors[e])
      fanState |= 2;
  }

  if (fanState & 1)
    setHotendCoolingFanSpeed(255);
  else if (fanState & 2)
    setHotendCoolingFanSpeed(0);


}

static void setHeaterOutput(int8_t heater_nr, uint8_t output)
{
    if (heater_nr >= 0) // is always the nozzles
    {
        // When this hotend is allowed to be removed, for the output to be always off.
        // Extra safety as normally the target temperature is also forced to 0.
        if (allow_temperature_sensor_errors[heater_nr])
            output = 0;
        
        //If the heaters are locked to "off" we are active leveling, and thus should not turn on the PWM of hotends.
        if (stop_heaters_pwm)
        {
            output = 0;
        }
        last_pwm_output[heater_nr] = output;
        heater_output_accumulator[heater_nr] += output;
        heater_output_accumulator_counter[heater_nr] += 1;

        if (heater_nr == 0)
            analogWrite(HEATER_0_PIN, output);
#if EXTRUDERS > 1
        else if (heater_nr == 1)
            analogWrite(HEATER_1_PIN, output);
#endif
#if EXTRUDERS > 2
        else if (heater_nr == 2)
            analogWrite(HEATER_2_PIN, output);
#endif
    }
    else// (heater_nr < 0) is always the bed.
    {
        //If the heaters are locked to "off" we are active leveling, and thus should not turn on the PWM of bed.
        //We can however, put the bed on 100% output.
        if (stop_heaters_pwm && output != PID_MAX)
        {
            output = 0;
        }

        analogWrite(HEATER_BED_PIN, output);
        bed_heater_output_accumulator += output;
        bed_heater_output_accumulator_counter += 1;
    }
}

#define PGM_RD_W(x)   ((short)pgm_read_word(&x))
static float analog2temp(int raw, uint8_t e)
{
    float celsius = 0;
    uint8_t i;
    short (*tt)[][2] = (short (*)[][2])(heater_ttbl_map[e]);

    for (i = 1; i < heater_ttbllen_map[e]; i++)
    {
        if (PGM_RD_W((*tt)[i][0]) > raw)
        {
            celsius = PGM_RD_W((*tt)[i - 1][1]) +
                (raw - PGM_RD_W((*tt)[i - 1][0])) *
                (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i - 1][1])) /
                (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i - 1][0]));
            break;
        }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e])
        celsius = PGM_RD_W((*tt)[i - 1][1]);

    return celsius;
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
static float analog2tempBed(int raw)
{
    float celsius = 0;
    byte i;

    for (i = 1; i < BEDTEMPTABLE_LEN; i++)
    {
      if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw)
      {
        celsius  = PGM_RD_W(BEDTEMPTABLE[i - 1][1]) +
                   (raw - PGM_RD_W(BEDTEMPTABLE[i - 1][0])) *
                   (float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i - 1][1])) /
                   (float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i - 1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == BEDTEMPTABLE_LEN) celsius = PGM_RD_W(BEDTEMPTABLE[i - 1][1]);

    return celsius;
}

/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues()
{
    for (uint8_t e = 0; e < EXTRUDERS; e++)
    {
        current_temperature[e] = analog2temp(current_temperature_raw[e], e);
    }
    current_temperature_bed = analog2tempBed(current_temperature_bed_raw);
    // Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();

    CRITICAL_SECTION_START;
    temp_meas_ready = false;
    CRITICAL_SECTION_END;
}

/**
 * Initialize the temperature manager
 * The manager is implemented by periodic calls to manage_heater()
 */
void tp_init()
{
#if defined(HEATER_0_USES_ADS101X) || defined(HEATER_1_USES_ADS101X) || defined(HEATER_2_USES_ADS101X) || defined(BED_USES_ADS101X)
    initTemperatureADS101X();
#endif

  hotend_pid[0].setControlRange(HEATER_0_MINTEMP, HEATER_0_MAXTEMP);
#if EXTRUDERS > 1
  hotend_pid[1].setControlRange(HEATER_1_MINTEMP, HEATER_1_MAXTEMP);
#endif
#if EXTRUDERS > 2
  hotend_pid[2].setControlRange(HEATER_2_MINTEMP, HEATER_2_MAXTEMP);
#endif
  heated_bed_pid.setControlRange(BED_MINTEMP, BED_MAXTEMP);

  #if defined(HEATER_0_PIN) && (HEATER_0_PIN > -1)
    SET_OUTPUT(HEATER_0_PIN);
  #endif
  #if defined(HEATER_1_PIN) && (HEATER_1_PIN > -1)
    SET_OUTPUT(HEATER_1_PIN);
  #endif
  #if defined(HEATER_2_PIN) && (HEATER_2_PIN > -1)
    SET_OUTPUT(HEATER_2_PIN);
  #endif
  #if defined(HEATER_BED_PIN) && (HEATER_BED_PIN > -1)
    SET_OUTPUT(HEATER_BED_PIN);
  #endif

  // Set analog inputs
  ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIF | 0x07;
  DIDR0 = 0;
  #ifdef DIDR2
    DIDR2 = 0;
  #endif
  #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
    #if TEMP_0_PIN < 8
       DIDR0 |= 1 << TEMP_0_PIN;
    #else
       DIDR2 |= 1<<(TEMP_0_PIN - 8);
    #endif
  #endif
  #if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1)
    #if TEMP_1_PIN < 8
       DIDR0 |= 1<<TEMP_1_PIN;
    #else
       DIDR2 |= 1<<(TEMP_1_PIN - 8);
    #endif
  #endif
  #if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1)
    #if TEMP_2_PIN < 8
       DIDR0 |= 1 << TEMP_2_PIN;
    #else
       DIDR2 |= 1<<(TEMP_2_PIN - 8);
    #endif
  #endif
  #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
    #if TEMP_BED_PIN < 8
       DIDR0 |= 1<<TEMP_BED_PIN;
    #else
       DIDR2 |= 1<<(TEMP_BED_PIN - 8);
    #endif
  #endif

  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  OCR0B = 128;
  TIMSK0 |= (1<<OCIE0B);
  
#ifdef HEATER_0_MINTEMP
  while (analog2temp(minttemp_raw[0], 0) < HEATER_0_MINTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    minttemp_raw[0] += OVERSAMPLENR;
#else
    minttemp_raw[0] -= OVERSAMPLENR;
#endif
  }
#endif //MINTEMP
#ifdef HEATER_0_MAXTEMP
  while (analog2temp(maxttemp_raw[0], 0) > HEATER_0_MAXTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    maxttemp_raw[0] -= OVERSAMPLENR;
#else
    maxttemp_raw[0] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP

#if (EXTRUDERS > 1) && defined(HEATER_1_MINTEMP)
  while (analog2temp(minttemp_raw[1], 1) < HEATER_1_MINTEMP) {
#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
    minttemp_raw[1] += OVERSAMPLENR;
#else
    minttemp_raw[1] -= OVERSAMPLENR;
#endif
  }
#endif // MINTEMP 1
#if (EXTRUDERS > 1) && defined(HEATER_1_MAXTEMP)
  while (analog2temp(maxttemp_raw[1], 1) > HEATER_1_MAXTEMP) {
#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
    maxttemp_raw[1] -= OVERSAMPLENR;
#else
    maxttemp_raw[1] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP 1

#if (EXTRUDERS > 2) && defined(HEATER_2_MINTEMP)
  while (analog2temp(minttemp_raw[2], 2) < HEATER_2_MINTEMP) {
#if HEATER_2_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
    minttemp_raw[2] += OVERSAMPLENR;
#else
    minttemp_raw[2] -= OVERSAMPLENR;
#endif
  }
#endif //MINTEMP 2
#if (EXTRUDERS > 2) && defined(HEATER_2_MAXTEMP)
  while (analog2temp(maxttemp_raw[2], 2) > HEATER_2_MAXTEMP) {
#if HEATER_2_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
    maxttemp_raw[2] -= OVERSAMPLENR;
#else
    maxttemp_raw[2] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP 2

#ifdef BED_MINTEMP
  /* No bed MINTEMP error implemented?!? */ /*
  while (analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
    bed_minttemp_raw += OVERSAMPLENR;
#else
    bed_minttemp_raw -= OVERSAMPLENR;
#endif
  }
  */
#endif //BED_MINTEMP
#ifdef BED_MAXTEMP
  while (analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
    bed_maxttemp_raw -= OVERSAMPLENR;
#else
    bed_maxttemp_raw += OVERSAMPLENR;
#endif
  }
#endif //BED_MAXTEMP
}


void disable_all_heaters()
{
    for(uint8_t e=0; e<EXTRUDERS; e++)
        hotend_pid[e].setTargetTemperature(0);
    heated_bed_pid.setTargetTemperature(0);

    #if defined(HEATER_0_PIN) && HEATER_0_PIN > -1
    WRITE(HEATER_0_PIN, LOW);
    #endif
    #if defined(HEATER_1_PIN) && HEATER_1_PIN > -1
    WRITE(HEATER_1_PIN,LOW);
    #endif
    #if defined(HEATER_2_PIN) && HEATER_2_PIN > -1
    WRITE(HEATER_2_PIN,LOW);
    #endif

    #if defined(HEATER_BED_PIN) && HEATER_BED_PIN > -1
    WRITE(HEATER_BED_PIN,LOW);
    #endif
}

void max_temp_error(uint8_t e) {
  disable_all_heaters();
  Stop(STOP_REASON_MAXTEMP(e));
}

void min_temp_error(uint8_t e) {
  disable_all_heaters();
  Stop(STOP_REASON_MINTEMP(e));
}

void bed_max_temp_error(void) {
#if HEATER_BED_PIN > -1
  WRITE(HEATER_BED_PIN, 0);
#endif
  Stop(STOP_REASON_MAXTEMP_BED);
}

/**
 * Timer 0 is shared with millies
 *  - Manage PWM to all the heaters and fan
 *  - Update the raw temperature values
 *  - Check new temperature values for MIN/MAX errors
 */
ISR(TIMER0_COMPB_vect)
{
  // These variables are only accessible from the ISR, but static, so they don't lose their value
  static unsigned char temp_count = 0;
  static unsigned long raw_temp_0_value = 0;
#if EXTRUDERS > 1
  static unsigned long raw_temp_1_value = 0;
#endif
#if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1) && EXTRUDERS > 2
  static unsigned long raw_temp_2_value = 0;
#endif
  static unsigned long raw_temp_bed_value = 0;
  static unsigned char temp_state = 5;
  #if defined(HEATER_0_USES_ADS101X) || defined(HEATER_1_USES_ADS101X) || defined(HEATER_2_USES_ADS101X) || defined(BED_USES_ADS101X)
  static uint8_t ads101x_state = 0;
  #endif

  #if defined(HEATER_0_USES_ADS101X) || defined(HEATER_1_USES_ADS101X) || defined(HEATER_2_USES_ADS101X) || defined(BED_USES_ADS101X)
  if (temperatureADS101XReady())
  {
    switch (ads101x_state)
    {
    case 0:
      temperatureADS101XSetupAIN0();
      ads101x_state = 1;
      break;
    case 1:
      //Make sure the ADS101X has time to get a sample.
      ads101x_state = 2;
      break;
    case 2:
      temperatureADS101XRequestResult();
      temperatureADS101XSetupAIN1();
      ads101x_state = 3;
      break;
    case 3:
      #ifdef HEATER_0_USES_ADS101X
      raw_temp_0_value = long(temperatureADS101XGetResult()) * OVERSAMPLENR;
      #endif
      ads101x_state = 4;
      break;
    case 4:
      temperatureADS101XRequestResult();
      ads101x_state = 5;
      break;
    case 5:
      #ifdef HEATER_1_USES_ADS101X
      raw_temp_1_value = long(temperatureADS101XGetResult()) * OVERSAMPLENR;
      #endif
      ads101x_state = 6;
      break;
    }
  }
  #endif

  switch (temp_state) {
    case 1: // Measure TEMP_0
      #ifdef HEATER_0_USES_ADS101X
      #elif defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
        raw_temp_0_value += ADC;
      #endif
      // Prepare TEMP_BED
      #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
        #if TEMP_BED_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_BED_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      temp_state = 2;
      break;
    case 2: // Measure TEMP_BED
      #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
        raw_temp_bed_value += ADC;
      #endif
      // Prepare TEMP_1
      #if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1) && EXTRUDERS > 1
        #if TEMP_1_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_1_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      temp_state = 3;
      break;
    case 3: // Measure TEMP_1
      #ifdef HEATER_1_USES_ADS101X
      #elif defined(TEMP_1_PIN) && (TEMP_1_PIN > -1) && EXTRUDERS > 1
        raw_temp_1_value += ADC;
      #endif
      // Prepare TEMP_2
      #if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1) && EXTRUDERS > 2
        #if TEMP_2_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_2_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      temp_state = 4;
      break;
    case 4: // Measure TEMP_2
      #if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1) && EXTRUDERS > 2
        raw_temp_2_value += ADC;
      #endif
      temp_count++;
      // Fall through to state 0
    case 0: // Prepare TEMP_0
      #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
        #if TEMP_0_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      temp_state = 1;
      break;
    case 5: //Startup, delay initial temp reading a tiny bit so the hardware can settle.
      temp_state = 0;
      break;
//    default:
//      SERIAL_ERROR_START;
//      SERIAL_ERRORLNPGM("Temp measurement error!");
//      break;
  }


  if (temp_count >= OVERSAMPLENR) // 8 ms * 16 = 128ms.
  {
    if (!temp_meas_ready) //Only update the raw values if they have been read. Else we could be updating them during reading.
    {
      current_temperature_raw[0] = raw_temp_0_value;
#if EXTRUDERS > 1
      current_temperature_raw[1] = raw_temp_1_value;
#endif
#if EXTRUDERS > 2
      current_temperature_raw[2] = raw_temp_2_value;
#endif
      current_temperature_bed_raw = raw_temp_bed_value;
    }

    temp_meas_ready = true;
    temp_count = 0;
    raw_temp_0_value = 0;
#if EXTRUDERS > 1
    raw_temp_1_value = 0;
#endif
#if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1) && EXTRUDERS > 2
    raw_temp_2_value = 0;
#endif
    raw_temp_bed_value = 0;
    #if defined(HEATER_0_USES_ADS101X) || defined(HEATER_1_USES_ADS101X) || defined(HEATER_2_USES_ADS101X) || defined(BED_USES_ADS101X)
    ads101x_state = 0;
    #endif

    if (!allow_temperature_sensor_errors[0])
    {
#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
        if (current_temperature_raw[0] <= maxttemp_raw[0]) {
#else
        if (current_temperature_raw[0] >= maxttemp_raw[0]) {
#endif
            if (hotend_present[0])
                temp_error_cnt[0] += ERROR_COUNT_INCREASE;
            else
                temp_error_cnt[0] += ERROR_COUNT_INCREASE_NOT_PRESENT;

            if (temp_error_cnt[0] > ERROR_SETTLING_TIME_MILLIS * ERROR_COUNT_INCREASE)
                max_temp_error(0);
        }
#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
        else if (current_temperature_raw[0] >= minttemp_raw[0]) {
#else
        else if (current_temperature_raw[0] <= minttemp_raw[0]) {
#endif
            temp_error_cnt[0] += ERROR_COUNT_INCREASE;
            if (temp_error_cnt[0] > ERROR_SETTLING_TIME_MILLIS * ERROR_COUNT_INCREASE)
                min_temp_error(0);
        } else if (temp_error_cnt[0] > 0)
        {
            temp_error_cnt[0] -= ERROR_COUNT_DECREASE;
        }
    }
    else
    {
        temp_error_cnt[0] = 0;
    }
#if EXTRUDERS > 1
    if (!allow_temperature_sensor_errors[1])
    {
#if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
        if (current_temperature_raw[1] <= maxttemp_raw[1]) {
#else
        if (current_temperature_raw[1] >= maxttemp_raw[1]) {
#endif
            if (hotend_present[1])
                temp_error_cnt[1] += ERROR_COUNT_INCREASE;
            else
                temp_error_cnt[1] += ERROR_COUNT_INCREASE_NOT_PRESENT;

            if (temp_error_cnt[1] > ERROR_SETTLING_TIME_MILLIS * ERROR_COUNT_INCREASE)
                max_temp_error(1);
        }
#if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
        if (current_temperature_raw[1] >= minttemp_raw[1]) {
#else
        if (current_temperature_raw[1] <= minttemp_raw[1]) {
#endif
            temp_error_cnt[1] += ERROR_COUNT_INCREASE;
            if (temp_error_cnt[1] > ERROR_SETTLING_TIME_MILLIS * ERROR_COUNT_INCREASE)
                min_temp_error(1);
        }else if (temp_error_cnt[1] > 0)
        {
            temp_error_cnt[1] -= ERROR_COUNT_DECREASE;
        }
    }
    else
    {
        temp_error_cnt[1] = 0;
    }
#endif
#if EXTRUDERS > 2
    if (!allow_temperature_sensor_errors[2])
    {
#if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
        if (current_temperature_raw[2] <= maxttemp_raw[2]) {
#else
        if (current_temperature_raw[2] >= maxttemp_raw[2]) {
#endif
            if (hotend_present[2])
                temp_error_cnt[2] += ERROR_COUNT_INCREASE;
            else
                temp_error_cnt[2] += ERROR_COUNT_INCREASE_NOT_PRESENT;

            if (temp_error_cnt[2] > ERROR_SETTLING_TIME_MILLIS * ERROR_COUNT_INCREASE)
                max_temp_error(2);
        }
#if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
        else if (current_temperature_raw[2] >= minttemp_raw[2]) {
#else
        else if (current_temperature_raw[2] <= minttemp_raw[2]) {
#endif
            temp_error_cnt[2] += ERROR_COUNT_INCREASE;
            if (temp_error_cnt[2] > ERROR_SETTLING_TIME_MILLIS * ERROR_COUNT_INCREASE)
                min_temp_error(2);
        }else if (temp_error_cnt[2] > 0)
        {
            temp_error_cnt[2] -= ERROR_COUNT_DECREASE;
        }
    }
    else
    {
        temp_error_cnt[2] = 0;
    }
#endif

  /* No bed MINTEMP error? */
#if defined(BED_MAXTEMP) && (TEMP_SENSOR_BED != 0)
# if HEATER_BED_RAW_LO_TEMP > HEATER_BED_RAW_HI_TEMP
    if (current_temperature_bed_raw <= bed_maxttemp_raw)
#else
    if (current_temperature_bed_raw >= bed_maxttemp_raw)
#endif
    {
        temp_error_bed_cnt++;
        if (temp_error_bed_cnt > ERROR_COUNT_BED_TEMP_OVERFLOW_TRIGGER)
        {
            heated_bed_pid.setTargetTemperature(0);
            bed_max_temp_error();
        }
    }
    else if (temp_error_bed_cnt > 0)
    {
        temp_error_bed_cnt--;
    }
#endif
  }
}
