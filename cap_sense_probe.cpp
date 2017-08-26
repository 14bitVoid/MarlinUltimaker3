/*
   Routines to do z-probing using the capacitive sensor. The bed is raised
   slowly until it hits the nozzle. At that point, the measured values will
   not change as much anymore. The trick is to find this point and then stop
   the motion of the bed. The data does not show a linear progression, as
   the capacitance between two conductors is proportional to the inverse of
   the distance.

   The math involved is documented at the appropriate places in this file.

   NOTE: most of the expressions in this file contain hand-crafted typecasts
   to prevent overflows while maintaining precision. Do not modify any of
   these without extensive testing.
*/

#include "Marlin.h"
#include "i2c_capacitance_FDC1004.h"
#include "cap_sense_probe.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "limits.h"

#define SENSOR_MULT 1

#define SAMPLE_BUFFER_1_SIZE CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT1
#define SAMPLE_BUFFER_2_SIZE CONFIG_BED_LEVEL_SENSOR_SAMPLE_COUNT2

// Capacitive sensor samples will always have a bit of noise, but sometimes
// an outlier occurs (due to a bump or electrical noise such as ESD). Such
// samples can be rejected for reliability. They will be replaced by the
// average value of the three preceding samples. REJECT_THRESHOLD defines
// how much a sample is allowed to deviate from the mean value of the sample
// buffer, expressed in variance (square of standard deviation). A good
// starting point is 4 standard deviations, given as a variance of (4 * 4).
// 4 standard deviations may seem rather large, but there is a trend in the
// data (mostly increasing). It would be better to calculate the deviation
// from the trend, but the algorithm does not go that far.
#define REJECT_THRESHOLD (4*4)

// The impact of a rejected sample decreases over time. A single rejection
// should not matter, while a burst is an indication of trouble. The
// following values define when the data is considered to be so unreliable
// that the z-probe should be aborted. A counter is incremented by 1 for
// every rejected sample, while it is multiplied by REJECT_CONSTANT for
// every accepted sample (IIR filter). When the counter value reaches
// REJECT_COUNT_MAX, the run is aborted.
#define REJECT_COUNT_MAX 10
#define REJECT_CONSTANT 0.9

// ERROR_HEIGHT is a safe height that won't break the printer. Opinicus
// homes the bed after this for a retry (a number of times), since a move is
// planned to the detected height after probing (very useful for debugging,
// now it moves to ERROR_HEIGHT before homing which is also very clear
// behaviour indicating an error).
#define ERROR_HEIGHT 150

// buffer index handling (note % behavior for negative numbers)
#define BUFMOD(x) (((x) + SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE) % (SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE))

// each state explained:
// INIT     call initialization code and move to FILLING state
// FILLING  block analysis until the buffers have been filled
// WORKING  perform analysis
// DONE     analysis is complete and has found the inflection point
// CONTINUE collect some samples pressing the buildplate and nozzle together
//          a bit more after already being DONE (for debugging only)
// ERROR    a serious error has been detected; analysis will stop
enum probe_state_t { INIT = 0, FILLING, WORKING, DONE, CONTINUE, ERROR };

// work packet (storing global data)
struct WorkPacket
{
    enum probe_state_t state;
    float z;
    int debuglevel;             // from V option
    // An inflection point is a point on a curve where the curve changes from being concave to convex or vice versa.
    int64_t inflection;
    int64_t inflection_max;    // the maximum inflection so far
    float reject_count;        // nr of rejected samples; decreased for every good sample by multiplication with REJECT_CONSTANT
};

static struct WorkPacket workPacket;

struct stat_t {
    long sum_of_x;	    // sum of x values
    long sum_of_squared_x;  // sum of squares of x values
    long sum_of_xy;	    // sum of xy products
    long sum_of_y;	    // sum of y values
    long sum_of_squared_y;  // sum of squares of y values
};

static long capacitive_baseline = 0;

// sum_of_squared_x, sum_of_xy and sum_of_squared_y are prone to overflows
static long safeAdd(long a, long b)
{
    if (a > 0 && b > LONG_MAX - a)
    {
        if (workPacket.debuglevel > 0)
            MSerial.println("WARNING:CAPACITIVE_SENSOR_ERROR: integer overflow");// (%lu + %lu)\n", a, b);
        workPacket.state = ERROR;
        return 0;
    }
    return a + b;
}

// add a data point to a statistics struct
static void statPushSample(struct stat_t *stat, long x, long y)
{
    stat->sum_of_x += x;

    stat->sum_of_squared_x = safeAdd(stat->sum_of_squared_x, x * x);
    if (workPacket.state == ERROR)
    {
        if (workPacket.debuglevel > 0)
            MSerial.println("LOG:CAPACITIVE_SENSOR_ERROR - overflow while calculating sum_of_squared_x");
        return;
    }

    stat->sum_of_xy = safeAdd(stat->sum_of_xy, x * y);
    if (workPacket.state == ERROR)
    {
        if (workPacket.debuglevel > 0)
            MSerial.println("LOG:CAPACITIVE_SENSOR_ERROR - overflow while calculating sum_of_xy");
        return;
    }

    stat->sum_of_y  += y;

    stat->sum_of_squared_y = safeAdd(stat->sum_of_squared_y, y * y);
    if (workPacket.state == ERROR)
    {
        if (workPacket.debuglevel > 0)
            MSerial.println("LOG:CAPACITIVE_SENSOR_ERROR - overflow while calculating sum_of_squared_y");
        return;
    }
}

// remove a data point from a statistics struct
static void statPopSample(struct stat_t *stat, long x, long y)
{
    stat->sum_of_x  -= x;
    stat->sum_of_squared_x -= x * x;
    stat->sum_of_xy -= x * y;
    stat->sum_of_y  -= y;
    stat->sum_of_squared_y -= y * y;
}

static void processSample(uint16_t sample_value, float sample_z)
{
    // sample counter, also used as abscissa data for regression
    static long counter = -1;

    // data buffers (used as ring buffers)
    static uint16_t cbuf[SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE]; // capacitive data sample buffer
    static float    zbuf[SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE]; // z height buffer.
    static uint16_t write_pointer;

    // statistics
    static struct stat_t stat1, stat2;

    // slope of lines determined by linear regression over buffers (see math comment below)
    int64_t a1, a2;

    // x coordinate of intersection in terms of counter value
    static float intersection_sample_count;

    // x data for new sample (note that counter starts at -1 to allow pre-increment)
    if (workPacket.state == INIT)
        counter = -1; // workPacket.state is re-initialized below

    counter++;

    // initialization
    if (counter == 0)
    {
        workPacket.state = FILLING;
        memset(&stat1, 0, sizeof(struct stat_t));
        memset(&stat2, 0, sizeof(struct stat_t));
        write_pointer = 0;
        workPacket.inflection_max = 0;
        workPacket.reject_count = 0.0;
    }

    // anything to do?
    if (workPacket.state == DONE || workPacket.state == ERROR)
        return;

    // sample rejection logic -- skip this while filling the buffers to accumulate statistics
    if (workPacket.state == WORKING)
    {
        // calculate variance (calculation carefully crafted to maintain precision (casting to float immediately
        // will be more imprecise than dividing as integer and then casting to float) and avoid overflows)
        float variance2 = float(stat2.sum_of_squared_y - int64_t(stat2.sum_of_y) * stat2.sum_of_y / int64_t(SAMPLE_BUFFER_2_SIZE)) / float(SAMPLE_BUFFER_2_SIZE);
        // calculate distance from mean (ignoring slope)
        float distance2 = sample_value - float(stat2.sum_of_y) / SAMPLE_BUFFER_2_SIZE;

        // reject sample if it is an outlier
        if (distance2 * distance2 > REJECT_THRESHOLD * variance2)
        {
            int16_t sample_value_new;

            // replace aberrant sample with average of last three samples
            sample_value_new = int16_t((cbuf[BUFMOD(write_pointer - 1)] + cbuf[BUFMOD(write_pointer - 2)] + cbuf[BUFMOD(write_pointer - 3)]) / 3.0);

            if (workPacket.debuglevel > 0)
            {
                MSerial.print("* rejecting sample ");
                MSerial.print(sample_value);
                MSerial.print(" -> ");
                MSerial.println(sample_value_new);
            }
            sample_value = sample_value_new;
            workPacket.reject_count++;
            if (workPacket.reject_count > REJECT_COUNT_MAX)
            {
                if (workPacket.debuglevel > 0)
                    MSerial.println("LOG:CAPACITIVE_SENSOR_ERROR - reject limit exceeded - aborting");
                workPacket.state = ERROR;
                return;
            }
        }
        else
        {
            workPacket.reject_count *= REJECT_CONSTANT;
        }
    }

    // update statistics: remove old samples
    if (counter >= SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE)
        statPopSample(&stat1, counter - (SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE), cbuf[write_pointer]);
    if (counter >= SAMPLE_BUFFER_2_SIZE)
        statPopSample(&stat2, counter - SAMPLE_BUFFER_2_SIZE, cbuf[BUFMOD(write_pointer - SAMPLE_BUFFER_2_SIZE)]);

    // add new sample to buffer, overwriting old value
    cbuf[write_pointer] = sample_value;
    zbuf[write_pointer] = sample_z;
    write_pointer = BUFMOD(write_pointer + 1);

    // update statistics: add samples
    if (counter >= SAMPLE_BUFFER_2_SIZE)
        statPushSample(&stat1, counter - SAMPLE_BUFFER_2_SIZE, cbuf[BUFMOD(write_pointer - SAMPLE_BUFFER_2_SIZE - 1)]);
    statPushSample(&stat2, counter, sample_value);

    // check for overflows and abort if necessary
    if (workPacket.state == ERROR)
        return;

    // wait until buffer is filled
    if (counter < SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE - 1)
        return;

    if (counter == SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE - 1)
    {
        if (workPacket.debuglevel > 0)
            MSerial.print("* buffer filled\n");// (%ld)\n", counter);
        workPacket.state = WORKING;
    }

    /*
        Using linear least-squares regression, a (virtual) line is drawn
        through each buffer according to the equation

          y = a * x + b

        where a (slope) and b (intercept) are to be determined. Closed
        expressions exist for a and b:

              S(x*y) - 1/n * S(x) * S(y)   n * S(x*y) - S(x) * S(y)
          a = -------------------------- = ------------------------
               S(x^2) - 1/n * (S(x))^2      n * S(x^2) - (S(x))^2

              1/n * S(y) * S(x^2) - 1/n * S(x) * S(x*y)   S(y) * S(x^2) - S(x) * S(y)
          b = ----------------------------------------- = ---------------------------
                       S(x^2) - 1/n * (S(x))^2               n * S(x^2) - (S(x))^2

        where SAMPLE_BUFFER_1_SIZE and SAMPLE_BUFFER_2_SIZES are equivalent to n and
        S is to be read as sigma, the summation over a buffer, and x
        and y are x_i and y_i, respectively. This is convenient, because it
        allows us to calculate a and b based on streaming data:

          d1 = SAMPLE_BUFFER_1_SIZE * stat1.sum_of_squared_x - stat1.sum_of_x * stat1.sum_of_x;
          d2 = SAMPLE_BUFFER_2_SIZE * stat2.sum_of_squared_x - stat2.sum_of_x * stat2.sum_of_x;
          a1 = (SAMPLE_BUFFER_1_SIZE * stat1.sum_of_xy - stat1.sum_of_x * stat1.sum_of_y) / float(d1);
          a2 = (SAMPLE_BUFFER_2_SIZE * stat2.sum_of_xy - stat2.sum_of_x * stat2.sum_of_y) / float(d2);

        The denominators d1 and d2 represent n * Var(x). For the linear
        counter used here, these values are constant and equal for the two
        buffers when they are full. In this approach, the exact value is:

          n * Var(x) = n * S(x^2) - (S(x))^2 = 1/12 * n^2 * (n^2 - 1)

        For SAMPLE_BUFFER_1_SIZE = SAMPLE_BUFFER_2_SIZE = 50, the
        denominators are 520625. Since we are interested in the location of
        the intersection of the two lines, the denominators (and a good
        number of divisions) can be omitted from the analysis. See the next
        math comment for a proof.

        The numbers involved are large! The final results will usually fit
        in a long, so they can be cast before printing. The intermediate
        values can easily overflow a long, however.
    */

    // calculate slopes
    a1 = SAMPLE_BUFFER_1_SIZE * int64_t(stat1.sum_of_xy) - int64_t(stat1.sum_of_x) * stat1.sum_of_y;
    a2 = SAMPLE_BUFFER_2_SIZE * int64_t(stat2.sum_of_xy) - int64_t(stat2.sum_of_x) * stat2.sum_of_y;

    // inflection at point between SAMPLE_BUFFER_1_SIZE and SAMPLE_BUFFER_2_SIZE in buffer
    workPacket.inflection = a1 - a2;

    if (workPacket.inflection > workPacket.inflection_max)
    {
        // intercept (see math comment below)
        int64_t b1, b2;

        if (workPacket.debuglevel > 1)
        {
            MSerial.print("* new infmax ");
            MSerial.println((long)workPacket.inflection);
        }

        /*
            The math to calculate the intersection of two lines is:
              line 1: y1 = a1 * x + b1
              line 2: y2 = a2 * x + b2

              y1 = y2 ->
              a1 * x + b1 = a2 * x + b2 ->
              a1 * x - a2 * x = b2 - b1 ->
              (a1 - a2) * x = b2 - b1 ->
              x = (b2 - b1) / (a1 - a2)

            A common denominator in a and b drops out of the equation.
            The numbers involved can be huge: we're trying not to lose
            accuracy by using integers as long as possible. The difference
            between b2 and b1 will not be as large. The calculation below
            takes less than 150us on a 16-MHz ATmega2560.
        */

        // store inflection
        workPacket.inflection_max = workPacket.inflection;

        // calculate index of sample_z at peak, relative to x data (counter)
        b1 = stat1.sum_of_y * int64_t(stat1.sum_of_squared_x) - stat1.sum_of_x * int64_t(stat1.sum_of_xy);
        b2 = stat2.sum_of_y * int64_t(stat2.sum_of_squared_x) - stat2.sum_of_x * int64_t(stat2.sum_of_xy);
        intersection_sample_count = float(b2 - b1) / float(workPacket.inflection);

        if (workPacket.debuglevel > 1)
        {
            MSerial.print("* stat1.Sy ");
            MSerial.print(stat1.sum_of_y);
            MSerial.print(" Sx2 ");
            MSerial.print(stat1.sum_of_squared_x);
            MSerial.print(" Sx ");
            MSerial.print(stat1.sum_of_x);
            MSerial.print(" Sxy ");
            MSerial.println(stat1.sum_of_xy);
            MSerial.print("* stat2.Sy ");
            MSerial.print(stat2.sum_of_y);
            MSerial.print(" Sx2 ");
            MSerial.print(stat2.sum_of_squared_x);
            MSerial.print(" Sx ");
            MSerial.print(stat2.sum_of_x);
            MSerial.print(" Sxy ");
            MSerial.println(stat2.sum_of_xy);
            MSerial.print("* isc ");
            MSerial.print(intersection_sample_count);
            MSerial.print(" ctr ");
            MSerial.println(counter);
        }
    }

    /*
       inflection_max must be greater than CONFIG_BED_LEVELING_PEAK_DET1 *
       inflection_max before it is considered a real peak. Once the value
       has dropped below CONFIG_BED_LEVELING_PEAK_DET2 * inflection_max, we
       can be sure no higher values will happen.
       CONFIG_BED_LEVELING_PEAK_DET2 will be between 0 and 1 (typically
       0.75). Fractional math does not always work nicely with int64_t, so
       CONFIG_BED_LEVELING_PEAK_DET2 is split in a numerator and a
       denominator.
    */
    if (workPacket.inflection_max > CONFIG_BED_LEVELING_PEAK_DET1 &&
        workPacket.inflection < CONFIG_BED_LEVELING_PEAK_DET2N * (workPacket.inflection_max / CONFIG_BED_LEVELING_PEAK_DET2D))
    {
        float index;
        int offset, offset1;

        // peak found!
        workPacket.state = DONE;
        if (workPacket.debuglevel > 0)
            MSerial.println("* peak found");

        // find corresponding index in z buffer
        index = intersection_sample_count - (float(counter) - (SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE - 1));

        // index is offset from write_pointer in circular buffer
        offset  = BUFMOD(write_pointer + (int)index);
        offset1 = BUFMOD(write_pointer + (int)index + 1);

        if (workPacket.debuglevel > 1)
        {
            MSerial.print("* index ");
            MSerial.print(index);
            MSerial.print(" wptr ");
            MSerial.print(write_pointer);
            MSerial.print(" off ");
            MSerial.println(offset);
        }

        if (index < 0)
        {
            workPacket.state = ERROR;
            if (workPacket.debuglevel > 0)
                MSerial.println("LOG:CAPACITIVE_SENSOR_ERROR - peak is no longer in buffer!");
            return;
        }

        if (index >= SAMPLE_BUFFER_1_SIZE + SAMPLE_BUFFER_2_SIZE)
        {
            workPacket.state = ERROR;
            if (workPacket.debuglevel > 0)
                MSerial.println("LOG:CAPACITIVE_SENSOR_ERROR - peak is in the future?!");
            return;
        }

        // linear interpolation to find z
        // (this is y = y0 + (y1 - y0)/(x1 - x0)*(x - x0) where x1 - x0 = 1)
        if (workPacket.debuglevel > 1)
        {
            MSerial.print("* z[");
            MSerial.print(offset);
            MSerial.print("] ");
            MSerial.print(zbuf[offset], 5);
            MSerial.print(" z[");
            MSerial.print(offset1);
            MSerial.print("] ");
            MSerial.println(zbuf[offset1], 5);
        }
        workPacket.z = zbuf[offset] + (zbuf[offset1] - zbuf[offset]) * (index - (int)index);
        if (workPacket.debuglevel > 0)
        {
            MSerial.print("* z ");
            MSerial.println(workPacket.z, 5);
        }

        return;
    }

    return;
}

static uint16_t captureSample()
{
    uint16_t value = 0;
    // This is a different reject count than the one in workPacket.reject_count.
    // This rejects completely bogus samples early on.
    uint16_t reject_count = 0;

    while (reject_count < 50)
    {
        if (i2cCapacitanceDone(value))
        {
            if (value != 0 && value != 0xffff)
            {
                return value * SENSOR_MULT;
            }
            reject_count++;
        }
        manage_heater();
        manage_inactivity();
    }
    // 50 samples have returned with the same value. Most likely, the sensor
    // is not connected. Return error and stop all motion to prevent the bed
    // from colliding with the head.
    quickStop();

    return 0;
}

static void doTheProbing(const float start_position[], const int feedrate, const float extra_z_move)
{
    int t0 = millis();

    // Loop for as long as the Z-axis is moving.
    while (blocks_queued())
    {
        int t1 = millis();
        long sample_value = captureSample();    // returns unsigned 0 on error.
        if (sample_value == 0)
        {
            workPacket.state = ERROR;
            if (workPacket.debuglevel)
            {
                MSerial.print("LOG:CAPACITIVE_SENSOR_ERROR - capacitive sensor malfunction - stopped");
            }

            return;
        }
        sample_value -= capacitive_baseline;

        float zf = float(st_get_position(Z_AXIS)) / axis_steps_per_unit[Z_AXIS];
        if (workPacket.debuglevel)
        {
            //MSerial.print(millis());
            //MSerial.print(' ');
        }
        int t2 = millis();
        if (workPacket.state != CONTINUE)
            processSample(sample_value, zf);

        int t3 = millis();
        if (workPacket.debuglevel)
        {
            MSerial.print(zf, 5);
            MSerial.print(" ");
            MSerial.print(sample_value);
            MSerial.print(" ");
            MSerial.print((long)workPacket.inflection);
            MSerial.print(" ");
            MSerial.print(workPacket.reject_count);
            MSerial.print(" ");
            MSerial.print(workPacket.state);
            MSerial.print(" ");
            MSerial.print(t3 - t0);
            MSerial.print(" ");
            MSerial.print(t2 - t1);
            MSerial.print(" ");
            MSerial.println(t3 - t2);
        }

        if (workPacket.state == DONE)
        {
            if (workPacket.debuglevel)
            {
                MSerial.print("* done\n");
            }

            // Was requested to perform extra Z-move after detecting the build plate? (debug/test feature)
            if (extra_z_move > 0)
            {
                workPacket.state = CONTINUE;
                plan_buffer_line(start_position[X_AXIS], start_position[Y_AXIS], current_position[Z_AXIS] - extra_z_move,
                        current_position[E_AXIS], float(feedrate) / 60, active_extruder);
            }
            else
            {
                return;
            }
        }

        if (workPacket.state == ERROR)
        {
            return;
        }
        t0 = millis();
    }
}

float probeWithCapacitiveSensor(const float start_position[], const int feedrate, const int verbosity, const float move_distance, const float extra_z_move)
{
    // Prepare for probing
    stop_heaters_pwm = true;
    manage_heater();
    memset(&workPacket, 0, sizeof(WorkPacket));
    workPacket.debuglevel = verbosity;
    workPacket.z = ERROR_HEIGHT;

    // move to start position
    plan_buffer_line(start_position[X_AXIS], start_position[Y_AXIS], start_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Z_AXIS], active_extruder);
    st_synchronize();

    disable_x();
    disable_y();
    disable_e0();
    disable_e1();
    disable_e2();

    // A Q&D-hack to figure out if the distorted first few samples are caused by moving mechanics.
    // 20ms should be enough as only the first +/-10 samples are distorted.
    _delay_ms(20);

    // start Z-move
    plan_buffer_line(start_position[X_AXIS], start_position[Y_AXIS], start_position[Z_AXIS] - move_distance, current_position[E_AXIS], float(feedrate)/60, active_extruder);

    // Do the actual probing.
    doTheProbing(start_position, feedrate, extra_z_move);

    // Restore printer to a defined state
    quickStop();    // Discard any possible movements still going on.

    if (workPacket.state == ERROR)
    {
        // Set magic error value. A clumsy method to report error state (old history from early printer design when there was no display yet).
        workPacket.z = ERROR_HEIGHT;
    }

    enable_x();
    enable_y();
    stop_heaters_pwm = false;

    return workPacket.z;
}

void moveWithCapacitiveSensor(const int feedrate, const float move_distance)
{
    // prepare for move
    stop_heaters_pwm = true;
    manage_heater();

    st_synchronize();

    disable_x();
    disable_y();
    disable_e0();
    disable_e1();
    disable_e2();

    // start Z-move
    plan_buffer_line(current_position[X_AXIS],
                     current_position[Y_AXIS],
                     current_position[Z_AXIS] - move_distance,
                     current_position[E_AXIS], float(feedrate)/60, active_extruder);

    int t0 = millis();
    while (blocks_queued())
    {
        int t1 = millis();
        long sample_value = captureSample();
        float zf = float(st_get_position(Z_AXIS)) / axis_steps_per_unit[Z_AXIS];

        MSerial.print(zf, 5);
        MSerial.print(" ");
        MSerial.print(sample_value);
        MSerial.print(" ");
        MSerial.println(t1 - t0);

        t0 = t1;
    }

    enable_x();
    enable_y();
    stop_heaters_pwm = false;
}

/** @brief Capture a sample from the capdac
 *  @param capdac_level The step/level to sample
 *  @return Returns the 3rd sample
 */
static int16_t getCAPDACSample(uint8_t capdac_level)
{
    i2cCapacitanceSetCAPDAC(capdac_level);

    /* throw away the first samples; the CAPDAC needs some time to settle */
    captureSample();
    captureSample();

    return (int16_t)captureSample();
}

/* The static capacitance seen by the sensor may be larger than the
   measurement range (15pF). This may easily happen when ESD protection is
   used. The measured value should not be too large, because it will
   increase when the head is close to the bed. The value may even clip at
   32767 (reading only the 16 most significant bits).

   The FDC1004 contains a so-called CAPDAC which can be enbled to compensate
   for a static capacitance. One step in the CAPDAC setting subtracts
   3.125pF from the measured value. With the reading as used here (bits
   23:8), one step decreases the value by about 6300. The CAPDAC provides 32
   steps (0 = off).

   This function finds the best CAPDAC setting by increasing it until the
   measured capacitance is less than can be offset using the CAPDAC. Make
   sure that the bed is in the lowest position that will be used for
   measuring, otherwise later measurements may give negative results (2's
   complement). The linear regression algorithm cannot deal with negative
   numbers. We don't need to worry about edge cases: in typical situations
   the noise on the sensor is about 25 (peak-peak), while the sensor value
   will increase by about 100 when the bed is raised from the calibration
   height to the starting height for Z probing.
*/
#define CAPDAC_MAX_STEPS 32

void calibrateCapacitanceOffset(int verbosity)
{
    int16_t value;
    uint8_t capdac_level;

    if (verbosity > 0)
    {
        /* debug only: cycle once through all available settings to check out the CAPDAC */
        for (capdac_level = 0; capdac_level < CAPDAC_MAX_STEPS; capdac_level++)
        {
            value = getCAPDACSample(capdac_level);
            MSerial.print("capdac ");
            MSerial.print(capdac_level, DEC);
            MSerial.print(" value ");
            MSerial.println(value, DEC);
        }
    }

    for (capdac_level = 0; capdac_level < CAPDAC_MAX_STEPS; capdac_level++)
    {
        value = getCAPDACSample(capdac_level);

        /* continue until the measured capacitance is negative, then back up */
        if (value < 0)
        {
            if (capdac_level > 0)
            {
                --capdac_level;
                value = getCAPDACSample(capdac_level);
                MSerial.print("capdac ");
                MSerial.print(capdac_level, DEC);
                MSerial.print(" value ");
                MSerial.println(value, DEC);
                return;
            }
            else
            {
                /* the measured capacitance is already negative?! */
                MSerial.println("ERROR:CAPACITIVE_SENSOR_ERROR: negative capacitance without compensation");
                return;
            }
        }
    }

    /* the loop should never complete */
    MSerial.println("ERROR:CAPACITIVE_SENSOR_ERROR: cannot compensate static capacitance (sensor shorted?)");
}

void updateCapacitiveSensorBaseline(int verbosity)
{
    capacitive_baseline = (captureSample() + captureSample() + captureSample()) / 3;

    if (verbosity > 0)
    {
        MSerial.print("capacitive_baseline ");
        MSerial.println(capacitive_baseline, DEC);
    }
}
