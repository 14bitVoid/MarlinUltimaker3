#ifndef CAP_SENSE_PROBE_H
#define CAP_SENSE_PROBE_H

/**
 *  Probe the bed with the capacitive sensor. Report back the Z position where the bed is hit.
 *  Does not change the X/Y position. Sets the Z position on the reported Z position.
 */
float probeWithCapacitiveSensor(const float start_position[], const int feedrate, const int verbosity, const float move_distance, const float extra_z_move);

/**
 *  Make a move in the Z direction only while logging alle capacitive sensor data.
 *  Does not change the X/Y position. Sets the Z position on the reported Z position.
 *  @param feedrate: speed of which to move in mm/minute
 *  @param move_distance: distance moved in Z that the bed is moved upwards, in mm.
 */
void moveWithCapacitiveSensor(const int feedrate, const float move_distance);

/**
 * Compensate any static capacitance by setting the CAPDAC. Finds best value by increasing until
 * measured capacitance is closest to zero, but still positive.
 */
void calibrateCapacitanceOffset(int verbosity);

/**
 * sets the base capacitive offsets based on the average of a couple of current samples from the capacitive sensor
 */
void updateCapacitiveSensorBaseline(int verbosity);

#endif//CAP_SENSE_PROBE_H
