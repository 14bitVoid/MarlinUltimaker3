#ifndef PCA9635_DRIVER_H
#define PCA9635_DRIVER_H

#include <stdint.h>

void initPCA9635();

/** @brief sets up the command to set the value to the specified channel
 *  @param channel The channel to write to
 *  @param value The value to write
 */
void setupPCA9635output(uint8_t channel, uint8_t value);

/** @brief Executes the command to write the values previously setup.
*/
void executePCA9635output();

/** @brief sets head led x to a specific rgb color
 *  @param tool the toolhead
 *  @param red the red value of the rgb set
 *  @param green the green value of the rgb set
 *  @param blue the blue value of the rgb set
 */
void setPCA9635led(uint8_t tool, uint8_t red, uint8_t green, uint8_t blue);

#endif//PCA9635_DRIVER_H
