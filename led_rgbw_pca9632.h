#ifndef LED_RGBW_PCA9632_H
#define LED_RGBW_PCA9632_H

#include <stdint.h>

void ledRGBWInit();
void ledRGBWUpdate(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

/** @brief Sets the hotend color
 *  @param hotend_index The hotend index (>= 0 and < EXTRUDERS)
 *  @param r The red factor
 *  @param g The green factor
 *  @param b The blue factor
 */
void setHotendLedColor(uint8_t hotend_index, uint8_t r, uint8_t g, uint8_t b);

#endif//LED_RGBW_PCA9632_H
