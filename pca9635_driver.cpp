#include "pca9635_driver.h"
#include "i2c_driver.h"
#include "Marlin.h"

#define PCA9635_ADDRESS 0b1000100

static i2cCommand update_command;
static uint8_t update_buffer[17];

void initPCA9635()
{
    i2cCommand init_command;
    uint8_t init_buffer[25];

    i2cDriverCommandSetup(update_command, PCA9635_ADDRESS << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_MEDIUM, update_buffer, sizeof(update_buffer));
    i2cDriverCommandSetup(init_command, PCA9635_ADDRESS << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_MEDIUM, init_buffer, sizeof(init_buffer));

    init_buffer[0] = 0x80; //Start writing from address 0, with auto increase.
    init_buffer[1] = 0x00; //MODE1, disable sleep, disable all other addresses.
    init_buffer[2] = _BV(2); //MODE2, OUTDRV enabled
    //All PWM outputs to 0
    for(uint8_t n=3; n<19; n++)
        init_buffer[n] = 0x00;
    init_buffer[19] = 0x00; //GRPPWM
    init_buffer[20] = 0x00; //GRPFREQ
    init_buffer[21] = 0xAA; //Drivers enabled PWM only (no GRPPWM)
    init_buffer[22] = 0xAA; //Drivers enabled PWM only (no GRPPWM)
    init_buffer[23] = 0xAA; //Drivers enabled PWM only (no GRPPWM)
    init_buffer[24] = 0xAA; //Drivers enabled PWM only (no GRPPWM)

    //Setup the update buffer.
    update_buffer[0] = 0x82; //Start at PWM0 with auto increase.
    for(uint8_t n=1; n<17; n++)
        update_buffer[n] = 0x00;

    i2cDriverExecuteAndWait(&init_command);
}

void setupPCA9635output(uint8_t channel, uint8_t value)
{
    if (channel > 15)
        return;
    update_buffer[channel + 1] = value;
}

void executePCA9635output()
{
    if (update_command.finished)
        i2cDriverPlan(&update_command);
}

void setPCA9635led(uint8_t tool, uint8_t red, uint8_t green, uint8_t blue)
{
    if (tool == 0)
    {
        setupPCA9635output(10, blue);
        setupPCA9635output(11, green);
        setupPCA9635output(12, red);
    }
    else
    {
        setupPCA9635output(13, red);
        setupPCA9635output(14, green);
        setupPCA9635output(15, blue);
    }
    executePCA9635output();
}
