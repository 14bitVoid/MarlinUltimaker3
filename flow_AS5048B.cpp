#include "Marlin.h"

#include "flow_AS5048B.h"
#include "i2c_driver.h"

#define AS5048B_PROG            (0x03)
#define AS5048B_PROG_EN _BV     (0)
#define AS5048B_PROG_BURN _BV   (3)
#define AS5048B_PROG_VERIFY _BV (6)

#define AS5048B_ZERO_HI         (0x16) //bits 0..7
#define AS5048B_ZERO_LO         (0x17) //bits 0..5
#define AS5048B_AGC             (0xfa)
#define AS5048B_DIAG            (0xfb)
#define AS5048B_MAGN_HI         (0xfc) //bits 0..7
#define AS5048B_MAGN_HI_MASK    (0xff)
#define AS5048B_MAGN_LO         (0xfd) //bits 0..5
#define AS5048B_MAGN_LO_MASK    (0x3f)
#define AS5048B_ANGLE_HI        (0xfe) //bits 0..7
#define AS5048B_ANGLE_HI_MASK   (0xff)
#define AS5048B_ANGLE_LO        (0xff) //bits 0..5
#define AS5048B_ANGLE_LO_MASK   (0x3f)

#define AS5048B_RES             (16384) /* 14 bits */

#define AS5048B_ADDRESS_0       (0x40) /* 0b1000000 */
#define AS5048B_ADDRESS_1       (0x41) /* 0b1000001 */

struct flowAS5048B_sensor_data
{
    uint8_t sensor_address;
    i2cCommand i2c_flow_write_hi_command;
    i2cCommand i2c_flow_write_lo_command;
    i2cCommand i2c_flow_read_hi_command;
    i2cCommand i2c_flow_read_lo_command;
    uint8_t i2c_flow_write_hi_buffer[1];
    uint8_t i2c_flow_write_lo_buffer[1];
    uint8_t i2c_flow_read_hi_buffer[1];
    uint8_t i2c_flow_read_lo_buffer[1];
};

static struct flowAS5048B_sensor_data sd[NR_OF_FLOW_SENSORS];

void flowAS5048BInit(uint8_t n)
{
    i2cCommand i2c_flow_init_command;
    uint8_t i2c_flow_init_buffer[] = {AS5048B_ZERO_HI, 0x00, AS5048B_ZERO_LO, 0x00};

    switch(n)
    {
        case 1:
            sd[n].sensor_address = AS5048B_ADDRESS_1;
            break;
        case 0:
        default:
            sd[n].sensor_address = AS5048B_ADDRESS_0;
            break;
    }
//    SERIAL_ECHOPGM("flowAS5048BInit: sd[");
//    MSerial.print(n, HEX);
//    SERIAL_ECHOPGM("].sensor_addres=");
//    MSerial.println(sd[n].sensor_address, HEX);

    i2cDriverCommandSetup(sd[n].i2c_flow_write_hi_command, sd[n].sensor_address << 1  | I2C_WRITE_BIT, I2C_QUEUE_PRIO_MEDIUM, sd[n].i2c_flow_write_hi_buffer, sizeof(sd[n].i2c_flow_write_hi_buffer));
    i2cDriverCommandSetup(sd[n].i2c_flow_write_lo_command, sd[n].sensor_address << 1  | I2C_WRITE_BIT, I2C_QUEUE_PRIO_MEDIUM, sd[n].i2c_flow_write_lo_buffer, sizeof(sd[n].i2c_flow_write_lo_buffer));
    i2cDriverCommandSetup(sd[n].i2c_flow_read_hi_command, sd[n].sensor_address << 1 | I2C_READ_BIT, I2C_QUEUE_PRIO_MEDIUM, sd[n].i2c_flow_read_hi_buffer, sizeof(sd[n].i2c_flow_read_hi_buffer));
    i2cDriverCommandSetup(sd[n].i2c_flow_read_lo_command, sd[n].sensor_address << 1 | I2C_READ_BIT, I2C_QUEUE_PRIO_MEDIUM, sd[n].i2c_flow_read_lo_buffer, sizeof(sd[n].i2c_flow_read_lo_buffer));

    i2cDriverCommandSetup(i2c_flow_init_command, sd[n].sensor_address << 1 | I2C_WRITE_BIT, I2C_QUEUE_PRIO_LOW, i2c_flow_init_buffer, sizeof(i2c_flow_init_buffer));
    i2cDriverExecuteAndWait(&i2c_flow_init_command);

    sd[n].i2c_flow_write_hi_buffer[0] = AS5048B_ANGLE_HI;
    sd[n].i2c_flow_write_lo_buffer[0] = AS5048B_ANGLE_LO;
}

void flowAS5048BStart(uint8_t n)
{
    if (!sd[n].i2c_flow_write_hi_command.finished || !sd[n].i2c_flow_read_hi_command.finished || !sd[n].i2c_flow_write_lo_command.finished || !sd[n].i2c_flow_read_lo_command.finished)
        return;

//    SERIAL_ECHOPGM("flowAS5048Start, sensor=");
//    MSerial.println(n, HEX);

    i2cDriverPlan(&(sd[n].i2c_flow_write_hi_command));
    i2cDriverPlan(&(sd[n].i2c_flow_read_hi_command));
    i2cDriverPlan(&(sd[n].i2c_flow_write_lo_command));
    i2cDriverPlan(&(sd[n].i2c_flow_read_lo_command));
}

bool flowAS5048BDone(uint8_t n, uint16_t &value)
{
    if (!sd[n].i2c_flow_write_hi_command.finished || !sd[n].i2c_flow_read_hi_command.finished || !sd[n].i2c_flow_write_lo_command.finished || !sd[n].i2c_flow_read_lo_command.finished)
        return false;

    value = uint16_t(sd[n].i2c_flow_read_hi_buffer[0] << 6) + uint16_t(sd[n].i2c_flow_read_lo_buffer[0] & AS5048B_ANGLE_LO_MASK);

//    SERIAL_ECHOPGM("flowAS5048BDone, sensor=");
//    MSerial.print(n, HEX);
//    SERIAL_ECHOPGM(", value=");
//    MSerial.println(value, HEX);

    return true;
}

