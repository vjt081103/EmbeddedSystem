#include "MKL46Z4.h"
#include "LED.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"


#define MAG_WHO_AM_I_REG 0x07
#define MAG_WHO_AM_I_DATA 0XC4
#define MAG_ADDRESS 0X0E

#define MAG_I2C_CLK_SRC I2C0_CLK_SRC
#define MAG_I2C_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)

#define I2C_RELEASE_SDA_PORT PORTE
#define I2C_RELEASE_SCL_PORT PORTE
#define I2C_RELEASE_SDA_GPIO GPIOE
#define I2C_RELEASE_SDA_PIN 25U
#define I2C_RELEASE_SCL_GPIO GPIOE
#define I2C_RELEASE_SCL_PIN 24U


#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_BAUDRATE 100000U
#define MAG_STATUS_REG 0x00U
#define MAG_CTRL_REG1 0x10U
#define MAG_READ_TIMES 10U


#define X_MSB_REG 0x01
#define X_LSB_REG	0x02
#define Y_MSB_REG 0x03
#define Y_LSB_REG 0x04
#define Z_MSB_REG 0x05
#define Z_LSB_REG 0x06



