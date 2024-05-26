#include "MKL46Z4.h"
#include "fsl_slcd.h"
#include "fsl_gpio.h"

void Init_LCD(void);
void SLCD_DisplayNumber(uint8_t number);
void SLCD_DisplayNumber1(uint8_t digit, uint8_t position);