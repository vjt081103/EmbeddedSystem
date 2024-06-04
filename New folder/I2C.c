#include "MKL46Z4.h"

#define BUS_COUNT_RELEASE 100u
#define I2C_BAUDRATE 100000u

static void I2C_Release_Bus_Delay(void) {
	uint32_t i = 0;
    for (i = 0; i < BUS_COUNT_RELEASE; i++)
    {
        __NOP();
    }
}

void I2C0_Init() {
	SIM -> SCGC4 |= SIM_SCGC4_I2C0_MASK;
	
	SIM -> SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[24] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTE->PCR[25] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	
	PTE->PDDR |= 1 << 24;
	PTE->PDDR |= 1 << 25;

	PTE->PDOR &= ~(1 << 24);
	PTE->PDOR &= ~(1 << 25);
	
	PTE->PDOR |= 1 << 25;
	
	int i = 0;
	for (i = 0; i < 9; i++) {
		PTE->PDOR |= 1 << 24;
		I2C_Release_Bus_Delay();
		
		PTE->PDOR &= ~(1<<25);
		I2C_Release_Bus_Delay();
		PTE->PDOR &= ~(1<<24);
		I2C_Release_Bus_Delay();
		I2C_Release_Bus_Delay();
	}
	PTE -> PDOR |= 1 << 24;
	I2C_Release_Bus_Delay();
	
	PTE -> PDOR |= 1 << 25;
	I2C_Release_Bus_Delay();
	
	PTE -> PDOR &= ~(1<<24);
	I2C_Release_Bus_Delay();
	
	PTE ->PDOR &= ~(1<<25);
	I2C_Release_Bus_Delay();
	
	PTE->PDDR &= ~(1<<24);
	PTE->PDDR &= ~(1<<25);
	
	PORTE->PCR[24] = 0;
	PORTE->PCR[25] = 0;
	
	PORTE-> PCR[24] |= PORT_PCR_MUX(5) | PORT_PCR_PE_MASK |PORT_PCR_PS_MASK|PORT_PCR_SRE_MASK;
	PORTE->PCR[25] |= PORT_PCR_MUX(5) | PORT_PCR_PE_MASK |PORT_PCR_PS_MASK|PORT_PCR_SRE_MASK;
	
	I2C0->F = I2C_F_ICR(0x14) | I2C_F_MULT(0);
	I2C0->C1 = I2C_C1_IICEN_MASK |I2C_C1_DMAEN_MASK;
	
}

void I2C1_Init() {
	SIM -> SCGC4 |= SIM_SCGC4_I2C1_MASK;
	
	SIM -> SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[0] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTE->PCR[1] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	
	PTE->PDDR |= 1 << 0;
	PTE->PDDR |= 1 << 1;

	PTE->PDOR &= ~(1 << 0);
	PTE->PDOR &= ~(1 << 1);
	
	PTE->PDOR |= 1 << 0;
	
	int i = 0;
	for (i = 0; i < 9; i++) {
		PTE->PDOR |= 1 << 1;
		I2C_Release_Bus_Delay();
		
		PTE->PDOR &= ~(1<<0);
		I2C_Release_Bus_Delay();
		PTE->PDOR &= ~(1<<1);
		I2C_Release_Bus_Delay();
		I2C_Release_Bus_Delay();
	}
	PTE -> PDOR |= 1 << 1;
	I2C_Release_Bus_Delay();
	
	PTE -> PDOR |= 1 << 0;
	I2C_Release_Bus_Delay();
	
	PTE -> PDOR &= ~(1<<1);
	I2C_Release_Bus_Delay();
	
	PTE ->PDOR &= ~(1<<0);
	I2C_Release_Bus_Delay();
	
	PTE->PDDR &= ~(1<<0);
	PTE->PDDR &= ~(1<<0);
	
	PORTE->PCR[0] &= 0;
	PORTE->PCR[1] &= 0;
	
	PORTE-> PCR[0] |= PORT_PCR_MUX(6) | PORT_PCR_PE_MASK |PORT_PCR_PS_MASK|PORT_PCR_SRE_MASK;
	PORTE->PCR[1] |= PORT_PCR_MUX(6) | PORT_PCR_PE_MASK |PORT_PCR_PS_MASK|PORT_PCR_SRE_MASK;
	
	I2C1->F = I2C_F_ICR(0x14) | I2C_F_MULT(0);
	I2C1->C1 = I2C_C1_IICEN_MASK;
	
}



