#include "SWITCH.h"
#include "LED.h"

void Init_SW1() {
	SIM->SCGC5 |= 1u<<11; //cap clock cho port C
	PORTC->PCR[3] |= 1u<<8; // SW1 la GPIO;
	PORTC->PCR[3] |= 1u<<1; // enable pull cho sw1
	PORTC->PCR[3] |= 1u; // cho sw1 o trang thai pull up
	PTC->PDDR &= ~(1u<<3); //cho sw1 la input
	
	
	PORTC->PCR[3] |= PORT_PCR_IRQC(0xA); //cho ngat khi co falling edge
	NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);// xoa cac ngat dang cho xu ly tren Port D
  NVIC_EnableIRQ(PORTC_PORTD_IRQn); // kich hoat nguon ngat cho port D
	
}

void Init_SW2() {
	SIM->SCGC5 |= 1u<<11;
	PORTC->PCR[12] |= 1u<<8;
	PORTC->PCR[12] |= 1u<<1;
	PORTC->PCR[12] |= 1u;
	PTC->PDDR &= ~(1u<<12);
	
	PORTC->PCR[12] |= PORT_PCR_IRQC(0xA);
	NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
	NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

void PORTC_PORTD_IRQHandler() {
	if(PORTC->ISFR & (1 << 3)) { //Kiem tra ngat tu SW2 (PTC12)
		PTD->PTOR |= (1 << 5);
    PORTC->ISFR = (1 << 3);
	}
	
	
	if (PORTC->ISFR & (1 << 12)) { // Kiem tra ngat tu SW2 (PTC12)
		PTE->PTOR |= (1 << 29);
		PORTC->ISFR = (1 << 12);
  }
}
