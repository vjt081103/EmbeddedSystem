#include "LED.h"

void Init_Green_Led() {
	SIM->SCGC5 |= 1u<<12; //cap clock cho portD
	PORTD->PCR[5] |= 1u<<8; //Mux cua pin 5 portD la 001 hay GPIO
	PTD->PDDR |= 1u<<5; //Cho chan 5 la dau ra
	PTD->PDOR &= ~(1u<<5); 
	//PTD->PTOR |= 1u<<5; //Dat gia tri khoi tao la bat den
};


void Init_Red_Led() {
	SIM->SCGC5 |= 1u<<13;//cap clock cho portE
	PORTE->PCR[29] |= 1u<<8; //Mux cua pin 29 PortE la 001 hay GPIO
	PTE->PDDR |= 1u<<29; //cho chan 29 la dau ra
	PTE->PDOR &= ~(1u<<29); 
	//PTE->PTOR |= 1u<<29; // Dat gia tri khoi tao la bat den
}

void BlinkLedGreen() {
	uint32_t i;
	PTD->PSOR = 1u<<5;
	for(i = 0; i< 5000000; i++){};
	PTD->PCOR = 1u<<5;
	for(i =0;i<5000000;i++){};
}

void BlinkLedRed() {
	uint32_t i;
	PTE->PSOR = 1u<<29;
	for(i = 0; i< 5000000; i++){};
	PTE->PCOR = 1u<<29;
	for(i =0;i<5000000;i++){};
}
