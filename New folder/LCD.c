#include "LCD.h"

#define UsedPin 30

#define BackPlanes 8

#define Shift(x) (1<<(x));
void SLCD_DisplayNumber1(uint8_t digit, uint8_t position);

const uint8_t digit_to_segment[] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};


void Pin_En(void);

void Init_LCD() {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK |SIM_SCGC5_PORTD_MASK |SIM_SCGC5_PORTE_MASK;
	SIM->SCGC5 |= SIM_SCGC5_SLCD_MASK;
	
	LCD->GCR |= LCD_GCR_VSUPPLY_MASK |LCD_GCR_DUTY_MASK |LCD_GCR_LCLK(5);
	
	
	Pin_En();
	
	 SLCD_DisplayNumber1(0, 0);
    SLCD_DisplayNumber1(0, 1);
    SLCD_DisplayNumber1(0, 2);
    SLCD_DisplayNumber1(0, 3);
	LCD->GCR |= LCD_GCR_LCDEN_MASK;
	

}


void Pin_En() {
	
	LCD->PEN[0] = 0x00000000;
	LCD->BPEN[0] = 0x00000000;
	LCD->PEN[1] = 0x00000000;
	LCD->BPEN[1] = 0x00000000;
	
	for(int i = 0; i< UsedPin - BackPlanes; i++) {
		LCD->PEN[i/32] |= Shift(i%32);
	}
	
	for(int i =0; i< BackPlanes; i++) {
		LCD->BPEN[i/32] |= Shift(i%32);
		LCD->WF8B[i] = Shift(i);
	}
	
}

void SLCD_DisplayNumber(uint8_t number) {
    
}

void SLCD_DisplayNumber1(uint8_t digit, uint8_t position) {
	
	if (digit > 9) return;

    // Mã hóa ch? s? thành giá tr? tuong ?ng trên LCD
    uint8_t segment_value = digit_to_segment[digit];

    // Gán giá tr? mã hóa vào thanh ghi wavefront tuong ?ng v?i v? trí
    LCD->WF8B[position] = segment_value;
}
