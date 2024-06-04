#include "SYSTICKTIMER.h"

uint32_t Systick_timer_configuration(uint32_t ticks) {
	SysTick-> LOAD = ticks - 1;
	NVIC_SetPriority(SysTick_IRQn,15);
	SysTick->VAL = 0;
	SysTick->CTRL = (1<<2)|(1<<1)|(1<<0);
	return(0);
}

