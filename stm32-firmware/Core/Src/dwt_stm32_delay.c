/*
 * dwt_stm32_delay.c
 *
 *  Created on: Apr 7, 2025
 *      Author: ahmad
 */


#include "dwt_stm32_delay.h"

void DWT_Delay_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_Delay_us(uint32_t us) {
	// Each iteration of this loop takes approximately 4 cycles
	// So, for 1us delay: 16 cycles = 1us at 16MHz → 16 / 4 = 4 iterations
	for(uint32_t i = 0; i < (us * 4); i++) {
		__NOP();
	}
}
