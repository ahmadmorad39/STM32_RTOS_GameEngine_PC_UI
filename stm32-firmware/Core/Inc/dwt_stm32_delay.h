/*
 * dwt_stm32_delay.h
 *
 *  Created on: Apr 7, 2025
 *      Author: ahmad
 */

#ifndef INC_DWT_STM32_DELAY_H_
#define INC_DWT_STM32_DELAY_H_

#include "stm32f4xx_hal.h"


void DWT_Delay_Init(void);
void DWT_Delay_us(uint32_t us);

#endif /* INC_DWT_STM32_DELAY_H_ */
