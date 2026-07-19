#ifndef __STEP_PWM_H
#define __STEP_PWM_H

#include "stm32f10x.h"

/* Yaw STEP output: TIM1_CH4 on PA11. */
void STEP_PWM_Init(void);
void STEP_PWM_SetFreq(uint32_t freq);

#endif
