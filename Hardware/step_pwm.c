#include "step_pwm.h"

/* TIM1_CH4 on PA11 outputs STEP pulses for the yaw motor. */
void STEP_PWM_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1,
                           ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* 72 MHz / 72 = 1 MHz timer clock. */
    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler = 71;
    tim.TIM_Period = 999;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tim);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 500;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM1, &oc);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Disable);
    TIM_Cmd(TIM1, DISABLE);
}

void STEP_PWM_SetFreq(uint32_t freq)
{
    static uint32_t current_freq = 0;
    uint32_t arr;

    if (freq == 0)
    {
        TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Disable);
        TIM_Cmd(TIM1, DISABLE);
        TIM_SetCounter(TIM1, 0);
        current_freq = 0;
        return;
    }

    if (freq > 20000)
        freq = 20000;
    if (freq < 16)
        freq = 16;

    if (freq == current_freq)
        return;

    arr = 1000000 / freq - 1;
    TIM_SetAutoreload(TIM1, arr);
    TIM_SetCompare4(TIM1, (arr + 1) / 2);

    if (current_freq == 0)
    {
        TIM_SetCounter(TIM1, 0);
        TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
        TIM_ClearFlag(TIM1, TIM_FLAG_Update);
        TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Enable);
        TIM_Cmd(TIM1, ENABLE);
    }

    /* While running, preload the new period without resetting the pulse. */
    current_freq = freq;
}
