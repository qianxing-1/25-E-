#ifndef __EMM_GIMBAL_H
#define __EMM_GIMBAL_H
#include "stm32f10x.h"

/* EMM步进电机结构体 - 每个轴一个实例 */
typedef struct
{
    GPIO_TypeDef *STEP_Port;
    uint16_t STEP_Pin;

    GPIO_TypeDef *DIR_Port;
    uint16_t DIR_Pin;

    GPIO_TypeDef *ENA_Port;
    uint16_t ENA_Pin;

    float Step_Frequency;        /* 当前STEP频率(Hz) */
    uint8_t Direction;           /* 1=正转, 0=反转 */
} EMM_Motor;

/* PID控制器结构体 */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Integral;              /* 积分累加值 */
    float Last_Error;            /* 上一次误差(用于微分) */
    float Max_Output;            /* 输出上限 */
    float Min_Output;            /* 输出下限 */
} PID_Controller;

/* ---- 电机硬件接口 ---- */
void EMM_Motor_Init(EMM_Motor *motor);
void EMM_Enable(EMM_Motor *motor);
void EMM_Disable(EMM_Motor *motor);
void EMM_Set_Direction(EMM_Motor *motor, uint8_t dir);
void EMM_Set_Speed(EMM_Motor *motor, float frequency);

/* ---- PID控制器 ---- */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float max, float min);
float PID_Calculate(PID_Controller *pid, float error);

/* ---- 视觉跟踪控制 (PID + 电机方向/速度) ---- */
void EMM_Visual_Control(EMM_Motor *motor, PID_Controller *pid,
                        float image_error);
void EMM_Mode4_Control(EMM_Motor *motor, PID_Controller *pid,
                       float image_error);
void EMM_Mode4_Reset(void);
#endif
