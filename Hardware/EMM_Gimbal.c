#include "step_pwm.h"
#include "EMM_Gimbal.h"
#include "Delay.h"

void EMM_Motor_Init(EMM_Motor *motor)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = motor->DIR_Pin;
    GPIO_Init(motor->DIR_Port, &gpio);

    gpio.GPIO_Pin = motor->ENA_Pin;
    GPIO_Init(motor->ENA_Port, &gpio);

    motor->Direction = 0;
    motor->Step_Frequency = 0;
    EMM_Set_Direction(motor, 0);
    EMM_Disable(motor);
}

void EMM_Enable(EMM_Motor *motor)
{
    GPIO_ResetBits(motor->ENA_Port, motor->ENA_Pin);
}

void EMM_Disable(EMM_Motor *motor)
{
    GPIO_SetBits(motor->ENA_Port, motor->ENA_Pin);
}

/* Existing hardware convention: 1 turns left and 0 turns right. */
void EMM_Set_Direction(EMM_Motor *motor, uint8_t dir)
{
    motor->Direction = dir;
    if (dir)
        GPIO_ResetBits(motor->DIR_Port, motor->DIR_Pin);
    else
        GPIO_SetBits(motor->DIR_Port, motor->DIR_Pin);
}

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float max, float min)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Integral = 0;
    pid->Last_Error = 0;
    pid->Max_Output = max;
    pid->Min_Output = min;
}

float PID_Calculate(PID_Controller *pid, float error)
{
    float output;

    pid->Integral += error;
    if (pid->Integral > 10000)
        pid->Integral = 10000;
    if (pid->Integral < -10000)
        pid->Integral = -10000;

    output = pid->Kp * error
           + pid->Ki * pid->Integral
           + pid->Kd * (error - pid->Last_Error);
    pid->Last_Error = error;

    if (output > pid->Max_Output)
        output = pid->Max_Output;
    if (output < pid->Min_Output)
        output = pid->Min_Output;
    return output;
}

void EMM_Visual_Control(EMM_Motor *motor, PID_Controller *pid,
                        float image_error)
{
    float speed;
    float abs_error;
    float min_speed;
    float max_speed;
    uint8_t desired_direction;

    speed = PID_Calculate(pid, image_error);
    abs_error = (image_error >= 0.0f) ? image_error : -image_error;

    /* Slow down near center so successive vision frames cannot skip it. */
    if (abs_error > 160.0f)
    {
        min_speed = 300.0f;
        max_speed = 600.0f;
    }
    else if (abs_error > 80.0f)
    {
        min_speed = 200.0f;
        max_speed = 400.0f;
    }
    else if (abs_error > 30.0f)
    {
        min_speed = 120.0f;
        max_speed = 240.0f;
    }
    else
    {
        min_speed = 60.0f;
        max_speed = 120.0f;
    }

    if (speed > 0.0f)
    {
        if (speed < min_speed)
            speed = min_speed;
        if (speed > max_speed)
            speed = max_speed;
    }
    else
    {
        if (speed > -min_speed)
            speed = -min_speed;
        if (speed < -max_speed)
            speed = -max_speed;
    }

    desired_direction = (speed > 0.0f) ? 0 : 1;

    /* Stop STEP, satisfy DIR setup time, then resume at the new speed. */
    if (desired_direction != motor->Direction)
    {
        STEP_PWM_SetFreq(0);
        motor->Step_Frequency = 0;
        EMM_Set_Direction(motor, desired_direction);
        Delay_us(5);
    }

    EMM_Set_Speed(motor, (speed > 0.0f) ? speed : -speed);
}

void EMM_Set_Speed(EMM_Motor *motor, float frequency)
{
    EMM_Enable(motor);
    motor->Step_Frequency = frequency;
    STEP_PWM_SetFreq((uint32_t)frequency);
}
