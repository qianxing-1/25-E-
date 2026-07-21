#include "step_pwm.h"
#include "EMM_Gimbal.h"
#include "Delay.h"

#define TRACK_MIN_SPEED       16.0f
#define TRACK_MAX_SPEED       600.0f
#define TRACK_SPEED_RAMP      80.0f
#define MODE4_SPEED_RAMP      40.0f

static void EMM_Apply_Speed_Command(EMM_Motor *motor, float target_speed,
                                    float speed_ramp)
{
    float speed;
    float current_speed;
    uint8_t desired_direction;

    speed = (target_speed >= 0.0f) ? target_speed : -target_speed;
    if (speed < 1.0f)
    {
        STEP_PWM_SetFreq(0);
        motor->Step_Frequency = 0;
        EMM_Disable(motor);
        return;
    }

    if (speed < TRACK_MIN_SPEED)
        speed = TRACK_MIN_SPEED;
    if (speed > TRACK_MAX_SPEED)
        speed = TRACK_MAX_SPEED;

    desired_direction = (target_speed >= 0.0f) ? 0 : 1;

    /* Stop STEP, satisfy DIR setup time, then resume at the new speed. */
    if (desired_direction != motor->Direction)
    {
        STEP_PWM_SetFreq(0);
        motor->Step_Frequency = 0;
        EMM_Set_Direction(motor, desired_direction);
        Delay_us(5);
    }

    current_speed = motor->Step_Frequency;
    EMM_Enable(motor);

    if (speed > current_speed + speed_ramp)
        current_speed += speed_ramp;
    else if (speed + speed_ramp < current_speed)
        current_speed -= speed_ramp;
    else
        current_speed = speed;

    if (current_speed < TRACK_MIN_SPEED)
        current_speed = TRACK_MIN_SPEED;
    if (current_speed > TRACK_MAX_SPEED)
        current_speed = TRACK_MAX_SPEED;

    motor->Step_Frequency = current_speed;
    STEP_PWM_SetFreq((uint32_t)current_speed);
}

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
    float target_speed;

    target_speed = PID_Calculate(pid, image_error);
    EMM_Apply_Speed_Command(motor, target_speed, TRACK_SPEED_RAMP);
}

void EMM_Visual_Control_FF(EMM_Motor *motor, PID_Controller *pid,
                           float image_error, float speed_feedforward)
{
    float target_speed;

    target_speed = PID_Calculate(pid, image_error) + speed_feedforward;
    if (target_speed > pid->Max_Output)
        target_speed = pid->Max_Output;
    if (target_speed < pid->Min_Output)
        target_speed = pid->Min_Output;

    EMM_Apply_Speed_Command(motor, target_speed, MODE4_SPEED_RAMP);
}

void EMM_Set_Speed(EMM_Motor *motor, float frequency)
{
    EMM_Apply_Speed_Command(motor, frequency, TRACK_SPEED_RAMP);
}
