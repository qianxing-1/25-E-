#include "stm32f10x.h"
#include "Delay.h"
#include "Timer.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "step_pwm.h"
#include "EMM_Gimbal.h"

/* PC14 selects mode; PC15 starts the selected mode. */
uint8_t KeyNum = 0;
volatile uint16_t x = 0;
volatile uint16_t y = 0;
volatile int16_t Gimbal_Target_Offset_X = 0;
volatile uint32_t SystemTickMs = 0;

#define SCREEN_CENTER_X          320
#define CENTER_ENTER_ERROR       5
#define CENTER_EXIT_ERROR        8
#define CENTER_FINAL_ERROR       4
#define ERROR_DEADBAND           CENTER_ENTER_ERROR
#define CENTER_SETTLE_MS         200
#define MODE4_LASER_CONFIRM_MS   80
#define TARGET_STALE_MS          150

/* 1.8 degree motor, 8 microsteps: 400 pulses make 90 degrees. */
#define MODE1_STEP_FREQ          200
#define MODE1_DURATION_MS        2000

#define SEARCH_FAST_SPEED        200
#define SEARCH_TIMEOUT_MS        4000
#define AIM_TIMEOUT_MS           3000

static uint8_t LaserOn = 0;
static uint8_t CurrentMode = 1;
static uint8_t ModeActive = 0;
static volatile uint8_t TrackingEnabled = 0;
static uint8_t Mode4LaserChecking = 0;
static uint32_t Mode4LaserStartMs = 0;
static uint32_t Mode4LastFrameId = 0;
static volatile uint32_t Mode4ControlLastFrameId = 0;

EMM_Motor Yaw_Motor;
PID_Controller Yaw_PID;

static void Laser_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

static void Laser_On(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_0);
    LaserOn = 1;
}

static void StopMotor(void)
{
    STEP_PWM_SetFreq(0);
    Yaw_Motor.Step_Frequency = 0;
    EMM_Disable(&Yaw_Motor);
}

static void ResetAimPID(void)
{
    Yaw_PID.Integral = 0.0f;
    Yaw_PID.Last_Error = 0.0f;
}

static uint8_t TargetIsFresh(uint16_t target_x, uint32_t last_rx_ms)
{
    return target_x != 0 &&
           (uint32_t)(SystemTickMs - last_rx_ms) <= TARGET_STALE_MS;
}

static void ShowMode(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "Mode:");
    OLED_ShowNum(1, 6, CurrentMode, 1);
    OLED_ShowString(2, 1, "PC14=Sel PC15=Run");
}

static void ShowRunning(void)
{
    OLED_ShowString(4, 1, "Running...");
}

static void StartFastSearch(uint8_t direction)
{
    StopMotor();
    EMM_Enable(&Yaw_Motor);
    EMM_Set_Direction(&Yaw_Motor, direction);
    STEP_PWM_SetFreq(SEARCH_FAST_SPEED);
    Yaw_Motor.Step_Frequency = SEARCH_FAST_SPEED;
}

static void Mode1_TurnLeft90(void)
{
    ModeActive = 1;
    TrackingEnabled = 0;
    ShowRunning();

    StopMotor();
    EMM_Enable(&Yaw_Motor);
    EMM_Set_Direction(&Yaw_Motor, 1);
    STEP_PWM_SetFreq(MODE1_STEP_FREQ);
    Delay_ms(MODE1_DURATION_MS);
    StopMotor();

    Laser_On();
    ModeActive = 0;
    ShowMode();
}

/*
 * Search quickly without a target. With a fresh target, PID may change both
 * direction and speed until several consecutive frames are centered.
 */
static uint8_t SearchAndAim(uint8_t initial_direction)
{
    uint32_t search_start_ms = SystemTickMs;
    uint32_t aim_start_ms = 0;
    uint32_t last_frame_id = 0;
    uint32_t settle_start_ms = 0;
    uint8_t settling = 0;
    uint8_t searching = 1;
    uint8_t target_acquired = 0;

    ResetAimPID();
    StartFastSearch(initial_direction);

    while (1)
    {
        uint16_t target_x;
        uint16_t target_y;
        uint32_t last_rx_ms;
        uint32_t frame_id;

        Serial_ReadTarget(&target_x, &target_y, &last_rx_ms, &frame_id);
        (void)target_y;

        if (!target_acquired)
        {
            if ((uint32_t)(SystemTickMs - search_start_ms) >= SEARCH_TIMEOUT_MS)
                break;
        }
        else if ((uint32_t)(SystemTickMs - aim_start_ms) >= AIM_TIMEOUT_MS)
        {
            break;
        }

        if (!TargetIsFresh(target_x, last_rx_ms))
        {
            settling = 0;
            if (target_acquired)
            {
                target_acquired = 0;
                search_start_ms = SystemTickMs;
            }
            if (!searching)
            {
                ResetAimPID();
                StartFastSearch(initial_direction);
                searching = 1;
            }
        }
        else if (frame_id != last_frame_id)
        {
            int16_t error = (int16_t)target_x - SCREEN_CENTER_X;
            last_frame_id = frame_id;
            searching = 0;

            if (!target_acquired)
            {
                target_acquired = 1;
                aim_start_ms = SystemTickMs;
            }

            if (!settling &&
                error >= -CENTER_ENTER_ERROR && error <= CENTER_ENTER_ERROR)
            {
                StopMotor();
                settle_start_ms = SystemTickMs;
                settling = 1;
            }
            else if (settling)
            {
                if (error < -CENTER_EXIT_ERROR || error > CENTER_EXIT_ERROR)
                {
                    settling = 0;
                    EMM_Visual_Control(&Yaw_Motor, &Yaw_PID, (float)error);
                }
                else if ((uint32_t)(SystemTickMs - settle_start_ms) >=
                         CENTER_SETTLE_MS)
                {
                    if (error >= -CENTER_FINAL_ERROR &&
                        error <= CENTER_FINAL_ERROR)
                        return 1;

                    settling = 0;
                    EMM_Visual_Control(&Yaw_Motor, &Yaw_PID, (float)error);
                }
            }
            else
            {
                EMM_Visual_Control(&Yaw_Motor, &Yaw_PID, (float)error);
            }
        }

        Delay_ms(2);
    }

    StopMotor();
    return 0;
}

static void Mode2_SearchLeft(void)
{
    ModeActive = 1;
    TrackingEnabled = 0;
    ShowRunning();

    if (SearchAndAim(1))
        Laser_On();

    StopMotor();
    ModeActive = 0;
    ShowMode();
}

static void Mode3_SearchRight(void)
{
    ModeActive = 1;
    TrackingEnabled = 0;
    ShowRunning();

    if (SearchAndAim(0))
        Laser_On();

    StopMotor();
    ModeActive = 0;
    ShowMode();
}

static void Mode4_StartTracking(void)
{
    TrackingEnabled = 1;
    Mode4LaserChecking = 0;
    Mode4LaserStartMs = 0;
    Mode4LastFrameId = 0;
    Mode4ControlLastFrameId = 0;
    ResetAimPID();
    EMM_Mode4_Reset();
    OLED_Clear();
    OLED_ShowString(1, 1, "Mode:4 Align");
    OLED_ShowString(2, 1, "X:         ");
    OLED_ShowString(3, 1, "Y:         ");
    OLED_ShowString(4, 1, "Laser:OFF  ");
}

void Yaw_Init(void)
{
    Yaw_Motor.STEP_Port = GPIOA;
    Yaw_Motor.STEP_Pin = GPIO_Pin_11;
    Yaw_Motor.DIR_Port = GPIOA;
    Yaw_Motor.DIR_Pin = GPIO_Pin_10;
    Yaw_Motor.ENA_Port = GPIOA;
    Yaw_Motor.ENA_Pin = GPIO_Pin_12;
    EMM_Motor_Init(&Yaw_Motor);

    PID_Init(&Yaw_PID, 5.0f, 0.0f, 0.0f, 600.0f, -600.0f);
}

int main(void)
{
    Serial_Init();
    Key_Init();
    OLED_Init();
    STEP_PWM_Init();
    Yaw_Init();
    Laser_Init();
    Timer_Init();
    ShowMode();

    while (1)
    {
        uint16_t target_x;
        uint16_t target_y;
        uint32_t last_rx_ms;
        uint32_t frame_id;

        KeyNum = Key_GetNum();

        if (KeyNum == 1 && !ModeActive)
        {
            TrackingEnabled = 0;
            StopMotor();
            CurrentMode++;
            if (CurrentMode > 4)
                CurrentMode = 1;

            if (CurrentMode == 4)
                Mode4_StartTracking();
            else
                ShowMode();
        }

        if (KeyNum == 2 && !ModeActive)
        {
            switch (CurrentMode)
            {
                case 1: Mode1_TurnLeft90(); break;
                case 2: Mode2_SearchLeft(); break;
                case 3: Mode3_SearchRight(); break;
                case 4: Mode4_StartTracking(); break;
                default: break;
            }
        }

        Serial_ReadTarget(&target_x, &target_y, &last_rx_ms, &frame_id);
        if (CurrentMode == 4)
        {
            static uint16_t last_x = 0xFFFF;
            static uint16_t last_y = 0xFFFF;

            if (target_x != last_x || target_y != last_y)
            {
                last_x = target_x;
                last_y = target_y;
                OLED_ShowString(2, 3, "   ");
                OLED_ShowNum(2, 3, target_x, 3);
                OLED_ShowString(3, 3, "   ");
                OLED_ShowNum(3, 3, target_y, 3);
            }

            if (!LaserOn && frame_id != Mode4LastFrameId)
            {
                Mode4LastFrameId = frame_id;

                if (TargetIsFresh(target_x, last_rx_ms))
                {
                    int16_t error = (int16_t)target_x - SCREEN_CENTER_X;

                    if (error >= -CENTER_FINAL_ERROR &&
                        error <= CENTER_FINAL_ERROR)
                    {
                        if (!Mode4LaserChecking)
                        {
                            Mode4LaserStartMs = SystemTickMs;
                            Mode4LaserChecking = 1;
                        }
                        else if ((uint32_t)(SystemTickMs - Mode4LaserStartMs) >=
                                 MODE4_LASER_CONFIRM_MS)
                        {
                            Laser_On();
                            OLED_ShowString(4, 7, "ON ");
                            Mode4LaserChecking = 0;
                        }
                    }
                    else
                        Mode4LaserChecking = 0;
                }
                else
                {
                    Mode4LaserChecking = 0;
                }
            }
        }
    }
}

void TIM2_IRQHandler(void)
{
    static uint16_t tim_10ms = 0;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        SystemTickMs++;
        tim_10ms++;

        if (tim_10ms >= 10)
        {
            if (TrackingEnabled)
            {
                if (x != 0 &&
                    (uint32_t)(SystemTickMs - Serial_LastRxMs) <= TARGET_STALE_MS)
                {
                    uint32_t frame_id = Serial_RxFrameCount;
                    if (frame_id != Mode4ControlLastFrameId)
                    {
                        Mode4ControlLastFrameId = frame_id;
                        EMM_Mode4_Control(&Yaw_Motor, &Yaw_PID,
                                          (float)Gimbal_Target_Offset_X);
                    }
                }
                else
                {
                    StopMotor();
                    EMM_Mode4_Reset();
                }
            }
            tim_10ms = 0;
        }

        Key_Tick();
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
