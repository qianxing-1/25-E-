#include "stm32f10x.h"
#include "Delay.h"
#include "Timer.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "step_pwm.h"
#include "EMM_Gimbal.h"

/**
 * @brief TI杯电赛单轴Yaw云台主控程序（优化分层死区防过冲版本）
 * @author 电赛视觉自瞄配套STM32主控
 * @note 按键逻辑：PC14切换模式，PC15执行当前选中模式
 * @note 通信配套K230 CanMV视觉，串口上传320分辨率装甲X/Y坐标+帧ID+接收时间戳
 * @note 硬件：PA10(DIR) PA11(STEP) PA12(ENA) 两相步进电机；PB0控制激光指示灯
 */

uint8_t KeyNum = 0;                     // 按键返回值：1=模式切换，2=执行模式
volatile uint16_t x = 0;                // 串口中断更新：视觉识别装甲X坐标(0~319)
volatile uint16_t y = 0;                // 串口中断更新：视觉识别装甲Y坐标
volatile int16_t Gimbal_Target_Offset_X = 0; // 画面X轴误差：目标-画面中心
volatile uint32_t SystemTickMs = 0;     // TIM2 1ms系统毫秒计时戳，全局时序基准

/* ===================== 视觉瞄准分层死区参数(核心防过冲优化) ===================== */
#define SCREEN_CENTER_X          320    // K230识别画面水平中心(320宽度分辨率)
#define CENTER_ENTER_ERROR       5      // 进入稳定区阈值：误差±5像素内开始计时稳定
#define CENTER_EXIT_ERROR        8      // 退出稳定区阈值：误差超出±8像素直接重置计时
#define CENTER_FINAL_ERROR       4      // 最终锁死判定阈值：稳定200ms后误差必须小于±4才点亮激光
#define ERROR_DEADBAND           CENTER_ENTER_ERROR // 跟踪模式基础停机死区复用进入阈值
#define MODE4_TRACK_DEADBAND     2      // 持续运动跟踪使用更小死区，降低动态滞后
#define CENTER_SETTLE_MS         200    // 稳定持续计时窗口：连续200ms停留在分层死区内判定对准
#define MODE4_LASER_CONFIRM_MS   80     // 模式4仅确认激光，不阻塞连续跟踪
#define TARGET_STALE_MS          150    // 视觉数据超时阈值：150ms无新帧判定目标丢失，停机

/* ===================== 模式1：固定左转90°旋转参数 ===================== */
/* 步进电机1.8°整步，8细分，一圈400脉冲；90°为1/4圈 */
#define MODE1_STEP_FREQ          200    // 固定旋转模式脉冲输出频率200Hz
#define MODE1_DURATION_MS        2000   // 左转90°持续运行2000ms

/* ===================== 模式2/3：自动搜索粗搜+精调参数 ===================== */
#define SEARCH_FAST_SPEED        200    // 未识别到目标时高速搜索转速
#define SEARCH_TIMEOUT_MS        4000   // 全程搜索总超时4s，找不到靶直接退出
#define AIM_TIMEOUT_MS           3000   // 识别到目标后精调锁定超时3s，持续对不准则退出搜索

/* ===================== 全局状态标志 ===================== */
static uint8_t LaserOn = 0;                    // 激光指示灯状态：0关闭/1点亮
static uint8_t CurrentMode = 1;                 // 当前选中工作模式：1固定左转/2左搜靶/3右搜靶/4持续跟踪
static uint8_t ModeActive = 0;                  // 模式执行锁：1正在运行动作，禁止切换按键
static volatile uint8_t TrackingEnabled = 0;   // 模式4持续跟踪总使能开关
static uint8_t Mode4LaserChecking = 0;          // 激光对准确认计时，不参与电机停机
static uint32_t Mode4LaserStartMs = 0;          // 模式4激光确认起始时间戳
static uint32_t Mode4LastFrameId = 0;           // 上一帧视觉数据包ID，过滤重复帧重复运算

EMM_Motor Yaw_Motor;    // Yaw偏航步进电机驱动结构体
PID_Controller Yaw_PID;// 单轴视觉闭环PID控制器实例

/**
 * @brief 激光引脚初始化 PB0推挽输出，上电默认低电平关灯
 */
static void Laser_Init(void)
{
    GPIO_InitTypeDef gpio;
    // 开启GPIOB外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;      // 推挽输出
    gpio.GPIO_Pin = GPIO_Pin_0;             // PB0激光控制引脚
    gpio.GPIO_Speed = GPIO_Speed_50MHz;     // IO速度50MHz
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);      // 上电默认关闭激光
}

/**
 * @brief 打开激光指示灯
 */
static void Laser_On(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_0);
    LaserOn = 1;
}

/**
 * @brief 电机完整停机函数：清空PWM脉冲+清零速度+驱动器失能，抑制抖动、降低发热
 */
static void StopMotor(void)
{
    STEP_PWM_SetFreq(0);        // PWM脉冲输出置0，停止发步进脉冲
    Yaw_Motor.Step_Frequency = 0;
    EMM_Disable(&Yaw_Motor);     // 步进驱动器失能，电机无自锁力矩
}

/**
 * @brief PID控制器状态重置：清空积分累积、历史误差，防止积分饱和冲靶
 * @note 切换模式、丢失目标、重新搜索时必须调用
 */
static void ResetAimPID(void)
{
    Yaw_PID.Integral = 0.0f;
    Yaw_PID.Last_Error = 0.0f;
}

/**
 * @brief 视觉数据新鲜度校验
 * @param target_x 视觉回传X坐标
 * @param last_rx_ms 上一帧数据接收时间戳
 * @return 1=数据有效有目标；0=无目标/数据超时失效
 */
static uint8_t TargetIsFresh(uint16_t target_x, uint32_t last_rx_ms)
{
    // 两个条件同时满足：坐标非0有目标、帧接收间隔不超过超时阈值
    return target_x != 0 &&
           (uint32_t)(SystemTickMs - last_rx_ms) <= TARGET_STALE_MS;
}

/**
 * @brief OLED打印模式选择界面
 */
static void ShowMode(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "Mode:");
    OLED_ShowNum(1, 6, CurrentMode, 1);
    OLED_ShowString(2, 1, "PC14=Sel PC15=Run");
}

/**
 * @brief OLED打印运行中提示文字
 */
static void ShowRunning(void)
{
    OLED_ShowString(4, 1, "Running...");
}

/**
 * @brief 启动高速粗搜索：停机→驱动器使能→设置转向→开启固定高速脉冲
 * @param direction 0右转 / 1左转
 */
static void StartFastSearch(uint8_t direction)
{
    StopMotor();
    EMM_Enable(&Yaw_Motor);
    EMM_Set_Direction(&Yaw_Motor, direction);
    STEP_PWM_SetFreq(SEARCH_FAST_SPEED);
    Yaw_Motor.Step_Frequency = SEARCH_FAST_SPEED;
}

/**
 * @brief 模式1：固定向左旋转90°
 */
static void Mode1_TurnLeft90(void)
{
    ModeActive = 1;             // 上锁动作，禁止按键切换模式
    TrackingEnabled = 0;       // 关闭持续跟踪
    ShowRunning();              // 屏幕显示运行提示

    StopMotor();
    EMM_Enable(&Yaw_Motor);
    EMM_Set_Direction(&Yaw_Motor, 1);       // 转向左
    STEP_PWM_SetFreq(MODE1_STEP_FREQ);      // 设置旋转速度
    Delay_ms(MODE1_DURATION_MS);            // 持续延时完成90°转角
    StopMotor();

    Laser_On();                 // 旋转完成点亮激光
    ModeActive = 0;             // 解锁动作
    ShowMode();                 // 切回模式选择界面
}

/**
 * @brief 通用搜索瞄准核心函数：粗搜→识别目标PID精调→分层死区稳定判定
 * @param initial_direction 初始搜索方向 0右/1左
 * @return 1成功锁定靶心；0超时未找到
 * @note 分层死区逻辑：进入±5开始计时，超出±8重置计时，稳定200ms且误差<±4判定锁定
 */
static uint8_t SearchAndAim(uint8_t initial_direction)
{
    uint32_t search_start_ms = SystemTickMs;    // 搜索流程起始时间戳
    uint32_t aim_start_ms = 0;                  // 识别到目标后精调起始时间戳
    uint32_t last_frame_id = 0;                // 上一帧有效视觉ID，过滤重复包
    uint32_t settle_start_ms = 0;              // 进入稳定死区的计时起点
    uint8_t settling = 0;                      // 稳定计时标志
    uint8_t searching = 1;                      // 粗搜阶段标志
    uint8_t target_acquired = 0;               // 是否捕获到有效目标标志

    ResetAimPID();                              // 初始化清空PID积分
    StartFastSearch(initial_direction);         // 启动初始高速粗搜

    while (1)
    {
        uint16_t target_x;
        uint16_t target_y;
        uint32_t last_rx_ms;
        uint32_t frame_id;

        // 读取完整视觉数据包：X/Y坐标、接收时间戳、帧序号
        Serial_ReadTarget(&target_x, &target_y, &last_rx_ms, &frame_id);
        (void)target_y; // 当前仅单轴Yaw，Y坐标暂不使用，屏蔽未使用警告

        // 超时分支判定
        if (!target_acquired)
        {
            // 未捕获目标：总搜索4s超时直接退出循环
            if ((uint32_t)(SystemTickMs - search_start_ms) >= SEARCH_TIMEOUT_MS)
                break;
        }
        else if ((uint32_t)(SystemTickMs - aim_start_ms) >= AIM_TIMEOUT_MS)
        {
            // 已捕获目标但3s持续无法稳定对准，退出
            break;
        }

        // 视觉数据失效（超时/无目标）
        if (!TargetIsFresh(target_x, last_rx_ms))
        {
            settling = 0; // 清空稳定计时
            if (target_acquired)
            {
                // 之前捕获到目标，现在丢失，切回粗搜状态重置计时
                target_acquired = 0;
                search_start_ms = SystemTickMs;
            }
            // 当前不在粗搜状态，重新开启高速搜索
            if (!searching)
            {
                ResetAimPID();
                StartFastSearch(initial_direction);
                searching = 1;
            }
        }
        // 收到全新有效视觉帧，执行PID精调逻辑
        else if (frame_id != last_frame_id)
        {
            int16_t error = (int16_t)target_x - SCREEN_CENTER_X; // 计算画面水平误差
            last_frame_id = frame_id;
            searching = 0; // 退出粗搜，进入PID精调

            // 首次捕获到目标，记录精调起始时间戳
            if (!target_acquired)
            {
                target_acquired = 1;
                aim_start_ms = SystemTickMs;
            }

            // 未进入稳定计时，且误差落入进入死区±5，停机启动稳定计时
            if (!settling &&
                error >= -CENTER_ENTER_ERROR && error <= CENTER_ENTER_ERROR)
            {
                StopMotor();
                settle_start_ms = SystemTickMs;
                settling = 1;
            }
            // 正在稳定计时中
            else if (settling)
            {
                // 误差超出退出阈值±8，判定目标偏移，重置计时恢复PID调节
                if (error < -CENTER_EXIT_ERROR || error > CENTER_EXIT_ERROR)
                {
                    settling = 0;
                    EMM_Visual_Control(&Yaw_Motor, &Yaw_PID, (float)error);
                }
                // 稳定时长达到200ms
                else if ((uint32_t)(SystemTickMs - settle_start_ms) >= CENTER_SETTLE_MS)
                {
                    // 最终校验误差小于±4，判定完全锁定，返回成功
                    if (error >= -CENTER_FINAL_ERROR && error <= CENTER_FINAL_ERROR)
                        return 1;
                    // 未满足最终精度，重置计时继续调节
                    settling = 0;
                    EMM_Visual_Control(&Yaw_Motor, &Yaw_PID, (float)error);
                }
            }
            // 未进入稳定死区，持续PID调速修正误差
            else
            {
                EMM_Visual_Control(&Yaw_Motor, &Yaw_PID, (float)error);
            }
        }

        Delay_ms(2); // 循环最小延时，降低CPU占用
    }

    StopMotor();
    return 0; // 搜索超时未锁定目标
}

/**
 * @brief 模式2：向左自动搜索靶心
 */
static void Mode2_SearchLeft(void)
{
    ModeActive = 1;
    TrackingEnabled = 0;
    ShowRunning();

    // 向左搜索，锁定成功点亮激光
    if (SearchAndAim(1))
        Laser_On();

    StopMotor();
    ModeActive = 0;
    ShowMode();
}

/**
 * @brief 模式3：向右自动搜索靶心
 */
static void Mode3_SearchRight(void)
{
    ModeActive = 1;
    TrackingEnabled = 0;
    ShowRunning();

    // 向右搜索，锁定成功点亮激光
    if (SearchAndAim(0))
        Laser_On();

    StopMotor();
    ModeActive = 0;
    ShowMode();
}

/**
 * @brief 模式4初始化：开启持续实时跟踪界面，重置稳定状态与PID
 */
static void Mode4_StartTracking(void)
{
    TrackingEnabled = 1;
    Mode4LaserChecking = 0;
    Mode4LaserStartMs = 0;
    Mode4LastFrameId = 0;
    ResetAimPID();
    EMM_Tracking_Reset();
    // 切换跟踪专用OLED显示界面
    OLED_Clear();
    OLED_ShowString(1, 1, "Mode:4 Align");
    OLED_ShowString(2, 1, "X:         ");
    OLED_ShowString(3, 1, "Y:         ");
    OLED_ShowString(4, 1, "Laser:OFF  ");
}

/**
 * @brief Yaw偏航轴步进电机硬件初始化
 * @note STEP=PA11 DIR=PA10 ENA=PA12
 * @note PID参数：纯比例P=15，无积分微分，输出限幅±600Hz
 */
void Yaw_Init(void)
{
    Yaw_Motor.STEP_Port = GPIOA;
    Yaw_Motor.STEP_Pin = GPIO_Pin_11;
    Yaw_Motor.DIR_Port = GPIOA;
    Yaw_Motor.DIR_Pin = GPIO_Pin_10;
    Yaw_Motor.ENA_Port = GPIOA;
    Yaw_Motor.ENA_Pin = GPIO_Pin_12;
    EMM_Motor_Init(&Yaw_Motor);

    // PID初始化 Kp=15 Ki=0 Kd=0 输出限幅[-600,600]
    PID_Init(&Yaw_PID, 15.0f, 0.0f, 0.0f, 600.0f, -600.0f);
}

/**
 * @brief 主函数：硬件初始化+无限主循环，处理按键、视觉数据、模式4界面刷新
 */
int main(void)
{
    // 全部外设初始化
    Serial_Init();      // 串口初始化，接收K230视觉数据
    Key_Init();         // PC14/PC15按键初始化
    OLED_Init();        // 屏幕初始化
    STEP_PWM_Init();    // 步进脉冲PWM定时器初始化
    Yaw_Init();         // Yaw电机与PID初始化
    Laser_Init();       // 激光IO初始化
    Timer_Init();       // TIM2系统计时定时器初始化
    ShowMode();         // 上电显示模式选择界面

    while (1)
    {
        uint16_t target_x;
        uint16_t target_y;
        uint32_t last_rx_ms;
        uint32_t frame_id;

        KeyNum = Key_GetNum(); // 读取按键扫描结果

        // PC14按键：切换模式(仅无动作运行时允许切换)
        if (KeyNum == 1 && !ModeActive)
        {
            TrackingEnabled = 0; // 关闭跟踪
            StopMotor();        // 停机
            CurrentMode++;
            if (CurrentMode > 4)
                CurrentMode = 1; // 模式循环 1→2→3→4→1

            if (CurrentMode == 4)
                Mode4_StartTracking(); // 切到跟踪模式初始化界面
            else
                ShowMode();            // 普通模式显示选择界面
        }

        // PC15按键：执行当前选中模式(动作运行时屏蔽)
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

        // 读取最新一帧完整视觉数据包
        Serial_ReadTarget(&target_x, &target_y, &last_rx_ms, &frame_id);
        // ========== 模式4 持续跟踪界面刷新、激光判定逻辑 ==========
        if (CurrentMode == 4)
        {
            static uint16_t last_x = 0xFFFF;
            static uint16_t last_y = 0xFFFF;

            // 坐标发生变化才刷新OLED，消除屏幕残影、减少刷屏卡顿
            if (target_x != last_x || target_y != last_y)
            {
                last_x = target_x;
                last_y = target_y;
                OLED_ShowString(2, 3, "   "); // 清空原有数字
                OLED_ShowNum(2, 3, target_x, 3);
                OLED_ShowString(3, 3, "   ");
                OLED_ShowNum(3, 3, target_y, 3);
            }

            // 未点亮激光，且收到全新视觉帧，执行对准稳定判断
            if (!LaserOn && frame_id != Mode4LastFrameId)
            {
                Mode4LastFrameId = frame_id;

                // 视觉数据有效新鲜
                if (TargetIsFresh(target_x, last_rx_ms))
                {
                    int16_t error = (int16_t)target_x - SCREEN_CENTER_X;
                    // 激光确认与电机连续跟踪解耦，不暂停云台
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
                // 视觉数据失效，丢失目标重置稳定标志
                else
                {
                    Mode4LaserChecking = 0;
                }
            }
        }
    }
}

/**
 * @brief TIM2更新中断服务函数 1ms中断
 * @note 功能：1ms系统计时自增；每10ms执行一次跟踪PID闭环运算；定时按键扫描
 */
void TIM2_IRQHandler(void)
{
    static uint16_t tim_10ms = 0; // 10ms分频计数器

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        SystemTickMs++; // 每进一次中断+1，基准1ms计时
        tim_10ms++;

        // 每10ms执行一次跟踪闭环控制
        if (tim_10ms >= 10)
        {
            if (TrackingEnabled)
            {
                // 有有效目标且数据未超时，执行连续PID调速
                if (x != 0 &&
                    (uint32_t)(SystemTickMs - Serial_LastRxMs) <= TARGET_STALE_MS)
                {
                    int16_t error = Gimbal_Target_Offset_X;
                    // 死区外使用带加减速斜坡的持续跟踪控制
                    if (error > MODE4_TRACK_DEADBAND ||
                        error < -MODE4_TRACK_DEADBAND)
                        EMM_Tracking_Control(&Yaw_Motor, &Yaw_PID, (float)error);
                    else
                    {
                        EMM_Tracking_Control(&Yaw_Motor, &Yaw_PID, 0.0f);
                        EMM_Enable(&Yaw_Motor);
                    }
                }
                // 无目标/数据超时直接停机
                else
                {
                    StopMotor();
                    EMM_Tracking_Reset();
                }
            }
            tim_10ms = 0; // 重置10ms分频计数
        }

        Key_Tick(); // 定时按键防抖扫描
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // 清除中断标志位
    }
}
