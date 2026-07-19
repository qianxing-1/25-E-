#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>
#include "Serial.h"

uint16_t Serial_RxData;
uint16_t Serial_RxFlag;
volatile uint32_t Serial_LastRxMs = 0;
volatile uint32_t Serial_RxFrameCount = 0;

extern volatile uint16_t x;
extern volatile uint16_t y;
extern volatile int16_t Gimbal_Target_Offset_X;
extern volatile uint32_t SystemTickMs;

void Serial_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate = 115200;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &usart);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&nvic);

    USART_Cmd(USART2, ENABLE);
}

void Serial_SendByte(uint8_t byte)
{
    USART_SendData(USART2, byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *array, uint16_t length)
{
    uint16_t i;
    for (i = 0; i < length; i++)
        Serial_SendByte(array[i]);
}

void Serial_SendString(char *string)
{
    uint16_t i;
    for (i = 0; string[i] != '\0'; i++)
        Serial_SendByte(string[i]);
}

int fputc(int ch, FILE *f)
{
    (void)f;
    Serial_SendByte((uint8_t)ch);
    return ch;
}

void Serial_Printf(char *format, ...)
{
    char string[100];
    va_list arg;

    va_start(arg, format);
    vsprintf(string, format, arg);
    va_end(arg);
    Serial_SendString(string);
}

uint8_t Serial_GetRxFlag(void)
{
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;
        return 1;
    }
    return 0;
}

uint8_t Serial_GetRxData(void)
{
    return (uint8_t)Serial_RxData;
}

void Serial_ReadTarget(uint16_t *target_x, uint16_t *target_y,
                       uint32_t *last_rx_ms, uint32_t *frame_id)
{
    uint32_t before;
    uint32_t after;

    do
    {
        before = Serial_RxFrameCount;
        *target_x = x;
        *target_y = y;
        *last_rx_ms = Serial_LastRxMs;
        after = Serial_RxFrameCount;
    } while (before != after);

    *frame_id = after;
}

void USART2_IRQHandler(void)
{
    uint8_t data;
    static uint8_t rx_count = 0;
    static uint8_t rx_buffer[7] = {0};
    static uint8_t rx_state = 0;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) == RESET)
        return;

    data = (uint8_t)USART_ReceiveData(USART2);

    if (rx_state == 0)
    {
        if (data == 0x2C)
        {
            rx_buffer[0] = data;
            rx_count = 1;
            rx_state = 1;
        }
    }
    else if (rx_state == 1)
    {
        if (data == 0x12)
        {
            rx_buffer[1] = data;
            rx_count = 2;
            rx_state = 2;
        }
        else if (data == 0x2C)
        {
            rx_buffer[0] = data;
            rx_count = 1;
        }
        else
        {
            rx_count = 0;
            rx_state = 0;
        }
    }
    else
    {
        if (rx_count < 7)
            rx_buffer[rx_count++] = data;

        if (rx_count == 7)
        {
            if (rx_buffer[6] == 0x5B)
            {
                uint16_t x_raw = ((uint16_t)rx_buffer[2] << 8) | rx_buffer[3];
                uint16_t y_raw = ((uint16_t)rx_buffer[4] << 8) | rx_buffer[5];

                x = x_raw;
                y = y_raw;
                Gimbal_Target_Offset_X =
                    (x_raw != 0) ? ((int16_t)x_raw - 320) : 0;
                Serial_LastRxMs = SystemTickMs;
                Serial_RxFrameCount++;
                Serial_RxFlag = 1;
            }

            if (data == 0x2C)
            {
                rx_buffer[0] = data;
                rx_count = 1;
                rx_state = 1;
            }
            else
            {
                rx_count = 0;
                rx_state = 0;
            }
        }
    }
}
