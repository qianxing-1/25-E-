#ifndef __CHUANGAN_H
#define __CHUANGAN_H

#define L1 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)//读取 PB12
#define L2 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)//读取 PB13
#define L3 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)//读取 PB14
                                                                
#define Z GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)//读取 PB15

#define R1 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)//读取 PA12
#define R2 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)//读取 PA15
#define R3 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)//读取 PB3           传感器从左到右分别为L1,L2,L3,Z,R1,R2,R3

void chuangan_Init(void);



#endif

