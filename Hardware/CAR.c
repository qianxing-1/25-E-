#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "chuangan.h"
#include "Delay.h"

int basespeed=72;
uint32_t time1=530;    //直角转弯延时
uint32_t time2=1060;    //掉头转弯延时
extern int TargetL,TargetR,temp,distance;



void car_Init(void)
{
	Motor_Init();
}


void straight(void)    //直行
{
	if(Z == 1 && L1 == 0 && L2 == 0 && L3 == 0 && R1 == 0 && R2 == 0 && R3 == 0)
	{
		TargetL=basespeed;
		TargetR=basespeed;
			
	}
		
	if( L3 == 1 )
	{
		TargetL=basespeed-5;
		TargetR=basespeed;
	}
	if( R1 == 1 )
	{
		TargetL=basespeed;
		TargetR=basespeed-5;
	}
		
	 if( L2 == 1 )
	{
		TargetL=basespeed-15;
		TargetR=basespeed;
	}
	if( R2 == 1 )
	{
		TargetL=basespeed;
		TargetR=basespeed-15;
	}
	
	if( R3 == 1 )
	{
		TargetL=basespeed;
		TargetR=basespeed-25;
	}
	
	if( L1 == 1 )
	{
		TargetL=basespeed-25;
		TargetR=basespeed;
	}
//	if(Z == 0 && L1 == 0 && L2 == 0 && L3 == 0 && R1 == 0 && R2 == 0 && R3 == 0)
//	{
//		TargetL=0;
//		TargetR=0;
			
//	}
		
}

void bigleft(void)
{
	TargetL=-basespeed;
	TargetR=basespeed;
}

void bigright(void)
{
	TargetL=basespeed;
	TargetR=-basespeed;
}

void stop(void)
{
	TargetL=0;
	TargetR=0;
}

void num1(void)
{	
	while(distance<=72)
	{
		straight();
	}
		bigleft();
		Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=50)
	{					
	straight();
	}
	stop();
	Delay_s(1);
	bigleft();  //掉头
	Delay_ms(time2);
	temp=0;
	distance=0;
	while(distance<=50)
	{
	straight();
	}
	bigright();
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=72)
	{
	straight();
	}
	temp=0;
	distance=0;
	while(1)
		stop();
}

void num2(void)
{	
	while(distance<=72)
	{
		straight();
	}
		bigright();
		Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=50)
	{					
	straight();
	}
	stop();
	Delay_s(1);
	bigright();
	Delay_ms(time2);
	temp=0;
	distance=0;
	while(distance<=50)
	{
	straight();
	}
	bigleft();
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=72)
	{
	straight();
	}
	temp=0;
	distance=0;
	while(1)
		stop();
}

void num5(void)
{	
	while(distance<=162)
	{
		straight();
	}
	bigleft();
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=47)
	{
		straight();
	}
	stop();
	Delay_s(1);
		bigleft();
		Delay_ms(time2);
	temp=0;
	distance=0;
	while(distance<=47)
	{
		straight();
	}
	bigright();
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=162)
	{
	straight();
	}
	temp=0;
	distance=0;
	while(1)
		stop();
}

void num7(void)
{	
	while(distance<=162)
	{
		straight();
	}
	bigright();
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=47)
	{
		straight();
	}
	stop();
	Delay_s(1);
		bigleft();
		Delay_ms(time2);
	temp=0;
	distance=0;
	while(distance<=48)
	{
		straight();
	}
	bigleft();
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=162)
	{
	straight();
	}
	temp=0;
	distance=0;
	while(1)
		stop();
}
void num4(void)
{	
	while(distance<=253)
	{
		straight();
	}
	bigleft();
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=89)
	{
		straight();
	}
	bigright();
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=50)
	{
		straight();
	}
	stop();
	Delay_s(1);
		bigleft();  //掉头
		Delay_ms(time2);
	temp=0;
	distance=0;
	while(distance<=50)
	{					
	straight();
	}
	bigleft();//转弯
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=90)
	{					
	straight();
	}
	bigright();   //转弯
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=252)
	{
	straight();
	}
	temp=0;
	distance=0;
	while(1)
		stop();
}
void num8(void)
{	
	while(distance<=253)
	{
		straight();
	}
	bigleft();
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=89)
	{
		straight();
	}
	bigleft();
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=50)
	{
		straight();
	}
	stop();
	Delay_s(1);
		bigleft();  //掉头
		Delay_ms(time2);
	temp=0;
	distance=0;
	while(distance<=47)
	{					
	straight();
	}
	bigright();//转弯
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=90)
	{					
	straight();
	}
	bigright();   //转弯
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=252)
	{
	straight();
	}
	temp=0;
	distance=0;
	while(1)
		stop();
}
void num6(void)
{	
	while(distance<=253)
	{
		straight();
	}
	bigright();
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=88)
	{
		straight();
	}
	bigright();
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=50)
	{
		straight();
	}
	stop();
	Delay_s(1);
		bigleft();  //掉头
		Delay_ms(time2);
	temp=0;
	distance=0;
	while(distance<=49)
	{					
	straight();
	}
	bigleft();//转弯
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=90)
	{					
	straight();
	}
	bigleft();   //转弯
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=252)
	{
	straight();
	}
	temp=0;
	distance=0;
	while(1)
		stop();
}
void num3(void)
{	
	while(distance<=253)
	{
		straight();
	}
	bigright();
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=88)
	{
		straight();
	}
	bigleft();
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=50)
	{
		straight();
	}
	stop();
	Delay_s(1);
		bigleft();  //掉头
		Delay_ms(time2);
	temp=0;
	distance=0;
	while(distance<=49)
	{					
	straight();
	}
	bigright();//转弯
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=90)
	{					
	straight();
	}
	bigleft();   //转弯
	Delay_ms(time1);
	temp=0;
	distance=0;
	while(distance<=252)
	{
	straight();
	}
	temp=0;
	distance=0;
	while(1)
		stop();
}
