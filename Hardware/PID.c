#include "stm32f10x.h"                  // Device header

float kp=0.5;
float ki=0.12;
float kd=0.003;




float PIDL(float set,int16_t act)
{
	static float err0 = 0, err1 = 0, errint = 0;
	float out;
	
	err1=err0;
	err0=set-act;
	
	if(ki!=0)
		errint+=err0;
	else
		errint=0;
	
	if (errint > 80) {errint = 80;}
	if (errint < -80) {errint = -80;}
	
	out = kp * err0 + ki * errint + kd * (err0 - err1);
	
	if(out>100) {out=100;}
	if(out<-100) {out=-100;}
	
	return out;
}

float PIDR(float set,int16_t act)
{
	static float err0 = 0, err1 = 0, errint = 0;
	float out;
	
	err1=err0;
	err0=set-act;
	
	if(ki!=0)
		errint+=err0;
	else
		errint=0;
	
	if (errint > 80) {errint = 80;}
	if (errint < -80) {errint = -80;}
	
	out = kp * err0 + ki * errint + kd * (err0 - err1);
	
	if(out>100) {out=100;}
	if(out<-100) {out=-100;}
	
	return out;
}
