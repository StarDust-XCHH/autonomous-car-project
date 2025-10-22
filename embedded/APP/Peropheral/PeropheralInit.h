#ifndef __PeropheralInit_H
#define __PeropheralInit_H


#include "Motor.h"


//外设初始化使能封装
void peripheral_Initial(void);


//陀螺仪初始化
void imu_Initial(void);

typedef struct {
	
	int ires;		//陀螺仪读取数据状态标识
	
	float iyaw;
	float ipitch;
	float iroll;

	
	
} imuInfo;

//PWM初始化
void PWM_Initial(void);



//定时器-编码器初始化

void Tim6Encoder(void);

#endif
