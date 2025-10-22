#ifndef __PeropheralInit_H
#define __PeropheralInit_H


#include "Motor.h"


//�����ʼ��ʹ�ܷ�װ
void peripheral_Initial(void);


//�����ǳ�ʼ��
void imu_Initial(void);

typedef struct {
	
	int ires;		//�����Ƕ�ȡ����״̬��ʶ
	
	float iyaw;
	float ipitch;
	float iroll;

	
	
} imuInfo;

//PWM��ʼ��
void PWM_Initial(void);



//��ʱ��-��������ʼ��

void Tim6Encoder(void);

#endif
