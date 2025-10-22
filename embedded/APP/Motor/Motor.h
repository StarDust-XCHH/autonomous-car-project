#ifndef __Motor_H
#define __Motor_H


#include <stdint.h>

// ����ṹ��
typedef struct {
	
		int whoAmI;					//���������ֱ�־����peropheralInit�����Ӧ�ı�����һͬ��ʼ��
    int pwmDutyCycle;   // PWMʵ��ռ�ձȣ���Χ0-500-1000����ʼ����ֵ��PWM��ʼ����
		int pwmMirror;			// ӳ��PWM����Χ-500~0~+500,��pwm��ʼ����Эͬ��ʼ��
		
    double motorSpeed;   // ���ʵ��ת�٣���λcircle/min����PID�ٶȻ��г�ʼ����ֵ
		double targetSpeed_perMin;//���Ŀ��ת�٣����ⲿ�豸��ֵ����main�еļ�ʱ��ѭ���ڸ�ֵ
	
		//���²�����pid��ʼ���г�ʼ����ֵ
		double SpeedError;
		double SpeedErrorIteger;
		double SpeedErrorDerivate;
		double SpeedErrorLast;
		double kp,ki,kd;
		double PID_output;
		int PWM_output;
} MotorInfo;

//--------------set����--------------
// ����PWMʵ��ռ�ձ�
void setPwmDutyCycle(MotorInfo* motor, int pwmDutyCycle);

// ����ӳ��PWM
void setPwmMirror(MotorInfo* motor, int pwmMirror);

//���õ��ʵ��ת��
void setMotorInfoSpeed(MotorInfo* motor, double speed);

// ���õ��Ŀ��ת��
void setTargetSpeed_perMin(MotorInfo* motor, double targetSpeed);

// �����ٶ����
void setSpeedError(MotorInfo* motor, double error);

// �����ٶ�������
void setSpeedErrorIteger(MotorInfo* motor, double errorInteger);

// �����ٶ����΢��
void setSpeedErrorDerivate(MotorInfo* motor, double errorDerivate);

// ������һ�ε��ٶ����
void setSpeedErrorLast(MotorInfo* motor, double errorLast);

// ���ñ���ϵ��kp
void setKp(MotorInfo* motor, double kp);

// ���û���ϵ��ki
void setKi(MotorInfo* motor, double ki);

// ����΢��ϵ��kd
void setKd(MotorInfo* motor, double kd);

// ����PID����
void setMotorInfoPID_para(MotorInfo* motor, double kp,double ki,double kd);

// ���������
void setError_para(MotorInfo* motor, double error, double errorInteger, double errorDerivate, double errorLast);

//--------------PID��ʼ��--------------

// PID��ʼ��
void PID_init(MotorInfo* motor);


//--------------PID�ٶȻ�--------------

void PID_loop(MotorInfo* motor);

//--------------�������ٶȼ��㹤��--------------
double encoderSpeedCalculate(MotorInfo* motor);

#endif

