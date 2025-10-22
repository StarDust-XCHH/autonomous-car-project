#ifndef __Motor_H
#define __Motor_H


#include <stdint.h>

// 定义结构体
typedef struct {
	
		int whoAmI;					//左右轮区分标志，在peropheralInit中与对应的编码器一同初始化
    int pwmDutyCycle;   // PWM实际占空比，范围0-500-1000，初始化赋值在PWM初始化中
		int pwmMirror;			// 映射PWM，范围-500~0~+500,在pwm初始化中协同初始化
		
    double motorSpeed;   // 电机实际转速，单位circle/min，在PID速度环中初始化赋值
		double targetSpeed_perMin;//电机目标转速，由外部设备赋值，在main中的计时器循环内赋值
	
		//以下参数在pid初始化中初始化赋值
		double SpeedError;
		double SpeedErrorIteger;
		double SpeedErrorDerivate;
		double SpeedErrorLast;
		double kp,ki,kd;
		double PID_output;
		int PWM_output;
} MotorInfo;

//--------------set函数--------------
// 设置PWM实际占空比
void setPwmDutyCycle(MotorInfo* motor, int pwmDutyCycle);

// 设置映射PWM
void setPwmMirror(MotorInfo* motor, int pwmMirror);

//设置电机实际转速
void setMotorInfoSpeed(MotorInfo* motor, double speed);

// 设置电机目标转速
void setTargetSpeed_perMin(MotorInfo* motor, double targetSpeed);

// 设置速度误差
void setSpeedError(MotorInfo* motor, double error);

// 设置速度误差积分
void setSpeedErrorIteger(MotorInfo* motor, double errorInteger);

// 设置速度误差微分
void setSpeedErrorDerivate(MotorInfo* motor, double errorDerivate);

// 设置上一次的速度误差
void setSpeedErrorLast(MotorInfo* motor, double errorLast);

// 设置比例系数kp
void setKp(MotorInfo* motor, double kp);

// 设置积分系数ki
void setKi(MotorInfo* motor, double ki);

// 设置微分系数kd
void setKd(MotorInfo* motor, double kd);

// 设置PID参数
void setMotorInfoPID_para(MotorInfo* motor, double kp,double ki,double kd);

// 设置误差项
void setError_para(MotorInfo* motor, double error, double errorInteger, double errorDerivate, double errorLast);

//--------------PID初始化--------------

// PID初始化
void PID_init(MotorInfo* motor);


//--------------PID速度环--------------

void PID_loop(MotorInfo* motor);

//--------------编码器速度计算工具--------------
double encoderSpeedCalculate(MotorInfo* motor);

#endif

