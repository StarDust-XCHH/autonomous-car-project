#include "Motor.h"
#include <stdio.h>
#include "tim.h"
#include "math.h"

extern float btSpeedRight ;
extern float btSpeedLeft ;

//--------------set函数--------------
// 设置PWM实际占空比
void setPwmDutyCycle(MotorInfo* motor, int pwmDutyCycle){
    if (motor != NULL) {
				//赋值实际占空比
        motor->pwmDutyCycle = pwmDutyCycle;
			
				//赋值映射占空比
				motor->pwmMirror = pwmDutyCycle - 500 ;
			
			
				if(motor->whoAmI==1){//在这里改为电机1的判断逻辑
				// 假设htim8和TIM_CHANNEL_1已经在其他地方正确定义并初始化
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmDutyCycle);
				}	
				else if(motor->whoAmI==2){
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmDutyCycle);
				}
			
    }
}

// 设置映射PWM
void setPwmMirror(MotorInfo* motor, int pwmMirror){
    if (motor != NULL) {
			
				//赋值映射占空比
        motor->pwmMirror = pwmMirror;
			
				//赋值实际占空比
				motor->pwmDutyCycle = pwmMirror+500;
			
			
				if(motor->whoAmI==1){//在这里改为电机1的判断逻辑
				// 假设htim8和TIM_CHANNEL_1已经在其他地方正确定义并初始化
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmMirror+500);
				}
				else if(motor->whoAmI==2){
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmMirror+500);
				}
			

    }
}

//设置电机实际转速
void setMotorInfoSpeed(MotorInfo* motor, double speed){
    if (motor != NULL) {
        motor->motorSpeed = speed;
			
			if(motor->whoAmI==1){
				btSpeedRight = (float)speed;
			}
			else if(motor->whoAmI==2){
				btSpeedLeft = (float)speed;
			
			}
			
    }
}

// 设置电机目标转速
void setTargetSpeed_perMin(MotorInfo* motor, double targetSpeed){
    if (motor != NULL) {
			//限速每分钟300装
				if(targetSpeed>300){motor->targetSpeed_perMin=300;}
				else if(targetSpeed<-300){motor->targetSpeed_perMin=-300;}
				else{
				motor->targetSpeed_perMin = targetSpeed;
				}    
		}
}

// 设置速度误差
void setSpeedError(MotorInfo* motor, double error){
    if (motor != NULL) {
        motor->SpeedError = error;
    }
}

// 设置速度误差积分
void setSpeedErrorIteger(MotorInfo* motor, double errorInteger){
    if (motor != NULL) {
        motor->SpeedErrorIteger = errorInteger;
    }
}

// 设置速度误差微分
void setSpeedErrorDerivate(MotorInfo* motor, double errorDerivate){
    if (motor != NULL) {
        motor->SpeedErrorDerivate = errorDerivate;
    }
}

// 设置上一次的速度误差
void setSpeedErrorLast(MotorInfo* motor, double errorLast){
    if (motor != NULL) {
        motor->SpeedErrorLast = errorLast;
    }
}

// 设置比例系数kp
void setKp(MotorInfo* motor, double kp){
    if (motor != NULL) {
        motor->kp = kp;
    }
}

// 设置积分系数ki
void setKi(MotorInfo* motor, double ki){
    if (motor != NULL) {
        motor->ki = ki;
    }
}

// 设置微分系数kd
void setKd(MotorInfo* motor, double kd){
    if (motor != NULL) {
        motor->kd = kd;
    }
}


// 设置PID参数
void setMotorInfoPID_para(MotorInfo* motor, double kp,double ki,double kd) {
    if (motor != NULL) {
        setKp(motor,kp);
        setKi(motor,ki);
			  setKd(motor,kd);

    }
}

// 设置误差项
void setError_para(MotorInfo* motor, double error, double errorInteger, double errorDerivate, double errorLast){

    if (motor != NULL) {
			
			
        setSpeedError(motor,error);
        setSpeedErrorIteger(motor,errorInteger);
			  setSpeedErrorDerivate(motor,errorDerivate);
				setSpeedErrorLast(motor,errorLast);

    }


}

//--------------PID初始化--------------

// PID初始化
void PID_init(MotorInfo* motor){
	


	//设置PID参数
	
	double kp=2.0,ki=1.0,kd=0.0;
	setMotorInfoPID_para(motor,kp,ki,kd);
	

	
	//设置目标速度
	
	double targetSpeed = 0.0;
	setTargetSpeed_perMin(motor,targetSpeed);
	
	
	
	//初始化误差项
	
	double SpeedError=0.0;
	double SpeedErrorIteger=0.0;
	double SpeedErrorDerivate=0.0;
	double SpeedErrorLast=0.0;
	
	setError_para(motor,SpeedError,SpeedErrorIteger,SpeedErrorDerivate,SpeedErrorLast);

}

//--------------PID速度环--------------




void PID_loop(MotorInfo* motor){	
	

	setMotorInfoSpeed(motor,encoderSpeedCalculate(motor));

	//计算误差
	
	setSpeedError(motor,(motor->targetSpeed_perMin)-(motor->motorSpeed));
	
	setSpeedErrorIteger(motor,motor->SpeedErrorIteger+(motor->SpeedError)*0.05);//积分区间50ms
	
	

	
	setSpeedErrorDerivate(motor,(motor->SpeedError-(motor->SpeedErrorLast))/0.05);//微分区间50ms

	
	motor->PID_output = motor->kp*motor->SpeedError
										+motor->ki*motor->SpeedErrorIteger
										+motor->kd*motor->SpeedErrorDerivate;
	
	motor->PWM_output =(int) (motor->PID_output);
	
	if((motor->PWM_output)>500){motor->PWM_output=500;}
	if((motor->PWM_output)<-500){motor->PWM_output=-500;}

	
	//使用映射赋值
	setPwmMirror(motor,motor->PWM_output);
	
	
	
	
	//打印和更新数据
	
//	if(motor->whoAmI==1){
//		printf("m1: %f,%f,%d,%f\n", motor->motorSpeed,motor->targetSpeed_perMin,motor->pwmDutyCycle,motor->SpeedErrorIteger);
//	}
//	else if(motor->whoAmI==2){
//		printf("m2: %f,%f,%d,%f\n", motor->motorSpeed,motor->targetSpeed_perMin,motor->pwmDutyCycle,motor->SpeedErrorIteger);
//	}


	setSpeedErrorLast(motor,motor->SpeedError);

}

//--------------编码器速度计算工具--------------

int current_counter_1=0;
int32_t delta_counter_1=0;
int pre_counter_1=0;
double circle_perMin_1=0.0;

int current_counter_2=0;
int32_t delta_counter_2=0;
int pre_counter_2=0;
double circle_perMin_2=0.0;

double encoderSpeedCalculate(MotorInfo* motor){
	
	
	
	if(motor->whoAmI==1){
	
	
			//获取当前速度并计算
		current_counter_1 = __HAL_TIM_GET_COUNTER(&htim3); // 获取当前计数值
		
		// 计算未经修正的delta_counter
		delta_counter_1 = (int32_t)current_counter_1 - (int32_t)pre_counter_1;

		// 检查是否发生了溢出（假设是16位计数器）
		if (delta_counter_1 > 32767) {
				// 发生了向下溢出
				delta_counter_1 -= 65536; // 调整delta_counter
		} else if (delta_counter_1 < -32768) {
				// 发生了向上溢出
				delta_counter_1 += 65536; // 调整delta_counter
		}
		
		
		delta_counter_1 = delta_counter_1*20*60;//50ms测一次，一分钟60s，每分钟转速

		circle_perMin_1 = delta_counter_1 / 390.0 ;
		
		// 更新pre_counter为当前计数值
		pre_counter_1 = current_counter_1;
		
		
		return circle_perMin_1;
		
		
		


	}
	else if(motor->whoAmI==2){
		
		//获取当前速度并计算
		current_counter_2 = __HAL_TIM_GET_COUNTER(&htim4); // 获取当前计数值
		
		// 计算未经修正的delta_counter
		delta_counter_2 = (int32_t)current_counter_2 - (int32_t)pre_counter_2;

		// 检查是否发生了溢出（假设是16位计数器）
		if (delta_counter_2 > 32767) {
				// 发生了向下溢出
				delta_counter_2 -= 65536; // 调整delta_counter
		} else if (delta_counter_2 < -32768) {
				// 发生了向上溢出
				delta_counter_2 += 65536; // 调整delta_counter
		}
		
		
		delta_counter_2 = delta_counter_2*20*60;//50ms测一次，一分钟60s

		circle_perMin_2 = delta_counter_2 / 390.0 ;
		
		
		// 更新pre_counter为当前计数值
		pre_counter_2 = current_counter_2;
		
		
		return circle_perMin_2;

		
		
		

	
	
	
	
	}

	//理论上来讲永远不会运行到这里，但这确实是一个破绽，在后置函数中加判断不知为何无法正常运行，故未进行判断
	else return 0.0;


	


}


