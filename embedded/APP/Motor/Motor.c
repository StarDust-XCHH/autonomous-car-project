#include "Motor.h"
#include <stdio.h>
#include "tim.h"
#include "math.h"

extern float btSpeedRight ;
extern float btSpeedLeft ;

//--------------set����--------------
// ����PWMʵ��ռ�ձ�
void setPwmDutyCycle(MotorInfo* motor, int pwmDutyCycle){
    if (motor != NULL) {
				//��ֵʵ��ռ�ձ�
        motor->pwmDutyCycle = pwmDutyCycle;
			
				//��ֵӳ��ռ�ձ�
				motor->pwmMirror = pwmDutyCycle - 500 ;
			
			
				if(motor->whoAmI==1){//�������Ϊ���1���ж��߼�
				// ����htim8��TIM_CHANNEL_1�Ѿ��������ط���ȷ���岢��ʼ��
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmDutyCycle);
				}	
				else if(motor->whoAmI==2){
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmDutyCycle);
				}
			
    }
}

// ����ӳ��PWM
void setPwmMirror(MotorInfo* motor, int pwmMirror){
    if (motor != NULL) {
			
				//��ֵӳ��ռ�ձ�
        motor->pwmMirror = pwmMirror;
			
				//��ֵʵ��ռ�ձ�
				motor->pwmDutyCycle = pwmMirror+500;
			
			
				if(motor->whoAmI==1){//�������Ϊ���1���ж��߼�
				// ����htim8��TIM_CHANNEL_1�Ѿ��������ط���ȷ���岢��ʼ��
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmMirror+500);
				}
				else if(motor->whoAmI==2){
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmMirror+500);
				}
			

    }
}

//���õ��ʵ��ת��
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

// ���õ��Ŀ��ת��
void setTargetSpeed_perMin(MotorInfo* motor, double targetSpeed){
    if (motor != NULL) {
			//����ÿ����300װ
				if(targetSpeed>300){motor->targetSpeed_perMin=300;}
				else if(targetSpeed<-300){motor->targetSpeed_perMin=-300;}
				else{
				motor->targetSpeed_perMin = targetSpeed;
				}    
		}
}

// �����ٶ����
void setSpeedError(MotorInfo* motor, double error){
    if (motor != NULL) {
        motor->SpeedError = error;
    }
}

// �����ٶ�������
void setSpeedErrorIteger(MotorInfo* motor, double errorInteger){
    if (motor != NULL) {
        motor->SpeedErrorIteger = errorInteger;
    }
}

// �����ٶ����΢��
void setSpeedErrorDerivate(MotorInfo* motor, double errorDerivate){
    if (motor != NULL) {
        motor->SpeedErrorDerivate = errorDerivate;
    }
}

// ������һ�ε��ٶ����
void setSpeedErrorLast(MotorInfo* motor, double errorLast){
    if (motor != NULL) {
        motor->SpeedErrorLast = errorLast;
    }
}

// ���ñ���ϵ��kp
void setKp(MotorInfo* motor, double kp){
    if (motor != NULL) {
        motor->kp = kp;
    }
}

// ���û���ϵ��ki
void setKi(MotorInfo* motor, double ki){
    if (motor != NULL) {
        motor->ki = ki;
    }
}

// ����΢��ϵ��kd
void setKd(MotorInfo* motor, double kd){
    if (motor != NULL) {
        motor->kd = kd;
    }
}


// ����PID����
void setMotorInfoPID_para(MotorInfo* motor, double kp,double ki,double kd) {
    if (motor != NULL) {
        setKp(motor,kp);
        setKi(motor,ki);
			  setKd(motor,kd);

    }
}

// ���������
void setError_para(MotorInfo* motor, double error, double errorInteger, double errorDerivate, double errorLast){

    if (motor != NULL) {
			
			
        setSpeedError(motor,error);
        setSpeedErrorIteger(motor,errorInteger);
			  setSpeedErrorDerivate(motor,errorDerivate);
				setSpeedErrorLast(motor,errorLast);

    }


}

//--------------PID��ʼ��--------------

// PID��ʼ��
void PID_init(MotorInfo* motor){
	


	//����PID����
	
	double kp=2.0,ki=1.0,kd=0.0;
	setMotorInfoPID_para(motor,kp,ki,kd);
	

	
	//����Ŀ���ٶ�
	
	double targetSpeed = 0.0;
	setTargetSpeed_perMin(motor,targetSpeed);
	
	
	
	//��ʼ�������
	
	double SpeedError=0.0;
	double SpeedErrorIteger=0.0;
	double SpeedErrorDerivate=0.0;
	double SpeedErrorLast=0.0;
	
	setError_para(motor,SpeedError,SpeedErrorIteger,SpeedErrorDerivate,SpeedErrorLast);

}

//--------------PID�ٶȻ�--------------




void PID_loop(MotorInfo* motor){	
	

	setMotorInfoSpeed(motor,encoderSpeedCalculate(motor));

	//�������
	
	setSpeedError(motor,(motor->targetSpeed_perMin)-(motor->motorSpeed));
	
	setSpeedErrorIteger(motor,motor->SpeedErrorIteger+(motor->SpeedError)*0.05);//��������50ms
	
	

	
	setSpeedErrorDerivate(motor,(motor->SpeedError-(motor->SpeedErrorLast))/0.05);//΢������50ms

	
	motor->PID_output = motor->kp*motor->SpeedError
										+motor->ki*motor->SpeedErrorIteger
										+motor->kd*motor->SpeedErrorDerivate;
	
	motor->PWM_output =(int) (motor->PID_output);
	
	if((motor->PWM_output)>500){motor->PWM_output=500;}
	if((motor->PWM_output)<-500){motor->PWM_output=-500;}

	
	//ʹ��ӳ�丳ֵ
	setPwmMirror(motor,motor->PWM_output);
	
	
	
	
	//��ӡ�͸�������
	
//	if(motor->whoAmI==1){
//		printf("m1: %f,%f,%d,%f\n", motor->motorSpeed,motor->targetSpeed_perMin,motor->pwmDutyCycle,motor->SpeedErrorIteger);
//	}
//	else if(motor->whoAmI==2){
//		printf("m2: %f,%f,%d,%f\n", motor->motorSpeed,motor->targetSpeed_perMin,motor->pwmDutyCycle,motor->SpeedErrorIteger);
//	}


	setSpeedErrorLast(motor,motor->SpeedError);

}

//--------------�������ٶȼ��㹤��--------------

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
	
	
			//��ȡ��ǰ�ٶȲ�����
		current_counter_1 = __HAL_TIM_GET_COUNTER(&htim3); // ��ȡ��ǰ����ֵ
		
		// ����δ��������delta_counter
		delta_counter_1 = (int32_t)current_counter_1 - (int32_t)pre_counter_1;

		// ����Ƿ����������������16λ��������
		if (delta_counter_1 > 32767) {
				// �������������
				delta_counter_1 -= 65536; // ����delta_counter
		} else if (delta_counter_1 < -32768) {
				// �������������
				delta_counter_1 += 65536; // ����delta_counter
		}
		
		
		delta_counter_1 = delta_counter_1*20*60;//50ms��һ�Σ�һ����60s��ÿ����ת��

		circle_perMin_1 = delta_counter_1 / 390.0 ;
		
		// ����pre_counterΪ��ǰ����ֵ
		pre_counter_1 = current_counter_1;
		
		
		return circle_perMin_1;
		
		
		


	}
	else if(motor->whoAmI==2){
		
		//��ȡ��ǰ�ٶȲ�����
		current_counter_2 = __HAL_TIM_GET_COUNTER(&htim4); // ��ȡ��ǰ����ֵ
		
		// ����δ��������delta_counter
		delta_counter_2 = (int32_t)current_counter_2 - (int32_t)pre_counter_2;

		// ����Ƿ����������������16λ��������
		if (delta_counter_2 > 32767) {
				// �������������
				delta_counter_2 -= 65536; // ����delta_counter
		} else if (delta_counter_2 < -32768) {
				// �������������
				delta_counter_2 += 65536; // ����delta_counter
		}
		
		
		delta_counter_2 = delta_counter_2*20*60;//50ms��һ�Σ�һ����60s

		circle_perMin_2 = delta_counter_2 / 390.0 ;
		
		
		// ����pre_counterΪ��ǰ����ֵ
		pre_counter_2 = current_counter_2;
		
		
		return circle_perMin_2;

		
		
		

	
	
	
	
	}

	//������������Զ�������е��������ȷʵ��һ���������ں��ú����м��жϲ�֪Ϊ���޷��������У���δ�����ж�
	else return 0.0;


	


}


