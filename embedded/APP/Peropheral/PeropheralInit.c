#include "PeropheralInit.h"
#include "MPU6050.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

extern MotorInfo motorDataRight;
extern MotorInfo motorDataLeft;
extern float btYawF2H;
extern float x_vector;
extern float y_vector;



//�����ʼ����װ
void peripheral_Initial(void){
	
	//�����ǳ�ʼ��
	imu_Initial();
	HAL_Delay(2000);
	
	//PWM��ʼ��
	PWM_Initial();
	HAL_Delay(200);
	
	//��ʱ��-������6��ʼ���������ֳ�ʼ��
	Tim6Encoder();
	motorDataRight.whoAmI=1;
	motorDataLeft.whoAmI=2;
	
	
	//��̼Ƴ�ʼ��
	
	btYawF2H = 0.0f;
	x_vector = 0.0f;
	y_vector = 0.0f;
	
	
	
	HAL_Delay(200);

	
	
	






}



//�����ǳ�ʼ��

extern int res;

void imu_Initial(void){
	
	
	res = MPU6050_DMP_Init();
	if(res !=0){
	
		while(res){
		
			HAL_Delay(1000);
			res = MPU6050_DMP_Init();

		
		}
	
	
	}else{
		
		//��ʼ��ʧ�ܴ���
	
	
	}

}


//PWM��ʼ��


void PWM_Initial(void){
	
			//ͨ��1��ʼ��
			HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);         
			HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);   

			//ͨ��2��ʼ��
			HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);           
			HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);        
			
	
	
			//���ó�ʼռ�ձ�
			int pulse1=500;
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pulse1); 
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 500); 
	
			setPwmDutyCycle(&motorDataRight,pulse1);
			setPwmDutyCycle(&motorDataLeft,pulse1);

	
	
	
}

//��ʱ��-��������ʼ��

void Tim6Encoder(void){

			HAL_TIM_Base_Start_IT(&htim6);    
		
			HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	
			HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);


}







