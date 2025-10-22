#include "PeropheralInit.h"
#include "MPU6050.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

extern MotorInfo motorDataRight;
extern MotorInfo motorDataLeft;
extern float btYawF2H;
extern float x_vector;
extern float y_vector;



//外设初始化封装
void peripheral_Initial(void){
	
	//陀螺仪初始化
	imu_Initial();
	HAL_Delay(2000);
	
	//PWM初始化
	PWM_Initial();
	HAL_Delay(200);
	
	//计时器-编码器6初始化，左右轮初始化
	Tim6Encoder();
	motorDataRight.whoAmI=1;
	motorDataLeft.whoAmI=2;
	
	
	//里程计初始化
	
	btYawF2H = 0.0f;
	x_vector = 0.0f;
	y_vector = 0.0f;
	
	
	
	HAL_Delay(200);

	
	
	






}



//陀螺仪初始化

extern int res;

void imu_Initial(void){
	
	
	res = MPU6050_DMP_Init();
	if(res !=0){
	
		while(res){
		
			HAL_Delay(1000);
			res = MPU6050_DMP_Init();

		
		}
	
	
	}else{
		
		//初始化失败处理
	
	
	}

}


//PWM初始化


void PWM_Initial(void){
	
			//通道1初始化
			HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);         
			HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);   

			//通道2初始化
			HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);           
			HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);        
			
	
	
			//设置初始占空比
			int pulse1=500;
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pulse1); 
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 500); 
	
			setPwmDutyCycle(&motorDataRight,pulse1);
			setPwmDutyCycle(&motorDataLeft,pulse1);

	
	
	
}

//定时器-编码器初始化

void Tim6Encoder(void){

			HAL_TIM_Base_Start_IT(&htim6);    
		
			HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	
			HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);


}







