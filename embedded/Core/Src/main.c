/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MPU6050.h"
#include "stdio.h"
#include "PeropheralInit.h"
#include "Motor.h"
#include "math.h"
#include "AutoCar.h"
#include "string.h"
#include "BT.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int res;
float pitch;
float roll;
float yaw;
uint8_t timeflag;										


MotorInfo motorDataRight;
MotorInfo motorDataLeft;
AutoCarInfo autoCar;
imuInfo imu;
float_to_hex yaw_F2H;
float_to_hex x_F2H;
float_to_hex y_F2H;
float btYawF2H;
float yawOdometry;
float distance_vector;
float btSpeedRight;
float btSpeedLeft;
float x_vector;
float y_vector;
uint8_t sdBuffer[16];




uint8_t receiveData[128];
uint8_t receiveDataDMAbuffer[128];
uint8_t sendDataDMA[128];
uint8_t laserRowData_frame[88];
uint8_t laserRowData_buffer[84];
uint8_t laserTxInitial[9];
int signDMA = 1;
int signLaser = 0;


extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



//DMA发送中断响应函数

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	
	
	

	signDMA = 1;
	
	
}


double BT_yaw=0.0;
double BT_rmv=0.0;
double BT_lwv=0.0;




void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	
	
	
	if(huart==&huart3){
		
		for(int i=0;i<sizeof(receiveData);i++){
			sendDataDMA[i]=receiveData[i];
		}
		
		
//		if(signDMA == 1){
//			signDMA = 0;
//			//回发接收内容9
//			HAL_UART_Transmit_DMA(&huart3,sendDataDMA,Size);
//		}
		
		
		
		//解析接收数据
		 sscanf((const char *)receiveData, "%lf %lf %lf", &BT_yaw, &BT_rmv, &BT_lwv);
		
		
		//清空接收缓冲区
		for(int i=0;i<sizeof(receiveData);i++){
		
			receiveData[i]=0x00;
		
		}

		
		//重新打开DMA接收
			HAL_UARTEx_ReceiveToIdle_DMA(&huart3,receiveData,sizeof(receiveData));
		//关闭DMA半中断响应
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
	
	}
	
	
		//串口1接收中断
	else if(huart==&huart1){
		
		
		
		//assign the laser frame
		for(int i=0;i<Size;i++){
		
		laserRowData_frame[i+4]=laserRowData_buffer[i];
		
		
		}
		
		signLaser = 1;
		
		//重新打开串口1DMA接收
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,laserRowData_buffer,sizeof(laserRowData_buffer));
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  // 开启空闲中断
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);



	}
	
	
	
	
	
	
	
	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	//Wait for the Electricity
	HAL_Delay(200);
	
	
	//显示初始化发送缓冲区，以防止有奇怪的内容（会影响DMA发送）
		
		for(int i=0;i<sizeof(sendDataDMA);i++){
		
			sendDataDMA[i]=0x00;
		
		}
		
		for(int i=0;i<sizeof(receiveData);i++){
		
			receiveData[i]=0x00;
		
		}
		
		for(int i=0;i<sizeof(laserRowData_frame);i++){
			
			laserRowData_frame[i]=0x00;
		
		
		}
		
		for(int i=0;i<sizeof(laserRowData_buffer);i++){
		
			laserRowData_buffer[i]=0x00;			
		
		}
		
		
		laserRowData_frame[0]=0xFF;
		laserRowData_frame[1]=0xFF;
		laserRowData_frame[2]=0xFF;
		laserRowData_frame[3]=0x54;//负载长

		laserTxInitial[0] = 0xA5;  //帧头
		laserTxInitial[1] = 0x82;  //启动扫描命令
		laserTxInitial[2] = 05;
		laserTxInitial[3] = 0;
		laserTxInitial[4] = 0;
		laserTxInitial[5] = 0;
		laserTxInitial[6] = 0;
		laserTxInitial[7] = 0;
		laserTxInitial[8] = 0x22;

		
		
		HAL_UART_Transmit_DMA(&huart1,laserTxInitial,sizeof(laserTxInitial));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_tx, DMA_IT_HT);
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,laserRowData_buffer,sizeof(laserRowData_buffer));
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  // 开启空闲中断
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
		
		

		
	//打开DMA接收
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3,receiveData,sizeof(receiveData));
	//关闭DMA接收半中断响应
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
		
		
		
	
	
	// Initial peripheral
	peripheral_Initial();
	
	
	
		
	HAL_Delay(5000);
		
	
	// Initial PID
	PID_Angle_init(&autoCar);
	PID_init(&motorDataRight);
	PID_init(&motorDataLeft);
	


	
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		if(signDMA==1){
			// do NOT comment this loop, it will assign the imu
			if (MPU6050_DMP_Get_Data(&pitch, &roll, &yaw))
			{
				imu.ipitch=pitch;
				imu.iyaw=yaw;
				imu.iroll=roll;
				
			}
		}
		
		
		if(signDMA == 1&&signLaser==1){
			signDMA = 0;
			signLaser = 0;
			//send laser frame 4B+5B*8
			HAL_UART_Transmit_DMA(&huart3,laserRowData_frame,sizeof(laserRowData_frame));
			__HAL_DMA_DISABLE_IT(&hdma_usart3_tx, DMA_IT_HT);
			
		}
		
		
		
		
		//every 50ms do this loop------TIM6
		if(timeflag > 4)
		{
			
			

			//setYawTarget(&autoCar,BT_yaw , BT_rmv ,BT_lwv);  // 小车对象，预设yaw角，右轮参考速度，左轮参考速度
			//PID_Angle_loop(&autoCar);
			
			setTargetSpeed_perMin(&motorDataRight,BT_rmv);
			PID_loop(&motorDataRight);
			setTargetSpeed_perMin(&motorDataLeft,BT_lwv);
			PID_loop(&motorDataLeft);
			
			
			// The assigned of the btSpeedRight and btSpeedLeft is in the function setYawCurrent
			
//			float avgTwoWheelSpeed =((float) btSpeedRight+(float)btSpeedLeft)/2.0f;
//			
//			distance_vector = avgTwoWheelSpeed*50.0f*6.5f*3.1415926f /(60.0f*1000.0f*1.0f);
//			
//			x_vector = x_vector + distance_vector * (-sin(yawOdometry));
//				
//			y_vector = y_vector + distance_vector * cos(yawOdometry);
		
			
			
//			if(signDMA == 1){
//								
//				// The assigned of the btYawF2H is in the function setYawCurrent
//				set_float_value(&yaw_F2H,btYawF2H);
//				
//				sdBuffer[0]=0xFF; //帧头
//				sdBuffer[1]=0xFF; //帧头
//				sdBuffer[2]=0xFF;	//帧头
//				sdBuffer[3]=0x0C; //负载长
//				sdBuffer[4]=yaw_F2H.BT_float[0];
//				sdBuffer[5]=yaw_F2H.BT_float[1];
//				sdBuffer[6]=yaw_F2H.BT_float[2];
//				sdBuffer[7]=yaw_F2H.BT_float[3];
//				
//				
//				
//				set_float_value(&x_F2H,x_vector);
//				
//				set_float_value(&y_F2H,y_vector);
//				
//				sdBuffer[8]=x_F2H.BT_float[0];
//				sdBuffer[9]=x_F2H.BT_float[1];
//				sdBuffer[10]=x_F2H.BT_float[2];
//				sdBuffer[11]=x_F2H.BT_float[3];
//				
//				sdBuffer[12]=y_F2H.BT_float[0];
//				sdBuffer[13]=y_F2H.BT_float[1];
//				sdBuffer[14]=y_F2H.BT_float[2];
//				sdBuffer[15]=y_F2H.BT_float[3];
//				
//				

//				
//				signDMA = 0;

//				HAL_UART_Transmit_DMA(&huart3, sdBuffer, 16);
//					//关闭DMA接收半中断响应
//				__HAL_DMA_DISABLE_IT(&hdma_usart3_tx,DMA_IT_HT);
//					
//				
//			}
			
			
			
			
			
			//EMERGENCY STOP
			
//			if(fabs(motorDataRight.motorSpeed)>250.0||fabs(motorDataLeft.motorSpeed)>250.0){

//							
//						__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 500); 
//						__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 500); 
//							
//				while(1){
//					
//							if(signDMA == 1){
//								uint8_t sdBuffer[4];
//								
//								for(int i=0;i<sizeof(sdBuffer);i++){
//									
//									sdBuffer[i] = 0xFF;
//								}

//								signDMA = 0;

//								HAL_UART_Transmit_DMA(&huart3, sdBuffer, sizeof(sdBuffer));
//						}
//				
//				}

//			}
			
			
		  timeflag = 0x00;
			
		}
		
		
			
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int fputc(int ch,FILE *f){
	
	
	
	
	HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1,1000);

	
	return ch;




}


//���ڱ������
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   
{
   if (htim == (&htim6)) 
	 {
	   __HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);
		
		timeflag++;

	 }
 }



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
