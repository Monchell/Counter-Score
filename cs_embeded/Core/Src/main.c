/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "System_Config.h"
#include "drv_timer.h"
#include "drv_imu.h"
#include "drv_iic.h"
#include "inv_mpu.h"
#include <stdio.h>
#include "string.h"
#include "mahony_ahrs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
extern "C"
{
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}
}
typedef struct 
{
	uint32_t adc_now;//使用0.5指数形滤波方式
	uint32_t adc_tmp[5];//使用0.5指数形滤波方式
	uint32_t adc_sum;//使用0.5指数形滤波方式
	uint8_t  point;
	float adc_out;
}adc_module;
void mpu_get(gyro_module *aim_gyro);
float adc_update(adc_module* aimadc,uint32_t adc_value);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_TURN HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13)
#define LED_OFF HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET)
#define LED_ON HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//时钟
#define gyro_count 30 
#define button_count 5
#define adc_count 1
#define send_count 20

uint32_t gyrotimer=0;
uint32_t sendtimer=0;
uint32_t buttontimer=0;
uint32_t adctimer=0;

//串口发送数据
uint8_t send_buff[50];

//按键情况
uint8_t keystate=0;
uint8_t key0,key1,key2,key3,key4;

//adc电压值
uint32_t adc_value=0;

adc_module aimadc;

float adc_out=0;//使用0.5指数形滤波方式

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
  
float tm1;
float tm2;
float tm3;
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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  LED_OFF;
  board_config();  
  gyrotimer=Get_SystemTimer();
  buttontimer=Get_SystemTimer();
  sendtimer=Get_SystemTimer();
  adctimer=Get_SystemTimer();
  LED_ON;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//HAL_UART_Transmit(&huart1,&data,1,0xff);
	//陀螺仪任务
	if(microsecond()>=gyrotimer)
	{
		gyrotimer=microsecond()+gyro_count*1000;
		mpu_get(&head);
		mpu_get(&rhand);
		mpu_get(&lhand);
	}
	//按键检测任务
	if(microsecond()>=buttontimer)
	{
		buttontimer=microsecond()+button_count*1000;
		key0=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4);
		key1=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)<<1;
		key2=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)<<2;
		key3=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)<<3;
		key4=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)<<4;	
		keystate=key0+key1+key2+key3+key4;
	}
	//蓝牙发送任务，5ms为任务周期
	if(microsecond()>=sendtimer)
	{
		sendtimer=microsecond()+send_count*1000;
		memcpy(&send_buff[0],&head.pitch,12); //头三轴俯仰，横滚，航向，float型各两个字节，下同
		memcpy(&send_buff[12],&rhand.pitch,12);//右手三轴
		memcpy(&send_buff[24],&lhand.pitch,12);//左手三轴
		
		memcpy(&send_buff[36],&keystate,1);//上拉按键状态，一个字节，按下对应位为0低电平，否则为1高电平，
		//第0位到第4位对应B4-B8，其中B8是开枪，456按顺序向下排列，7为狙击键（鼠标左键）
		memcpy(&send_buff[37],&adc_out,4);//弯曲传感器，float型两个字节
		send_buff[41]=222;//包尾为222		
		HAL_UART_Transmit_DMA(&huart1,send_buff,42);//一个包传输42个字节
		//printf("hello world\n");//可使用printf进行调试
		//HAL_UART_Transmit_DMA(&huart1,&keystate,1);//一个包传输42个字节
	}
	//adc读取任务
	if(microsecond()>=adctimer)
	{
		adctimer=microsecond()+adc_count*1000;
		HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);    //等待转换完成，第二个参数表示超时时间，单位ms        
        if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
        {
			adc_value=HAL_ADC_GetValue(&hadc1);                      
			adc_out=adc_update(&aimadc,adc_value);
        }                          						
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief   Get mpu
	* @param   float gyro_x,gyro_y,gyro_z,angle_x,angle_y,angle_z;
  * @retval  void
  */
void mpu_get(gyro_module* aim_gyro)
{
	SDA_GPIOx=aim_gyro->Gpio_Sda;
	SDA_PIN=aim_gyro->Pin_Sda;
	SCL_GPIOx=aim_gyro->Gpio_Scl;
	SCL_PIN=aim_gyro->Pin_scl;
    uint8_t buf[14];
    if(IIC_MPU6050_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf)==0)
        //缩短时间，不解算姿态的情况下只获取所需的陀螺仪GYO3轴数据
    {
//如果要加速度值需要换寄存器

        MPU6050.gx = (int16_t)(((buf[0]<<8) | buf[1])) ;//- MPU6050.gx_offset);
        MPU6050.gy = (int16_t)(((buf[2]<<8) | buf[3])) ;//- MPU6050.gy_offset);
        MPU6050.gz = (int16_t)(((buf[4]<<8) | buf[5])) ;//- MPU6050.gz_offset);


    }
    aim_gyro->gyro_x = MPU6050.gx / 32768.0f * 2000;//2000dps->rad/s
    aim_gyro->gyro_y = MPU6050.gy / 32768.0f * 2000;
    aim_gyro->gyro_z = MPU6050.gz / 32768.0f * 2000;
	mpu_dmp_get_data(&(aim_gyro->pitch),&(aim_gyro->roll),&(aim_gyro->yaw));
}

float adc_update(adc_module* aimadc,uint32_t adc_value)
{
	uint8_t point=aimadc->point;
	aimadc->adc_now=adc_value; //更新adc值
	aimadc->adc_sum = aimadc->adc_sum - aimadc->adc_tmp[point];//剔除历史末端
	aimadc->adc_tmp[point]=adc_value; //末端变成首部赋予新值
	aimadc->adc_sum = aimadc->adc_sum + aimadc->adc_tmp[point];//添加新值
	aimadc->adc_out =((float)aimadc->adc_sum/5)/4096*3.3;//为归一化.3.3化了;
	point++;
	if(point==5)point=0;
	aimadc->point=point;
	return aimadc->adc_out;
}
/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
