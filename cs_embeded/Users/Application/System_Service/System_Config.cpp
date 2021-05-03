/**
  ******************************************************************************
  * @file   System_config.c
  * @brief  Deploy resources and services in this file.
  ******************************************************************************
  * @note
  *  - Before running your Task you should first include your headers and init- 
  *    ialize used resources in "board_config()". This function will be 
  *    called before tasks Start.
  *    
  *
 */
 
 /* Includes ------------------------------------------------------------------*/
#include "System_Config.h"
#include "drv_iic.h"
#include "inv_mpu.h"
#include "core_cm3.h"
#include "stdio.h"


/* Private variables ---------------------------------------------------------*/

gyro_module head={GPIOA,GPIOA,GPIO_PIN_4,GPIO_PIN_3};
gyro_module rhand={GPIOA,GPIOA,GPIO_PIN_2,GPIO_PIN_1};
gyro_module lhand={GPIOB,GPIOB,GPIO_PIN_5,GPIO_PIN_4};

/*Founctions------------------------------------------------------------------*/
/**
  * @brief   initialize board bsp
  * @param   void
  * @retval  void
  */
  
void board_config(void)
{
	//时钟设置
	Timer_Init(&htim3,USE_MODULE_DELAY);
	//陀螺仪初始化
	mpu_init(&head);
	mpu_init(&rhand);
	mpu_init(&lhand);
	//串口初始化
	//Uart_Init(&huart1,usart1_rx_buff,USART1_RX_LENGTH,RecHandle);
	//Uart_Init(&huart2,usart2_rx_buff,USART2_RX_LENGTH,dr16handle);
	//Uart_Init(&huart4,usart4_rx_buff,USART4_RX_LENGTH,referee_Handle);

}

/**
  * @brief   初始化陀螺仪函数
  * @param   gyro_module
  * @retval  void
  */
void mpu_init(gyro_module* aim_gyro)
{
	SDA_GPIOx=aim_gyro->Gpio_Sda;
	SDA_PIN=aim_gyro->Pin_Sda;
	SCL_GPIOx=aim_gyro->Gpio_Scl;
	SCL_PIN=aim_gyro->Pin_scl;
	imu_config();
	if(mpu_dmp_init()!=0)
	{
		delay_ms_nos(100);
		__set_FAULTMASK(1);
		NVIC_SystemReset();
	}		
}


