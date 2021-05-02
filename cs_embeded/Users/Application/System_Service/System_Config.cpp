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

/*Founctions------------------------------------------------------------------*/
/**
  * @brief   initialize board bsp
  * @param   void
  * @retval  void
  */
  
void board_config(void)
{
	//时钟设置
	Timer_Init(&htim3,USE_HAL_DELAY);
	//陀螺仪初始化
	
	//串口初始化
	//Uart_Init(&huart1,usart1_rx_buff,USART1_RX_LENGTH,RecHandle);
	//Uart_Init(&huart2,usart2_rx_buff,USART2_RX_LENGTH,dr16handle);
	//Uart_Init(&huart4,usart4_rx_buff,USART4_RX_LENGTH,referee_Handle);
	SDA_GPIOx=GPIOB;
	SDA_PIN=GPIO_PIN_1;
	SCL_GPIOx=GPIOB;
	SCL_PIN=GPIO_PIN_0;
	imu_config();
	if(mpu_dmp_init()!=0)
	{
		delay_ms_nos(100);
		__set_FAULTMASK(1);
		NVIC_SystemReset();
	}	
}

/**
  * @brief   裁判系统通信
  * @param   uint8_t,uint16_t
  * @retval  uint32_t
  */



