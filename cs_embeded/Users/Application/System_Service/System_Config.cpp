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
#include "adc.h"
#include "inv_mpu.h"
#include "core_cm3.h"
#include "drv_uart.h"
#include "stdio.h"


/* Private variables ---------------------------------------------------------*/
uint8_t usart1_rx_buff[USART1_RX_LENGTH];
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
	//adc初始化
	HAL_ADCEx_Calibration_Start(&hadc1);
	//串口初始化
	Uart_Init(&huart1,usart1_rx_buff,USART1_RX_LENGTH,HC06handle);

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

/**
  * @brief   电脑蓝牙接收
  * @param   uint8_t,uint16_t
  * @retval  uint32_t
  */
uint32_t HC06handle(uint8_t *buf, uint16_t len)
{
	if(len == 1&&buf[1]==1) {	
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		return 0;
	} else
	return 1;
}
