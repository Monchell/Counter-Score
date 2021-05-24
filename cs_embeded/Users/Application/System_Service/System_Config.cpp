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
gyro_module head={GPIOA,GPIOA,GPIO_PIN_2,GPIO_PIN_1};
gyro_module rhand={GPIOA,GPIOA,GPIO_PIN_6,GPIO_PIN_5};
gyro_module lhand={GPIOA,GPIOA,GPIO_PIN_4,GPIO_PIN_3};



/*Founctions------------------------------------------------------------------*/
/**
  * @brief   initialize board bsp
  * @param   void
  * @retval  void
  */

void board_config(void)
{
	//ʱ������
	Timer_Init(&htim3,USE_MODULE_DELAY);
	//�����ǳ�ʼ��
	mpu_init(&head);
	mpu_init(&rhand);
	mpu_init(&lhand);
	//adc��ʼ��
	HAL_ADCEx_Calibration_Start(&hadc1);
	//���ڳ�ʼ��
	Uart_Init(&huart1,usart1_rx_buff,USART1_RX_LENGTH,HC06handle);

}

/**
  * @brief   ��ʼ�������Ǻ���
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
  * @brief   ������������
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
