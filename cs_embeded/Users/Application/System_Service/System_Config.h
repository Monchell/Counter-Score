/**
  ******************************************************************************
  * @file   : System_Config.h
  * @brief  : Header for System_Config.c
  ****************************************************************************** 
**/

 /* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "drv_timer.h"
#include "drv_imu.h"
#include "usart.h"
#include "usart.h"

#ifndef __MYSTSTEM__
#define __MYSTSTEM__

#ifdef __cplusplus
extern "C" {
#endif
	
typedef struct 
{
	GPIO_TypeDef* Gpio_Sda, *Gpio_Scl;
	uint16_t Pin_Sda,Pin_scl;
	float gyro_x,gyro_y,gyro_z;
	float roll,pitch,yaw;
}gyro_module;
/* Private define ------------------------------------------------------------*/
#define USART2_RX_LENGTH 100
#define USART4_RX_LENGTH 150
#define USART1_RX_LENGTH 100
/* Private variables ---------------------------------------------------------*/
extern gyro_module head;
extern gyro_module rhand;
extern gyro_module lhand;
	
/* Private function declarations --------------------------------------------*/
void board_config(void);
void mpu_init(gyro_module* aim_gyro);
 
	#ifdef __cplusplus
}
#endif
#endif
