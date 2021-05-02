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
//#include "drv_uart.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart.h"

#ifndef __MYSTSTEM__
#define __MYSTSTEM__

#ifdef __cplusplus
extern "C" {
#endif
	
/* Private define ------------------------------------------------------------*/
#define USART2_RX_LENGTH 100
#define USART4_RX_LENGTH 150
#define USART1_RX_LENGTH 100
/* Private variables ---------------------------------------------------------*/
	
	
/* Private function declarations --------------------------------------------*/
 void board_config(void);
 
	#ifdef __cplusplus
}
#endif
#endif
