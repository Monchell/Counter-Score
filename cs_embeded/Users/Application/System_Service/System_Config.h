/**
  ******************************************************************************
  * @file   : System_Config.h
  * @brief  : Header for System_Config.c
  ****************************************************************************** 
**/

 /* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "tim.h"
#include "System_Shoot.h"

#include "newgimbal.h"
#include "uppermonitor.h"
#include "drv_timer.h"
#include "drv_can.h"
#include "drv_imu.h"
#include "drv_uart.h"
#include "usart.h"
#include "dr16.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "referee.h"

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
 uint32_t dr16handle(uint8_t *buf, uint16_t len);
 void can1_update(CAN_RxBuffer* can_data);
 void can2_update(CAN_RxBuffer* can_data);
 void board_config(void);
 
	#ifdef __cplusplus
}
#endif
#endif
