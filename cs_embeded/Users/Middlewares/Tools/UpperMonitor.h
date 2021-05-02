/**
  ******************************************************************************
  * @file Tool_Host.h
  * @author
  * @brief
  * @version 1.0
  * @date
  * @editby

  ==============================================================================
                     ##### How to use this conf #####
  ==============================================================================

  ******************************************************************************
  * @attention
  *
  * if you had modified this file, please make sure your code does not have many
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding
  * through your new brief.
  ******************************************************************************
  */
#ifndef _UPPERMONITOR_H_
#define _UPPERMONITOR_H_
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private type --------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
union type_change						//数据传输共用体结构
{
    uint8_t change_uint8_t[4];					// 8位无符号整型【1个字节】
    float change_float;					//32位浮点型数据【4个字节】
    int	change_int;						//32位有符号整型【4个字节】
    int16_t change_uint16_t;						//16位有符号整型【2个字节】
};
/* Private function prototypes -----------------------------------------------*/
void USART1_Sent_Set(float *data);
void USART_Sent_Choose(float * data);
float PARAMETER_Change_float(uint8_t * PARAMETER);
void PARAMETER_MODIFICATION(uint8_t * PARAMETER);
void MODE_MODIFICATION(uint8_t * PARAMETER);
uint32_t RecHandle(uint8_t *data_buf,uint16_t length);
void Sent_Contorl(UART_HandleTypeDef* huart);
	
#ifdef __cplusplus
}
#endif

#endif


/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/

