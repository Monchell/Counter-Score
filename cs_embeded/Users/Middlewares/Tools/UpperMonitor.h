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
union type_change						//���ݴ��乲����ṹ
{
    uint8_t change_uint8_t[4];					// 8λ�޷������͡�1���ֽڡ�
    float change_float;					//32λ���������ݡ�4���ֽڡ�
    int	change_int;						//32λ�з������͡�4���ֽڡ�
    int16_t change_uint16_t;						//16λ�з������͡�2���ֽڡ�
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

