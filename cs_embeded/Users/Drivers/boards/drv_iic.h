/**
  ******************************************************************************
  * @file bsp_iic.h
  * @author  Sweet
  * @brief 	 STM32F4软件gpio模拟iic


  * @version 1.0
  * @date	2019.06.23
  * @editby
  
	******************************************************************************
*/
#ifndef _DRV_IIC_H_
#define _DRV_IIC_H_
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stm32f1xx_hal.h>

/* Private define ------------------------------------------------------------*/
#define delay_us delay_us_nos
#define delay_ms delay_ms_nos
/* Private function prototypes -----------------------------------------------*/
extern GPIO_TypeDef *SDA_GPIOx;
extern uint16_t SDA_PIN;
extern GPIO_TypeDef *SCL_GPIOx;
extern uint16_t SCL_PIN;

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节

void IIC_SCL(const char x);
void IIC_SDA(const char x);
void SDA_IN(void);
void SDA_OUT(void);
GPIO_PinState READ_SDA(void);

#ifdef __cplusplus
}
#endif

#endif


/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/

