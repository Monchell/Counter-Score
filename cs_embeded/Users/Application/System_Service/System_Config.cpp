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
#include "GlobalValue.h"
#include "newgimbal.h"
#include "System_Referee.h"
#include "VSEC.h"


/* Private variables ---------------------------------------------------------*/
extern Motor_C610 turnplate_motor;
extern Motor_C620 FricR_motor,FricL_motor;
extern xQueueHandle dr16_data_handler;
extern xQueueHandle referee_data_handler;
extern TaskHandle_t Task_monitor_Handler;
extern referee_Classdef myreferee;

uint8_t usart2_rx_buff[USART2_RX_LENGTH];
uint8_t usart1_rx_buff[USART1_RX_LENGTH];
uint8_t usart4_rx_buff[USART4_RX_LENGTH];
DR16_Classdef dr_control;
uint8_t res;
/*Founctions------------------------------------------------------------------*/
/**
  * @brief   initialize board bsp
  * @param   void
  * @retval  void
  */
  
void board_config(void)
{
	//时钟设置
	Timer_Init(&htim4,USE_HAL_DELAY);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start(&htim1);
	//陀螺仪初始化
	//can初始化，依次为拨盘，左，右摩擦轮，pitch电机，YAW电机
	CAN_Init(&hcan1);
	CAN_Init(&hcan2);
	CAN_Rx_Callback_Register(&hcan1,can1_update);
	CAN_Rx_Callback_Register(&hcan2,can2_update);  
	CAN_Filter_Mask_Config(&hcan1,CanFilter_1|CanFifo_0|Can_STDID|Can_DataType,0x181,0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_2|CanFifo_0|Can_STDID|Can_DataType,0x202,0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_3|CanFifo_0|Can_STDID|Can_DataType,0x203,0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_4|CanFifo_0|Can_STDID|Can_DataType,0x200,0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_5|CanFifo_0|Can_STDID|Can_DataType,0x1ff,0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_6|CanFifo_0|Can_STDID|Can_DataType,0x201,0x3ff);
	CAN_Filter_Mask_Config(&hcan2,CanFilter_17|CanFifo_0|Can_STDID|Can_DataType,0x206,0x3ff);
	CAN_Filter_Mask_Config(&hcan2,CanFilter_16|CanFifo_0|Can_STDID|Can_DataType,0x205,0x3ff);
	
	//can回调设置
	
	//串口初始化
	Uart_Init(&huart1,usart1_rx_buff,USART1_RX_LENGTH,RecHandle);
	Uart_Init(&huart2,usart2_rx_buff,USART2_RX_LENGTH,dr16handle);
	Uart_Init(&huart4,usart4_rx_buff,USART4_RX_LENGTH,referee_Handle);
	//裁判系统
	myreferee.Init(&huart4,Get_SystemTimer);
	//云台，发射初始化
	Gimbal_Init();
	shoot_pid_config();	
	imu_config();
}

/**
  * @brief   裁判系统通信
  * @param   uint8_t,uint16_t
  * @retval  uint32_t
  */
uint32_t referee_Handle(uint8_t *buf, uint16_t len)
{
		if(referee_data_handler!=NULL)
		{
			myreferee.unPackDataFromRF(buf,len);
			return 0;
		}
		return 1;
}
/**
  * @brief   遥控控制获取
  * @param   uint8_t,uint16_t
  * @retval  uint32_t
  */
uint32_t dr16handle(uint8_t *buf, uint16_t len)
{
	static DR16_DataPack_Typedef dr_data;
	if(len == 18) {		
		BaseType_t xHigherpriorityTaskWoken;
		memcpy(&dr_data,buf,17);
		if(dr16_data_handler!=NULL)
		{
			xQueueSendFromISR(dr16_data_handler,&dr_data,&xHigherpriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherpriorityTaskWoken);
		}
		return 0;
	} else
	return 1;
}
/**
  * @brief  can2中断
  * @param  CAN_RxBuffer*
  * @retval void
  */

void can2_update(CAN_RxBuffer* can_data)
{
	//pitch电机
	if(can_data->header.StdId==0x205) 
	{
		static uint16_t ecd=0;
		ecd=(can_data->data[0]<<8|can_data->data[1]);	
		Pitch_motor.update(can_data->data);
		myGimbal.Pitch_joint->Rectangle_Angleupdate((float)ecd*ECD2ANG);
	}
	//Yaw电机
	if(can_data->header.StdId==0x206) 
	{
		static uint16_t ecd=0;
		ecd=(can_data->data[0]<<8|can_data->data[1]);	
		Yaw_motor.update(can_data->data);
		myGimbal.Yaw_joint->Rectangle_Angleupdate((float)ecd*ECD2ANG);
	}	

}

/**
  * @brief  can1中断
  * @param  CAN_RxBuffer*
  * @retval void
  */
void can1_update(CAN_RxBuffer* can_data)
{
	//陀螺仪数据
	if(can_data->header.StdId == 0x181)//0x181
	{
		float ay,az;
		memcpy(&az,&can_data->data[0],4);
		memcpy(&ay,&can_data->data[4],4);
		myGimbal.Pitch_joint->Nature_Angleupdate(ay);
		myGimbal.Yaw_joint->Nature_Angleupdate(-az);
	}
	//Roll的陀螺仪数据,如果有，没有就算了
	if(can_data->header.StdId == 0x182)
	{
		float ax;
		memcpy(&ax,&can_data->data[0],4);
		myGimbal.Pitch_joint->Nature_Angleupdate(-ax);		
	}
	//拨盘
	if(can_data->header.StdId == 0x201)
	{
		static int16_t ecd=0;
		static int64_t ecd_count=0;
		if(ecd>7000&&(can_data->data[0]<<8|can_data->data[1])<1000)
		{
			ecd_count++;
		}
		if(ecd<1000&&(can_data->data[0]<<8|can_data->data[1])>7000)
		{
			ecd_count--; 
		}
		ecd=(can_data->data[0]<<8|can_data->data[1]);
		turnplate_motor.update(can_data->data);
		Turnplate.Current=ecd+ecd_count*8191;
	}
	//发射1
	if(can_data->header.StdId==0x202)
	{
		//static int16_t ecd=0;
		//ecd=(can_data->data[0]<<8|can_data->data[1]);
		FricR_motor.update(can_data->data);
//		FricR_motor.MotorSpeed
		FricR.Current=FricR_motor.getSpeed();
	}
	//发射2
	if(can_data->header.StdId==0x203)
	{
		//static int16_t ecd=0;
		//ecd=(can_data->data[0]<<8|can_data->data[1]);
		FricL_motor.update(can_data->data);
		FricL.Current=FricL_motor.getSpeed();
	}
}



