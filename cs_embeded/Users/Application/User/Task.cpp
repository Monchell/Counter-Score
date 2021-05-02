///**
//  ******************************************************************************
//  * @file   Task.c
//  * @brief  Specific implementation of the task.
//  ******************************************************************************
//  * @note
//  ===============================================================================
//                                    Task List
//  ===============================================================================
//  * <table>
//  * <tr><th>Task Name          <th>Priority          <th>Frequency/Hz    <th>Stack/Byte
//  * <tr><td>task_gimbal        <td>1                 <td>1000            <td>512    
//  * <tr><td>task_dr16          <td>1                 <td>5000            <td>512  
//  * <tr><td>task_monitor       <td>1                 <td>100000          <td>512
//  * <tr><td>task_shoot         <td>1                 <td>1000            <td>512	
//  * <tr><td>shootCmd_task      <td>1                 <td>50000           <td>512  
//	* <tr><td>task_uppermonitor  <td>1                 <td>20000           <td>512
//  *...	
// */ 
// 
// /* Includes ------------------------------------------------------------------*/
//#include "Task_init.h"
//#include "filters.h"
//#include "dr16.h"
//#include "usart.h"
//#include "uppermonitor.h"
//#include "System_Config.h"
//#include "System_Shoot.h"
//#include "System_Referee.h"
//#include "System_Dr16.h"
//#include "Motor.h"
//#include "PID.h"
//#include "can.h"
//#include "tim.h"
//#include "drv_timer.h"
//#include "drv_can.h"
//#include "GlobalValue.h"

///* Private define ------------------------------------------------------------*/
//#define grid_ecd 32768.0f // 8192*36/9 9齿拨盘
///* Private variables ---------------------------------------------------------*/
//extern xQueueHandle dr16_data_handler;
//extern xQueueHandle referee_data_handler;
//extern DR16_Classdef dr_control;
///* Private function prototypes -----------------------------------------------*/
///**
//  * @brief
//  * @param
//  * @retval
//  */
//void task_gimbal(void * argument)
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xFrequency = 1;
//	xLastWakeTime = xTaskGetTickCount();

//	struct ahrs_sensor mpu_sensor;
//	for(;;) {
//		vTaskDelayUntil(&xLastWakeTime,xFrequency);
//		mpu_get_data(&mpu_sensor);
//		mpu_sensor.wx-=0.5f;
//		if(mpu_sensor.wx>=-0.5f&&mpu_sensor.wx<=0.5f)mpu_sensor.wx=0;
//		if(mpu_sensor.wx<-0.5f)mpu_sensor.wx+=0.5f;
//		if(mpu_sensor.wx>0.5f)mpu_sensor.wx-=0.5f;
//		myGimbal.Pitch_joint->Rate_update(mpu_sensor.wx);
//		myGimbal.Yaw_joint->Rate_update(-mpu_sensor.wz);
//		myGimbal.adjust();
//		Gimbal_Control();
//	}
//}

///**
//  * @brief
//  * @param
//  * @retval
//  */

//void task_dr16(void * argument)
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xFrequency = 5;
//	xLastWakeTime = xTaskGetTickCount();
//	for(;;) {
//		vTaskDelayUntil(&xLastWakeTime,xFrequency);		
//		DR16_DataPack_Typedef data;
//		if(xQueueReceive(dr16_data_handler,&data,10)==pdTRUE)
//		{
//			dr_control.DataCapture(&data);
//			DR16_Pack_Handle();
//			xTaskNotify(Task_monitor_Handler,BIT_0,eSetBits);//断线保护
//		}
//	}
//}

///**
//  * @brief
//  * @param
//  * @retval
//  */

//void task_monitor(void * argument)
//{
//    TickType_t xLastWakeTime;
//    const TickType_t xFrequency = 100;
//    xLastWakeTime = xTaskGetTickCount();
//    static uint32_t ulValue;
//    for(;;) 
//	{
//        vTaskDelayUntil(&xLastWakeTime,xFrequency);
//        xTaskNotifyWait (0xffffffff,0xffffffff,&ulValue, 0);
//        if((ulValue & BIT_0)!=0) {
//            dr16_online = 1;
//        } else {
//            dr16_online = 0;
//        }
//        if((ulValue & BIT_1)!=0) {
//            shoot.shoot_online = 1;
//        } else {
//            shoot.shoot_online = 0;
//        }
//    }
//}


///**
//  * @brief
//  * @param
//  * @retval
//  */
//void shoot_task(void * argument)
//{
//    TickType_t xLastWakeTime;
//    const TickType_t xFrequency = 1;
//    xLastWakeTime = xTaskGetTickCount();
//    shoot_pid_config();
//    for(;;) {
//        vTaskDelayUntil(&xLastWakeTime,xFrequency);
//        if(dr16_online != 1)
//		{
//			FricL.Target=0;
//			FricR.Target=0;
//			Turnplate.Target=Turnplate.Current;
//        }
//		shoot_output_calculate();
//    }
//}		


///**
//  * @brief
//  * @param
//  * @retval
//  */

//void shootCmd_task(void * argument)
//{
//    TickType_t xLastWakeTime;
//    const TickType_t xFrequency = 50;
//    xLastWakeTime = xTaskGetTickCount();

//    static int8_t backTimes = 0;
//    for(;;) {
//        vTaskDelayUntil(&xLastWakeTime,xFrequency);
//        if(shoot_block_check()) {
//            backTimes++;
//            if(backTimes >= 5) {
//                backTimes = 0;
//				Turnplate.Target=Turnplate.Current - grid_ecd;                
//                shoot_flagclear();
//            }
//        } else {
//            backTimes = 0;
//            if(shoot.shoot_once||shoot.shoot_constant) {
//                if(Turnplate.Current + grid_ecd * 2 > Turnplate.Target)
//					Turnplate.Target=Turnplate.Current + grid_ecd;                
//                if(shoot.shoot_once)
//                    shoot.shoot_once = 0;
//            }
//        }
//    }
//}
//		
///**
//  * @brief
//  * @param
//  * @retval
//  */
//void task_uppermonitor(void * argument)
//{
//    TickType_t xLastWakeTime;
//    const TickType_t xFrequency = 20;
//    xLastWakeTime = xTaskGetTickCount();
//    for(;;) {
//    vTaskDelayUntil(&xLastWakeTime,xFrequency);
//		Sent_Contorl(&huart1);
//		
//		
//    }
//}
///**
//  * @brief
//  * @param
//  * @retval
//  */
//void task_referee(void * argument)//好像用不到
//{
//    TickType_t xLastWakeTime;
//    const TickType_t xFrequency = 5;
//    xLastWakeTime = xTaskGetTickCount();
//	uint8_t buf[150];
//    for(;;) {    		
//		vTaskDelayUntil(&xLastWakeTime,xFrequency);		
//		if(xQueueReceive(referee_data_handler,buf,10)==pdTRUE)
//		{
//			myreferee.unPackDataFromRF(buf,150);
//		}
//    }
//}
