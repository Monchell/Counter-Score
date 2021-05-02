///**
//  ******************************************************************************
//  * @file   Task_init.c
//  * @brief  Initialize the tasks.
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
//#include "Task.h"
//#include "dr16.h"
//#include "System_Dr16.h"
//#include "UpperMonitor.h"

///* Private variables ---------------------------------------------------------*/
//TaskHandle_t Task_vision_Handler;
//TaskHandle_t Task_referee_Handler;	
//TaskHandle_t Task_carcommucation_Handler;
//TaskHandle_t Task_dr16_Handler;	
//TaskHandle_t Task_gyro_Handler;
//TaskHandle_t Task_shoot_Handler;
//TaskHandle_t Task_gimbal_Handler;
//TaskHandle_t Task_monitor_Handler;
//TaskHandle_t Task_shootcmd_Handler;
//TaskHandle_t Task_uppermonitor_Handler;
//xQueueHandle dr16_data_handler;
//xQueueHandle referee_data_handler;

///*Founctions------------------------------------------------------------------*/
///**
//  * @brief
//  * @param
//  * @retvalhtim
//  */
//void task_init(void){
//	
//	
//	dr16_data_handler=xQueueCreate(10, sizeof(DR16_DataPack_Typedef));
//	referee_data_handler=xQueueCreate(10, sizeof(uint8_t(150)));
//	
//	//云台任务
//	xTaskCreate((TaskFunction_t) task_gimbal, (const char*) "task_task1",
//    			(uint16_t) TASK_GIMBAL_SIZE, (void*) NULL,
//				(UBaseType_t) TASK_GIMBAL_PRIO,
//				(TaskHandle_t*) &Task_gimbal_Handler);		
//	//遥控器控制任务				
//	xTaskCreate((TaskFunction_t) task_dr16, (const char*) "task_task2",
//    			(uint16_t) TASK_DR16_SIZE, (void*) NULL,
//				(UBaseType_t) TASK_DR16_PRIO,
//				(TaskHandle_t*) &Task_dr16_Handler);		
//	//离线监测
//	xTaskCreate((TaskFunction_t) task_monitor, (const char*) "task_task3",
//    			(uint16_t) TASK_MONITOR_SIZE, (void*) NULL,
//				(UBaseType_t) TASK_MONITOR_PRIO,
//				(TaskHandle_t*) &Task_monitor_Handler);			
//	//发射任务			
//	xTaskCreate((TaskFunction_t) shootCmd_task, (const char*) "task_task5",
//    			(uint16_t) TASK_SHOOTCMD_SIZE, (void*) NULL,
//				(UBaseType_t) TASK_SHOOTCMD_PRIO,
//				(TaskHandle_t*) &Task_shootcmd_Handler);

//	xTaskCreate((TaskFunction_t) shoot_task, (const char*) "task_task9",
//    			(uint16_t) TASK_SHOOT_SIZE, (void*) NULL,
//				(UBaseType_t) TASK_SHOOT_PRIO,
//				(TaskHandle_t*) &Task_shoot_Handler);				
//	//上位机任务
//	#ifdef ISDEBUG
//	xTaskCreate((TaskFunction_t) task_uppermonitor, (const char*) "task_task6",
//    			(uint16_t) TASK_UPPERMONITOR_SIZE, (void*) NULL,
//				(UBaseType_t) TASK_UPPERMONITOR_PRIO,
//				(TaskHandle_t*) &Task_uppermonitor_Handler);
//	#endif
//}
