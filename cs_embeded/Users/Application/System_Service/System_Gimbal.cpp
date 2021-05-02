///**
//  ******************************************************************************
//  * @file   System_Gimbal.c
//  * @brief  
//  ******************************************************************************
//  * @note
//  *
// */
// 
// /* Includes ------------------------------------------------------------------*/
//#include "System_Gimbal.h"
//#include "filter.h"
//#include "System_Shoot.h"
//#include "VSEC.h"
//#include "drv_timer.h"
//#include "GlobalValue.h"

//GimbalCtrl_Classdef GimbalCtrl;
//myPID YawAngle,YawRate;
//myPID PitchAngle,PitchRate;
//Motor_GM6020 Yaw_motor(1);
//extern Motor_C610 turnplate_motor;

//void GimbalCtrl_Classdef::pid_config(void)
//{
//	YawAngle.SetPIDParam(16.f,0,0,FLT_MAX,2000);
//	YawAngle.I_SeparThresh = 400;
//	YawAngle.Target = 0;
//	YawAngle.Current = 0;
//	YawAngle.getMicroTick_regist(Get_SystemTimer);
//	YawRate.SetPIDParam(100.f,2.0f,0,2000,29000.0f);
//	YawRate.I_SeparThresh = 400;
//	YawRate.Target = 0;
//	YawRate.Current = 0;
//	YawRate.getMicroTick_regist(Get_SystemTimer);
//	PitchAngle.SetPIDParam(10.0f,0,0,FLT_MAX,2000);
//	PitchAngle.I_SeparThresh = 400;
//	PitchAngle.Target = 0;
//	PitchAngle.Current = 0;
//	PitchAngle.getMicroTick_regist(Get_SystemTimer);
//	PitchRate.SetPIDParam(100.f,4.5,0,5000,30000);
//	PitchRate.I_SeparThresh = 400;
//	PitchRate.Target = 0;
//	PitchRate.Current = 0;
//	PitchRate.getMicroTick_regist(Get_SystemTimer);
//	
//}

//void GimbalCtrl_Classdef::set_rate(float gy,float gz)
//{
//	Gimbal.Current.PitchRate = -gy;
//	Gimbal.Current.YawRate = gz;
//}

//void GimbalCtrl_Classdef::set_angle(float ay,float az)
//{
//	Gimbal.Current.PitchAngle = ay;
//	Gimbal.Current.YawAngle = -az;
//}

//void GimbalCtrl_Classdef::output_calculate(void)
//{
//	YawAngle.Target=Gimbal.Target.YawAngle;
//	YawAngle.Current=Gimbal.Current.YawAngle;
//	YawAngle.Adjust();
//	YawRate.Target=YawAngle.Out;
//	YawRate.Current=Gimbal.Current.YawRate;
//	YawRate.Adjust();
//	
//	PitchAngle.Target=Gimbal.Target.PitchAngle;
//	PitchAngle.Current=Gimbal.Current.PitchAngle;
//	PitchAngle.Adjust();
//	PitchRate.Target=PitchAngle.Out;
//	PitchRate.Current=Gimbal.Current.PitchRate;
//	PitchRate.Adjust();
//	
//}

//void GimbalCtrl_Classdef::PIDControl(void)
//{
//	if(dr16_online == 1) {
//		Yaw_motor.Out = YawRate.Out;
//		turnplate_motor.Out=Turnplate.Out;
//		Motor_GM6020 gimbal_motor[1] = {Yaw_motor};
//		MotorMsgSend(&hcan1,gimbal_motor);//nowusing
//		MotorMsgSend(&hcan1,turnplate_motor);
//		/* 注意发送值单位为A 这里需要除1000 */
//		/* 千万注意！！否则会烧电调电机 */
//		VSEC_Set_Current(&hcan2,1,PitchRate.Out/1000.0f);
//	} 
//	else {
//		/* 目标值置为当前值 */
//		Gimbal.Target.YawAngle = Gimbal.Current.YawAngle;
//		Gimbal.Target.PitchAngle = Gimbal.Current.PitchAngle;
//		/* Yaw轴输出置零 */
//		Yaw_motor.Out = 0;
//		Motor_GM6020 gimbal_motor[1]={Yaw_motor};
//		MotorMsgSend(&hcan1,gimbal_motor);
//		/* Pitch轴输出置零 */
//		PitchRate.Out = 0;
//		VSEC_Set_Current(&hcan2,1,PitchRate.Out);
//	}
//}

//void Gimbal_PIDControl()
//{
//	GimbalCtrl.PIDControl();
//}
