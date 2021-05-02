///**
//  ******************************************************************************
//  * @file   : System_Gimbal.h
//  * @brief  : Header for System_Gimbal.c
//  ****************************************************************************** 
//**/

//#ifndef __GIMBAL_H__
//#define __GIMBAL_H__

///* Includes ------------------------------------------------------------------*/
//#include "can.h"
//#include "PID.h"

///* Private define ------------------------------------------------------------*/
//#define FLT_MAX 3.40282347e+38F
//#define YAWMAX	1687
//#define YAWMIN	-2313
//#define YAWMID	4746
//#define PITCHMAX	20000
//#define PITCHMID	16000
//#define PITCHMIN	15000
////#define ECD2ANG	360.0f/8192.0f
///* Private variables ---------------------------------------------------------*/
////ÈÎÎñ¾ä±ú
///* Private type --------------------------------------------------------------*/

//#ifdef __cplusplus	

//typedef struct 
//{
//	int PitchECD;
//	float PitchCurrent;
//	short YawECD;
//}MotorECD_Typedef;

//typedef struct  
//{
//	float YawRate;
//	float PitchRate;
//	float YawAngle;
//	float PitchAngle;
//}PitchYaw_Typedef;

//typedef struct 
//{
//	PitchYaw_Typedef Current;
//	PitchYaw_Typedef Target;
//}Gimbal_Typedef;

//class GimbalCtrl_Classdef
//{
//public:
//	void pid_config(void);
//	void set_rate(float gy,float gz);
//	void set_angle(float ay,float az);
//	void output_calculate(void);
//	void PIDControl(void);

//	MotorECD_Typedef MotorECD;
//	Gimbal_Typedef Gimbal;
//private:
//};

//extern GimbalCtrl_Classdef GimbalCtrl;
//extern myPID YawAngle,YawRate;
//extern myPID PitchAngle,PitchRate;

//extern "C"
//{
//	void Gimbal_PIDControl();
//}

//#endif
//#endif

///************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
