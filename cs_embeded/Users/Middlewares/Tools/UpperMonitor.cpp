/**
  ******************************************************************************
  * @file Tool_Host.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "UpperMonitor.h"
#include "drv_uart.h"
#include "PID.h"
#include "newgimbal.h"
#include "system_shoot.h"

/* Private define ------------------------------------------------------------*/
#define Sent_Data_Num 9

//#define DEBUG
#define TUNE_YAW 	0
#define TUNE_FRIC 	0
#define TUNE_PITCH	0
#define TUNE_HEAD 	1
/* Debug variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
union type_change Sent_data_type[Sent_Data_Num+2];		//传输数据共用体
uint8_t	USART0_Sent_Choose_Data[9]= {0,0,0,0,0,0,0,0,0};		//串口选择发送的数据标志
uint8_t  USART0_Interrupt_flag=0xff;							//串口中断标志位
uint8_t	USART0_Get_Num_Flag=0;								//串口数据获取标志
uint8_t  USART0_receive[5]= {0};								//串口接收缓存数组
/* Private function prototypes -----------------------------------------------*/
/************************************************************************************
*                         串口1发送设置函数 	   		  	                  			*
*          入口参数:	data:需要传输的数组指针									  		*
*          函数说明:用于设置DMA串口的数据											*
************************************************************************************/
void USART1_Sent_Set(float *data)
{
    uint8_t j;
    Sent_data_type[0].change_uint8_t[3]=0xfd;						//发送数据头
    for(j=1; j<Sent_Data_Num+1; j++) {							//数据体
        Sent_data_type[j].change_float=data[j-1];
    }
    Sent_data_type[Sent_Data_Num+1].change_uint8_t[0]=Sent_Data_Num;			//数据尾
    Sent_data_type[Sent_Data_Num+1].change_uint8_t[1]=0xfe;	//校验位
}

/************************************************************************************
*                         串口1发送参数选择函数 	   		  	                  		*
*          入口参数:	data:需要传输的数组指针									  		*
*          函数说明:	用于选择需要传输的数据											*
*		   返回值:	无																*
************************************************************************************/
void USART_Sent_Choose(float * data)
{
    uint8_t i;
    for(i=0; i<Sent_Data_Num; i++) {
        switch(USART0_Sent_Choose_Data[i]) {
        //输入捕获
		#if TUNE_PITCH
        /*********************************************************************************/
        case 0:
            data[i] = myGimbal.Pitch_joint->pid_Rate.Target;
            break;
        case 1:
			data[i] = myGimbal.Pitch_joint->pid_Rate.Current;         
            break;
        case 2:
            data[i] = myGimbal.Pitch_joint->pid_Rate.Out;
            break;
        case 3:
            data[i] = myGimbal.Pitch_joint->pid_Rate.Kp;
            break;
        case 4:
            data[i] = myGimbal.Pitch_joint->pid_Rate.Ki;
            break;
        case 5:
            data[i] = myGimbal.Pitch_joint->pid_Rate.Kd;
            break;
        case 6:
            data[i] = myGimbal.Pitch_joint->pid_Rate.P_Term;
            break;
       case 7:
            data[i] = myGimbal.Pitch_joint->pid_Rate.I_Term;
            break;
        case 8:
            data[i] = myGimbal.Pitch_joint->pid_Rate.D_Term;
            break;
        case 9:
            data[i] = myGimbal.Pitch_joint->pid_Angle.Target;
            break;
        case 10:
            data[i] = myGimbal.Pitch_joint->pid_Angle.Current;
            break;
        case 11:
            data[i] = myGimbal.Pitch_joint->pid_Angle.Out;
            break;
        case 12:
            data[i] = myGimbal.Pitch_joint->pid_Angle.Kp;
            break;
        case 13:
            data[i] = myGimbal.Pitch_joint->pid_Angle.Ki;
            break;
        case 14:
            data[i] = myGimbal.Pitch_joint->pid_Angle.Kd;
            break;    
		case 15:
            data[i] = Pitch_speed_mode;
            break;    
		#endif
		#if TUNE_YAW
        /*********************************************************************************/
        case 0:
            data[i] = myGimbal.Yaw_joint->pid_Rate.Target;
            break;
        case 1:
			data[i] = myGimbal.Yaw_joint->pid_Rate.Current;         
            break;
        case 2:
            data[i] = myGimbal.Yaw_joint->pid_Rate.Out;
            break;
        case 3:
            data[i] = myGimbal.Yaw_joint->pid_Rate.Kp;
            break;
        case 4:
            data[i] = myGimbal.Yaw_joint->pid_Rate.Ki;
            break;
        case 5:
            data[i] = myGimbal.Yaw_joint->pid_Rate.Kd;
            break;
        case 6:
            data[i] = myGimbal.Yaw_joint->pid_Rate.P_Term;
            break;
       case 7:
            data[i] = myGimbal.Yaw_joint->pid_Rate.I_Term;
            break;
        case 8:
            data[i] = myGimbal.Yaw_joint->pid_Rate.D_Term;
            break;
        case 9:
            data[i] = myGimbal.Yaw_joint->pid_Angle.Target;
            break;
        case 10:
            data[i] = myGimbal.Yaw_joint->pid_Angle.Current;
            break;
        case 11:
            data[i] = myGimbal.Yaw_joint->pid_Angle.Out;
            break;
        case 12:
            data[i] = myGimbal.Yaw_joint->pid_Angle.Kp;
            break;
        case 13:
            data[i] = myGimbal.Yaw_joint->pid_Angle.Ki;
            break;
        case 14:
            data[i] = myGimbal.Yaw_joint->pid_Angle.Kd;
            break;    
		case 15:
            data[i] = Yaw_speed_mode;
            break;    
		#endif
		#if TUNE_FRIC
		case 0:
            data[i] = FricL.Target;
            break;
        case 1:
            data[i] = FricL.Current;
            break;
        case 2:
            data[i] = FricL.Out;
            break;
        case 3:
            data[i] = FricL.Kp;
            break;
        case 4:
            data[i] = FricL.Ki;
            break;
        case 5:
            data[i] = FricL.Kd;
            break;
        case 6:
            data[i] = FricL.P_Term;
            break;
        case 7:
            data[i] = FricL.I_Term;
            break;
		case 8:
            data[i] = FricL.D_Term;
            break;
		#endif
		#if TUNE_HEAD
		case 0:
            data[i] = Turnplate.Target;
            break;
        case 1:
            data[i] = Turnplate.Current;
            break;
        case 2:
            data[i] = Turnplate.Out;
            break;
        case 3:
            data[i] = Turnplate.Kp;
            break;
        case 4:
            data[i] = Turnplate.Ki;
            break;
        case 5:
            data[i] = Turnplate.Kd;
            break;
        case 6:
            data[i] = Turnplate.P_Term;
            break;
        case 7:
            data[i] = Turnplate.I_Term;
            break;
		case 8:
            data[i] = Turnplate.D_Term;
            break;
		#endif
        default:
            break;
        }
    }
}
/************************************************************************************
*                         发送数据线程函数 	   		  	                  			*
*          入口参数:	无									  							*
*          函数说明:用于发送传输数据													*
************************************************************************************/
void Sent_Contorl(UART_HandleTypeDef* huart)
{
    float temp[Sent_Data_Num];
    USART_Sent_Choose(temp);								//选择要传输的数据
    USART1_Sent_Set(temp);									//发送数据转换格式
    HAL_UART_Transmit_DMA(huart,(uint8_t*)Sent_data_type+3,39);				//开始一次DMA传输！
}

uint32_t RecHandle(uint8_t *data_buf,uint16_t length)
{
    uint8_t Temp=0;
    for(int i=0; i<length; i++) {
        Temp=data_buf[i];
        switch(USART0_Interrupt_flag) {
        case 0xff:	//USART0_Interrupt_flag==0xff时为等待模式，等待指令头输入
            if(Temp==0xf0)						//指令头，识别上位机发送了修改指令
                USART0_Interrupt_flag=0xf0;		//下一个指令将进入模式选择模式
            break;
        case 0xf0:										//进入模式选择
            if(Temp==0x00) {					//修改参数
                USART0_Interrupt_flag=0x00;		//进入参数修改模式
                USART0_Get_Num_Flag=0;
            } else if(Temp==0x01) {				//修改模式
                USART0_Interrupt_flag=0x01;		//进入模式修改模式
                USART0_Get_Num_Flag=0;
            }
            break;
        case 0x00:
            USART0_receive[USART0_Get_Num_Flag]=Temp;//参数号是多少
            USART0_Get_Num_Flag++;
            if(USART0_Get_Num_Flag>4) {		//参数处理
                PARAMETER_MODIFICATION(USART0_receive);  //修改该参数
                USART0_Interrupt_flag=0xff;		//回到等待模式
            }
            break;
        case 0x01:
            USART0_receive[USART0_Get_Num_Flag]=Temp; //修改要读取的数据号，是一个浮点数
            USART0_Get_Num_Flag++;
            if(USART0_Get_Num_Flag>4) {		//参数处理
                MODE_MODIFICATION(USART0_receive);
                USART0_Interrupt_flag=0xff;		//回到等待模式
            }
            break;
        default:
            USART0_Interrupt_flag=0xff;		//回到等待模式
            break;
        }
    }
    return 0;
}

/************************************************************************************
*                              上位机参数修改函数                                 	*
*   	入口参数：	PARAMETER：	指令数组指针，用于读取指令							*
*		返回值	：	无																*
*   	函数说明：	上位机参数修改													*
************************************************************************************/
void PARAMETER_MODIFICATION(uint8_t * PARAMETER)
{
    switch(PARAMETER[0]) {
		#if TUNE_PITCH		
		case 0x01:
			myGimbal.Pitch_joint->pid_Rate.Target = PARAMETER_Change_float(PARAMETER+1);	
			break;
		case 0x02:
			myGimbal.Pitch_joint->pid_Rate.Target*=-1;
			break;
		case 0x03:    	  
			myGimbal.Pitch_joint->pid_Rate.Kp = PARAMETER_Change_float(PARAMETER+1);
			break;
		case 0x04:
			myGimbal.Pitch_joint->pid_Rate.Ki = PARAMETER_Change_float(PARAMETER+1);		  
			break;
		case 0x05:
			myGimbal.Pitch_joint->pid_Rate.Kd = PARAMETER_Change_float(PARAMETER+1);		 
			break;	
		case 0x06:        
			myGimbal.Pitch_joint->pid_Angle.Kp = PARAMETER_Change_float(PARAMETER+1);
			break;
		case 0x07:
			myGimbal.Pitch_joint->pid_Angle.Ki = PARAMETER_Change_float(PARAMETER+1);
			break;
		case 0x08:
			myGimbal.Pitch_joint->pid_Angle.Kd = PARAMETER_Change_float(PARAMETER+1);
			break;
		case 0x09:
			Pitch_speed_mode = PARAMETER_Change_float(PARAMETER+1);
			break;
			#endif
		#if TUNE_YAW
		case 0x01:
			myGimbal.Yaw_joint->pid_Rate.Target = PARAMETER_Change_float(PARAMETER+1);	
			break;
		case 0x02:
			myGimbal.Yaw_joint->pid_Rate.Target*=-1;
			break;
		case 0x03:    	  
			myGimbal.Yaw_joint->pid_Rate.Kp = PARAMETER_Change_float(PARAMETER+1);
			break;
		case 0x04:
			myGimbal.Yaw_joint->pid_Rate.Ki = PARAMETER_Change_float(PARAMETER+1);		  
			break;
		case 0x05:
			myGimbal.Yaw_joint->pid_Rate.Kd = PARAMETER_Change_float(PARAMETER+1);		 
			break;	
		case 0x06:        
			myGimbal.Yaw_joint->pid_Angle.Kp = PARAMETER_Change_float(PARAMETER+1);
			break;
		case 0x07:
			myGimbal.Yaw_joint->pid_Angle.Ki = PARAMETER_Change_float(PARAMETER+1);
			break;
		case 0x08:
			myGimbal.Yaw_joint->pid_Angle.Kd = PARAMETER_Change_float(PARAMETER+1);
			break;
		case 0x09:
			Yaw_speed_mode = PARAMETER_Change_float(PARAMETER+1);
			break;
		#endif
		#if TUNE_FRIC
		case 1:
			FricL.Target=PARAMETER_Change_float(PARAMETER+1);
			FricR.Target=PARAMETER_Change_float(PARAMETER+1);
			break;
		case 3:
			FricL.Kp=PARAMETER_Change_float(PARAMETER+1);           
			FricR.Kp=PARAMETER_Change_float(PARAMETER+1);           
			break;
		case 4:
			FricL.Ki=PARAMETER_Change_float(PARAMETER+1);           
			FricR.Ki=PARAMETER_Change_float(PARAMETER+1);          
			break;
		case 5:
			FricL.Kd=PARAMETER_Change_float(PARAMETER+1);           
			FricR.Kd=PARAMETER_Change_float(PARAMETER+1);     
			break;
		#endif
		#if TUNE_HEAD
		case 1:
			Turnplate.Target=PARAMETER_Change_float(PARAMETER+1);
			break;
		case 3:
			Turnplate.Kp=PARAMETER_Change_float(PARAMETER+1);           
			break;
		case 4:
			Turnplate.Ki=PARAMETER_Change_float(PARAMETER+1);            
			break;
		case 5:
			Turnplate.Kd=PARAMETER_Change_float(PARAMETER+1);           
			break;
		#endif
    default:
        break;
    }
}

/************************************************************************************
*                              上位机参数修改函数                                 	*
*   	入口参数：	PARAMETER：	指令数组指针，用于读取指令							*
*		返回值	：	无																*
*   	函数说明：	上位机参数修改													*
************************************************************************************/
void MODE_MODIFICATION(uint8_t * PARAMETER)
{
    switch(PARAMETER[0]) {
    case 0x00:
        USART0_Sent_Choose_Data[0]=PARAMETER_Change_float(PARAMETER+1);
        break;
    case 0x01:
        USART0_Sent_Choose_Data[1]=PARAMETER_Change_float(PARAMETER+1);
        break;
    case 0x02:
        USART0_Sent_Choose_Data[2]=PARAMETER_Change_float(PARAMETER+1);
        break;
    case 0x03:
        USART0_Sent_Choose_Data[3]=PARAMETER_Change_float(PARAMETER+1);
        break;
    case 0x04:
        USART0_Sent_Choose_Data[4]=PARAMETER_Change_float(PARAMETER+1);
        break;
    case 0x05:
        USART0_Sent_Choose_Data[5]=PARAMETER_Change_float(PARAMETER+1);
        break;
    case 0x06:
        USART0_Sent_Choose_Data[6]=PARAMETER_Change_float(PARAMETER+1);
        break;
    case 0x07:
        USART0_Sent_Choose_Data[7]=PARAMETER_Change_float(PARAMETER+1);
        break;
    case 0x08:
        USART0_Sent_Choose_Data[8]=PARAMETER_Change_float(PARAMETER+1);
        break;
    default:
        break;
    }
}
/************************************************************************************
*                              上位机参数转变成浮点数函数                            *
*   	入口参数：	PARAMETER：	指令数组指针，用于读取指令							*
*		返回值	：	转换后的浮点数													*
*   	函数说明：	上位机参数修改													*
************************************************************************************/
float PARAMETER_Change_float(uint8_t * PARAMETER)
{
    uint8_t i=0;
    union type_change Sent_data_temp;									//传输数据共用体
    for(i=0; i<4; i++) {
        Sent_data_temp.change_uint8_t[i]=PARAMETER[3-i];						//转换成共用体数据类型
    }
    return Sent_data_temp.change_float;									//返回共用体转化后的数据
}

/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/

