/**
******************************************************************************
* @file 		drv_imu.c
* @author 	charlie 602894526@qq.com
* @brief  	MPU6050相关的一些函数
* @version 	2.0
* @date
* @editby 	charlie

==============================================================================
										##### How to use this conf #####
==============================================================================
1.0:
IIC_MPU6050_Write_Byte  		对IIC协议下的MPU6050陀螺仪写数据
IIC_MPU6050_Read_Byte				对IIC协议下的MPU6050陀螺仪读数据
IIC_MPU6050_Set_Gyro_Fsr		对IIC协议下的MPU6050陀螺仪设置角度值的满量程范围
IIC_MPU6050_Set_Accel_Fsr		对IIC协议下的MPU6050陀螺仪设置加速度值的满量程范围
IIC_MPU6050_Set_LPF					对IIC协议下的MPU6050陀螺仪设置数字低通滤波器
IIC_MPU6050_Set_Rate				对IIC协议下的MPU6050陀螺仪设置采样率
IIC_MPU6050_Init						IIC协议下的MPU6050陀螺仪初始化及初始化状态反馈
MPU6050_Cal									测量100次陀螺仪数据取平均作为OFFSET
imu_config					IIC协议下的MPU6050陀螺仪初始化
IIC_MPU6050_Get_Accelerometer		得到IIC协议下的MPU6050陀螺仪加速度值（原始值）
IIC_MPU6050_Get_Gyroscope		得到IIC协议下的MPU6050陀螺仪值（原始值）
IIC_MPU6050_Read_Len				IIC协议下的MPU6050连续读

2.0:
Add gyroscope data read interface
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
#include "drv_imu.h"
#include "drv_iic.h"
#include "drv_timer.h"

/* Private define ------------------------------------------------------------*/
#define CAL_TIME 500;
uint32_t tic;

/* Private variables ---------------------------------------------------------*/
MPU6050_t MPU6050;        //MPU6050读取数据

/* Private type --------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
* @brief  对IIC协议下的MPU6050陀螺仪写数据
* @param  reg  寄存器地址
* @param  data 写入的数据内容
* @retval 0  写数据成功   1  正在写数据
*/
uint8_t IIC_MPU6050_Write_Byte(uint8_t reg,uint8_t data)
{
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令
    if(IIC_Wait_Ack()) {	//等待应答
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Send_Byte(data);//发送数据
    if(IIC_Wait_Ack()) {	//等待ACK
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}

/**
* @brief  对IIC协议下的MPU6050陀螺仪读数据
* @param  reg  寄存器地址
* @retval 读取到的对应寄存器地址的数据内容
*/
uint8_t IIC_MPU6050_Read_Byte(uint8_t reg)
{
    uint8_t res;
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令
    IIC_Wait_Ack();		//等待应答
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令
    IIC_Wait_Ack();		//等待应答
    res=IIC_Read_Byte(0);//读取数据,发送nACK
    IIC_Stop();			//产生一个停止条件
    return res;
}

/**
* @brief  对IIC协议下的MPU6050陀螺仪设置角度值的满量程范围
* @param  fsr  陀螺仪角度值最大量程
* @retval 0  设置成功  1  正在设置
*/
uint8_t IIC_MPU6050_Set_Gyro_Fsr(uint8_t fsr)
{
    return IIC_MPU6050_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围
}

/**
* @brief  对IIC协议下的MPU6050陀螺仪设置加速度值的满量程范围
* @param  fsr  陀螺仪加速度值最大量程
* @retval 0  设置成功  1  正在设置
*/
uint8_t IIC_MPU6050_Set_Accel_Fsr(uint8_t fsr)
{
    return IIC_MPU6050_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围
}

/**
* @brief  对IIC协议下的MPU6050陀螺仪设置数字低通滤波器
* @param  lpf  数字低通滤波频率(Hz)
* @retval 0  设置成功  1  正在设置
*/
uint8_t IIC_MPU6050_Set_LPF(uint16_t lpf)
{
    uint8_t data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    return IIC_MPU6050_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器
}

/**
* @brief  对IIC协议下的MPU6050陀螺仪设置采样率(假定Fs=1KHz)
* @param  rate  采样率 4~(Hz)
* @retval 0  设置成功  1  正在设置
*/
uint8_t IIC_MPU6050_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=IIC_MPU6050_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
    return IIC_MPU6050_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

/**
* @brief  IIC协议下的MPU6050陀螺仪初始化及初始化状态反馈
* @param  void
* @retval 0  初始化成功  1  正在初始化
*/
uint8_t IIC_MPU6050_Init(void)
{
    uint8_t res;
    IIC_Init();//初始化IIC总线
    IIC_MPU6050_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050 
	delay_ms_nos(100);
	IIC_MPU6050_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050
    IIC_MPU6050_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
    IIC_MPU6050_Set_Accel_Fsr(80);					//加速度传感器,±2g
    IIC_MPU6050_Set_Rate(1000);						//设置采样率50Hz

    IIC_MPU6050_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
    IIC_MPU6050_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
    IIC_MPU6050_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
    IIC_MPU6050_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
    res=IIC_MPU6050_Read_Byte(MPU_DEVICE_ID_REG);

    if(res==MPU_ADDR) { //器件ID正确
        IIC_MPU6050_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
        IIC_MPU6050_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
        IIC_MPU6050_Set_Rate(1000);						//设置采样率为50Hz
    } else
        return 1;
    return 0;
}

/**
* @brief  测量100次陀螺仪数据取平均作为OFFSET
* @param  void
* @retval void
*/
void MPU6050_Cal()
{
    uint8_t buf[14];
    uint16_t cal_time = CAL_TIME;
    uint16_t i = 0;
    for(i=0; i<cal_time; i++) {
        IIC_MPU6050_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);

        MPU6050.ax_offset += ((int16_t)((buf[0] << 8) | buf[1]));
        MPU6050.ay_offset += ((int16_t)((buf[2] << 8) | buf[3]));
        MPU6050.az_offset += ((int16_t)(((buf[4] << 8) | buf[5]) - 4096));
        delay_ms(2);

        IIC_MPU6050_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
        MPU6050.gx_offset += ((int16_t)((buf[8] << 8) | buf[9]));
        MPU6050.gy_offset += ((int16_t)((buf[10] << 8) | buf[11]));
        MPU6050.gz_offset += ((int16_t)((buf[12] << 8) | buf[13]));
        delay_ms(2);
		tic++;
    }
    MPU6050.ax_offset = MPU6050.ax_offset / cal_time;
    MPU6050.ay_offset = MPU6050.ay_offset / cal_time;
    MPU6050.az_offset = MPU6050.az_offset / cal_time;
    MPU6050.gx_offset = MPU6050.gx_offset / cal_time;
    MPU6050.gy_offset = MPU6050.gy_offset / cal_time;
    MPU6050.gz_offset = MPU6050.gz_offset / cal_time;
}

/**
* @brief  IIC协议下的MPU6050陀螺仪初始化
* @param  void
* @retval 0  初始化成功  1  正在初始化
*/
void imu_config(void)
{
    while(IIC_MPU6050_Init()) {
        //        u1_printf("MPU Init Error\r\n");
    }
    //MPU6050_Cal();			//算出零偏
}

/**
* @brief  得到IIC协议下的MPU6050陀螺仪加速度值（原始值）
* @param  ax,ay,az  陀螺仪x,y,z轴的原始加速度读数(带符号)
* @retval 0  正常  其他  错误数据
*/
uint8_t IIC_MPU6050_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;
    res=IIC_MPU6050_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
    if(res==0) {
        *ax=((uint16_t)buf[0]<<8)|buf[1];
        *ay=((uint16_t)buf[2]<<8)|buf[3];
        *az=((uint16_t)buf[4]<<8)|buf[5];
    }
    return res;
}

/**
* @brief  得到IIC协议下的MPU6050陀螺仪值（原始值）
* @param  gx,gy,gz  陀螺仪x,y,z轴的原始读数(带符号)
* @retval 0  正常  其他  错误数据
*/
uint8_t IIC_MPU6050_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res;

    res=IIC_MPU6050_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
    if(res==0) {
        *gx=(short)(((uint16_t)buf[0]<<8)|buf[1]);
        *gy=(short)(((uint16_t)buf[2]<<8)|buf[3]);
        *gz=(short)(((uint16_t)buf[4]<<8)|buf[5]);
    }
    return res;;
}

/**
* @brief  IIC协议下的MPU6050连续读
* @param  addr  器件地址
* @param  reg   要读取的寄存器地址
* @param  len   要读取的长度
* @param  buf   读取到的数据存储区
* @retval 0  正常  其他  错误数据
*/
uint8_t IIC_MPU6050_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令
    if(IIC_Wait_Ack()) {	//等待应答
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
    IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令
    IIC_Wait_Ack();		//等待应答
    while(len) {
        if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK
        else *buf=IIC_Read_Byte(1);		//读数据,发送ACK
        len--;
        buf++;
    }
    IIC_Stop();	//产生一个停止条件
    return 0;
}

/**
  * @brief  IIC连续写
  * @param  addr:器件地址
  * @param  reg:寄存器地址
  * @param  len:写入长度
  * @param  buf:数据区
  * @retval 0,成功
  *         其他,错误代码
  */
uint8_t IIC_MPU6050_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    unsigned char i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令
    if(IIC_Wait_Ack()) {	//等待应答
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    for(i=0; i<len; i++) {
        IIC_Send_Byte(buf[i]);	//发送数据
        if(IIC_Wait_Ack()) {	//等待ACK
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
}

/**
  * @brief   Get mpu data
	* @param   sensor: the pointer of ahrs data.
  * @retval  void
  */
void mpu_get_data(struct ahrs_sensor *sensor)
{
    uint8_t buf[14];
    if(IIC_MPU6050_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,14,buf)==0)
        //缩短时间，不解算姿态的情况下只获取所需的陀螺仪GYO3轴数据
    {
//如果要加速度值需要换寄存器
		MPU6050.ax = ((buf[0]<<8) | buf[1]) ;//- MPU6050.ax_offset;
		MPU6050.ay = ((buf[2]<<8) | buf[3]);// - MPU6050.ay_offset;
		MPU6050.az = ((buf[4]<<8) | buf[5]) ;//- MPU6050.az_offset - 4096;

        MPU6050.gx = (int16_t)(((buf[8]<<8) | buf[9])) ;//- MPU6050.gx_offset);
        MPU6050.gy = (int16_t)(((buf[10]<<8) | buf[11])) ;//- MPU6050.gy_offset);
        MPU6050.gz = (int16_t)(((buf[12]<<8) | buf[13])) ;//- MPU6050.gz_offset);


    }
	sensor->ax = MPU6050.ax / 4096.0f * 9.80665f;//2g -> m/s^2
	sensor->ay = MPU6050.ay / 4096.0f * 9.80665f;
	sensor->az = MPU6050.az / 4096.0f * 9.80665f;
    sensor->wx = MPU6050.gx / 32768.0f * 2000;//2000dps->rad/s
    sensor->wy = MPU6050.gy / 32768.0f * 2000;
    sensor->wz = MPU6050.gz / 32768.0f * 2000;
}
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/

