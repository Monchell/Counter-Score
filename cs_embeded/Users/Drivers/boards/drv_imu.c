/**
******************************************************************************
* @file 		drv_imu.c
* @author 	charlie 602894526@qq.com
* @brief  	MPU6050��ص�һЩ����
* @version 	2.0
* @date
* @editby 	charlie

==============================================================================
										##### How to use this conf #####
==============================================================================
1.0:
IIC_MPU6050_Write_Byte  		��IICЭ���µ�MPU6050������д����
IIC_MPU6050_Read_Byte				��IICЭ���µ�MPU6050�����Ƕ�����
IIC_MPU6050_Set_Gyro_Fsr		��IICЭ���µ�MPU6050���������ýǶ�ֵ�������̷�Χ
IIC_MPU6050_Set_Accel_Fsr		��IICЭ���µ�MPU6050���������ü��ٶ�ֵ�������̷�Χ
IIC_MPU6050_Set_LPF					��IICЭ���µ�MPU6050�������������ֵ�ͨ�˲���
IIC_MPU6050_Set_Rate				��IICЭ���µ�MPU6050���������ò�����
IIC_MPU6050_Init						IICЭ���µ�MPU6050�����ǳ�ʼ������ʼ��״̬����
MPU6050_Cal									����100������������ȡƽ����ΪOFFSET
imu_config					IICЭ���µ�MPU6050�����ǳ�ʼ��
IIC_MPU6050_Get_Accelerometer		�õ�IICЭ���µ�MPU6050�����Ǽ��ٶ�ֵ��ԭʼֵ��
IIC_MPU6050_Get_Gyroscope		�õ�IICЭ���µ�MPU6050������ֵ��ԭʼֵ��
IIC_MPU6050_Read_Len				IICЭ���µ�MPU6050������

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
MPU6050_t MPU6050;        //MPU6050��ȡ����

/* Private type --------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
* @brief  ��IICЭ���µ�MPU6050������д����
* @param  reg  �Ĵ�����ַ
* @param  data д�����������
* @retval 0  д���ݳɹ�   1  ����д����
*/
uint8_t IIC_MPU6050_Write_Byte(uint8_t reg,uint8_t data)
{
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����
    if(IIC_Wait_Ack()) {	//�ȴ�Ӧ��
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Send_Byte(data);//��������
    if(IIC_Wait_Ack()) {	//�ȴ�ACK
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}

/**
* @brief  ��IICЭ���µ�MPU6050�����Ƕ�����
* @param  reg  �Ĵ�����ַ
* @retval ��ȡ���Ķ�Ӧ�Ĵ�����ַ����������
*/
uint8_t IIC_MPU6050_Read_Byte(uint8_t reg)
{
    uint8_t res;
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR<<1)|1);//����������ַ+������
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    res=IIC_Read_Byte(0);//��ȡ����,����nACK
    IIC_Stop();			//����һ��ֹͣ����
    return res;
}

/**
* @brief  ��IICЭ���µ�MPU6050���������ýǶ�ֵ�������̷�Χ
* @param  fsr  �����ǽǶ�ֵ�������
* @retval 0  ���óɹ�  1  ��������
*/
uint8_t IIC_MPU6050_Set_Gyro_Fsr(uint8_t fsr)
{
    return IIC_MPU6050_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ
}

/**
* @brief  ��IICЭ���µ�MPU6050���������ü��ٶ�ֵ�������̷�Χ
* @param  fsr  �����Ǽ��ٶ�ֵ�������
* @retval 0  ���óɹ�  1  ��������
*/
uint8_t IIC_MPU6050_Set_Accel_Fsr(uint8_t fsr)
{
    return IIC_MPU6050_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ
}

/**
* @brief  ��IICЭ���µ�MPU6050�������������ֵ�ͨ�˲���
* @param  lpf  ���ֵ�ͨ�˲�Ƶ��(Hz)
* @retval 0  ���óɹ�  1  ��������
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
    return IIC_MPU6050_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���
}

/**
* @brief  ��IICЭ���µ�MPU6050���������ò�����(�ٶ�Fs=1KHz)
* @param  rate  ������ 4~(Hz)
* @retval 0  ���óɹ�  1  ��������
*/
uint8_t IIC_MPU6050_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=IIC_MPU6050_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
    return IIC_MPU6050_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

/**
* @brief  IICЭ���µ�MPU6050�����ǳ�ʼ������ʼ��״̬����
* @param  void
* @retval 0  ��ʼ���ɹ�  1  ���ڳ�ʼ��
*/
uint8_t IIC_MPU6050_Init(void)
{
    uint8_t res;
    IIC_Init();//��ʼ��IIC����
    IIC_MPU6050_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
    delay_ms_nos(100);

    IIC_MPU6050_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050

    IIC_MPU6050_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
    IIC_MPU6050_Set_Accel_Fsr(2);					//���ٶȴ�����,��2g
    IIC_MPU6050_Set_Rate(500);						//���ò�����50Hz

    IIC_MPU6050_Write_Byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
    IIC_MPU6050_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
    IIC_MPU6050_Write_Byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
    IIC_MPU6050_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
    res=IIC_MPU6050_Read_Byte(MPU_DEVICE_ID_REG);

    if(res==MPU_ADDR) { //����ID��ȷ
        IIC_MPU6050_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
        IIC_MPU6050_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
        IIC_MPU6050_Set_Rate(1000);						//���ò�����Ϊ50Hz
    } else
        return 1;
    return 0;
}

/**
* @brief  ����100������������ȡƽ����ΪOFFSET
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
* @brief  IICЭ���µ�MPU6050�����ǳ�ʼ��
* @param  void
* @retval 0  ��ʼ���ɹ�  1  ���ڳ�ʼ��
*/
void imu_config(void)
{
    while(IIC_MPU6050_Init()) {
        //        u1_printf("MPU Init Error\r\n");
    }
    MPU6050_Cal();			//�����ƫ
}

/**
* @brief  �õ�IICЭ���µ�MPU6050�����Ǽ��ٶ�ֵ��ԭʼֵ��
* @param  ax,ay,az  ������x,y,z���ԭʼ���ٶȶ���(������)
* @retval 0  ����  ����  ��������
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
* @brief  �õ�IICЭ���µ�MPU6050������ֵ��ԭʼֵ��
* @param  gx,gy,gz  ������x,y,z���ԭʼ����(������)
* @retval 0  ����  ����  ��������
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
* @brief  IICЭ���µ�MPU6050������
* @param  addr  ������ַ
* @param  reg   Ҫ��ȡ�ļĴ�����ַ
* @param  len   Ҫ��ȡ�ĳ���
* @param  buf   ��ȡ�������ݴ洢��
* @retval 0  ����  ����  ��������
*/
uint8_t IIC_MPU6050_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0);//����������ַ+д����
    if(IIC_Wait_Ack()) {	//�ȴ�Ӧ��
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
    IIC_Send_Byte((addr<<1)|1);//����������ַ+������
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    while(len) {
        if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK
        else *buf=IIC_Read_Byte(1);		//������,����ACK
        len--;
        buf++;
    }
    IIC_Stop();	//����һ��ֹͣ����
    return 0;
}

/**
  * @brief  IIC����д
  * @param  addr:������ַ
  * @param  reg:�Ĵ�����ַ
  * @param  len:д�볤��
  * @param  buf:������
  * @retval 0,�ɹ�
  *         ����,�������
  */
uint8_t IIC_MPU6050_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    unsigned char i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0);//����������ַ+д����
    if(IIC_Wait_Ack()) {	//�ȴ�Ӧ��
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    for(i=0; i<len; i++) {
        IIC_Send_Byte(buf[i]);	//��������
        if(IIC_Wait_Ack()) {	//�ȴ�ACK
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
    if(IIC_MPU6050_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf)==0)
        //����ʱ�䣬��������̬�������ֻ��ȡ�����������GYO3������
    {
//		MPU6050.ax = ((buf[0]<<8) | buf[1]) - MPU6050.ax_offset;
//		MPU6050.ay = ((buf[2]<<8) | buf[3]) - MPU6050.ay_offset;
//		MPU6050.az = ((buf[4]<<8) | buf[5]) - MPU6050.az_offset - 4096;

        MPU6050.gx = (int16_t)(((buf[0]<<8) | buf[1]) - MPU6050.gx_offset);
        MPU6050.gy = (int16_t)(((buf[2]<<8) | buf[3]) - MPU6050.gy_offset);
        MPU6050.gz = (int16_t)(((buf[4]<<8) | buf[5]) - MPU6050.gz_offset);

//		MPU6050.ax = ((buf[0]<<8) | buf[1]);
//		MPU6050.ay = ((buf[2]<<8) | buf[3]);
//		MPU6050.az = ((buf[4]<<8) | buf[5]);
//
//		MPU6050.gx = ((buf[8]<<8)  | buf[9]);
//		MPU6050.gy = ((buf[10]<<8) | buf[11]);
//		MPU6050.gz = ((buf[12]<<8) | buf[13]);
    }
//	sensor->ax = MPU6050.ax / 4096.0f * 9.80665f;//2g -> m/s^2
//	sensor->ay = MPU6050.ay / 4096.0f * 9.80665f;
//	sensor->az = MPU6050.az / 4096.0f * 9.80665f;
    sensor->wx = MPU6050.gx / 32768.0f * 2000;//2000dps->rad/s
    sensor->wy = MPU6050.gy / 32768.0f * 2000;
    sensor->wz = MPU6050.gz / 32768.0f * 2000;
}
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/

