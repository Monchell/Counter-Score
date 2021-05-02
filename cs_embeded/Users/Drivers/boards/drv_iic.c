/**
  ******************************************************************************
  * @file 	 drv_iic.cpp
  * @author  Sweet
  * @brief 	 STM32F4���gpioģ��iic


  * @version 1.0
  * @date		 2019.06.23
  * @editby

  ==============================================================================
                     ##### How to use this conf #####
  ==============================================================================
	1������bsp_iic.h
	2��ʵ���� hiic ���
			����һ��IIC_HandleTypeDefָ��
	3������ IIC_Config ����io��
	4������ IIC_Init ��ʼ��
	5�����������ⲿ����
  ******************************************************************************
  * @attention

  ******************************************************************************
  */

/* Includes ----------------------------------------------------------------------*/
#include "drv_iic.h"
#include "drv_timer.h"
GPIO_TypeDef *SDA_GPIOx;
uint16_t SDA_PIN;
GPIO_TypeDef *SCL_GPIOx;
uint16_t SCL_PIN;
/* Private function prototypes -----------------------------------------------*/
/**
* @brief IIC��ʼ��
* @param void
* @return void
*/
void IIC_Init(void)
{
	//��ʼ��SCL����
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SCL_GPIOx, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SCL_GPIOx, SCL_PIN, GPIO_PIN_SET); 
    IIC_SCL(1);
    IIC_SDA(1);
}

/**
* @brief ����IIC��ʼ�ź�
* @param mode ѡ��IIC���ģ��
* @return void
*/
void IIC_Start(void)
{
    SDA_OUT();//sda�����
    IIC_SDA(1);
    IIC_SCL(1);
    delay_us(1);
    IIC_SDA(0);//START:when CLK is high,DATA change form high to low
    delay_us(1);
    IIC_SCL(0);//ǯסI2C���ߣ�׼�����ͻ��������
}


/**
* @brief ����IICֹͣ�ź�
* @param mode ѡ��IIC���ģ��
* @return void
*/
void IIC_Stop(void)
{
    SDA_OUT();//sda�����
    IIC_SCL(0);
    IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
    delay_us(1);
    IIC_SCL(1);
    IIC_SDA(1);//����I2C���߽����ź�
    delay_us(1);
}


/**
* @brief �ȴ�Ӧ���źŵ���
* @param mode ѡ��IIC���ģ��
* @return 1������Ӧ��ʧ��
*					0������Ӧ��ɹ�
*/
uint8_t IIC_Wait_Ack(void)
{
    uint8_t flag=0;
    uint8_t ucErrTime=0;
    SDA_IN();      //SDA����Ϊ����

    IIC_SDA(1);
    delay_us_nos(1);
    IIC_SCL(1);
    delay_us_nos(1);
    while(READ_SDA()) {
        ucErrTime++;
        if(ucErrTime>250) {
            IIC_Stop();
            flag = 1;
            break;
        }

    }
    IIC_SCL(0);//ʱ�����0
    return flag;
}


/**
* @brief ����ACKӦ��
* @param void
* @return void
*/
void IIC_Ack(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(0);
    delay_us(1);
    IIC_SCL(1);
    delay_us(1);
    IIC_SCL(0);
}


/**
* @brief ������ACKӦ��
* @param void
* @return void
*/
void IIC_NAck(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(1);
    delay_us(1);
    IIC_SCL(1);
    delay_us(1);
    IIC_SCL(0);
}


/**
* @brief IIC����һ���ֽ�
* @param mode ѡ��IIC���ģ��
*				 txd
* @return void
*/
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    IIC_SCL(0);//����ʱ�ӿ�ʼ���ݴ���
    for(t=0; t<8; t++) {
        IIC_SDA((txd&0x80)>>7);
        txd<<=1;
        delay_us(1);   //��TEA5767��������ʱ���Ǳ����
        IIC_SCL(1);
        delay_us(1);
        IIC_SCL(0);
        delay_us(1);
    }
}


/**
* @brief IIC��ȡһ���ֽ�
* @param mode ѡ��IIC���ģ��
*				 ack	1������ACK
*							0������nACK
* @return receive	��ȡ�����ֽ�
*/
uint8_t IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDA_IN();//SDA����Ϊ����
    for(i=0; i<8; i++ ) {
        IIC_SCL(0);
        delay_us(1);
        IIC_SCL(1);
        receive<<=1;
        if(READ_SDA())receive++;
        delay_us(1);
    }
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK
    return receive;
}

/**
  * @brief  SCL ������ݣ���ͬ��io�������Ӧ�ļĴ���
  * @param  const char x
  * @retval void
  */
void IIC_SCL(const char x)
{
    /* ip������ߵ͵�ƽ */
    (x!=0)?HAL_GPIO_WritePin(SCL_GPIOx,SCL_PIN,GPIO_PIN_SET):\
    HAL_GPIO_WritePin(SCL_GPIOx,SCL_PIN,GPIO_PIN_RESET);
}

/**
  * @brief  SDA �������ͬ��io�������Ӧ�ļĴ���
  * @param  const char x
  * @retval void
  */
void IIC_SDA(const char x)
{
    /* ip������ߵ͵�ƽ */
    (x!=0)?HAL_GPIO_WritePin(SDA_GPIOx,SDA_PIN,GPIO_PIN_SET):\
    HAL_GPIO_WritePin(SDA_GPIOx,SDA_PIN,GPIO_PIN_RESET);
}

/**
  * @brief  �޸�SDAΪ����ģʽ����ͬ��io�������Ӧ�ļĴ���
  * @param  void
  * @retval void
  */
void SDA_IN(void)
{
     GPIO_InitTypeDef GPIO_InitStruct;
     GPIO_InitStruct.Pin = SDA_PIN;
     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
     GPIO_InitStruct.Pull = GPIO_PULLUP;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SDA_GPIOx, &GPIO_InitStruct);
}

/**
  * @brief  �޸�SDAΪ���ģʽ����ͬ��IO�������Ӧ�ļĴ���
  * @param  void
  * @retval void
  */
void SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SDA_GPIOx, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SDA_GPIOx, SDA_PIN, GPIO_PIN_RESET);    
}

/**
  * @brief  ��IO��ֱ���ճ���hal��read
  * @param  void
  * @retval GPIO_PinState
  */
GPIO_PinState READ_SDA(void)
{
	return HAL_GPIO_ReadPin(SDA_GPIOx, SDA_PIN);    
}
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/

