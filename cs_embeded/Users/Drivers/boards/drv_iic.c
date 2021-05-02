/**
  ******************************************************************************
  * @file 	 drv_iic.cpp
  * @author  Sweet
  * @brief 	 STM32F4软件gpio模拟iic


  * @version 1.0
  * @date		 2019.06.23
  * @editby

  ==============================================================================
                     ##### How to use this conf #####
  ==============================================================================
	1、包含bsp_iic.h
	2、实例化 hiic 句柄
			声明一个IIC_HandleTypeDef指针
	3、调用 IIC_Config 配置io口
	4、调用 IIC_Init 初始化
	5、正常调用外部函数
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
* @brief IIC初始化
* @param void
* @return void
*/
void IIC_Init(void)
{
	//初始化SCL引脚
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
* @brief 产生IIC起始信号
* @param mode 选择IIC输出模块
* @return void
*/
void IIC_Start(void)
{
    SDA_OUT();//sda线输出
    IIC_SDA(1);
    IIC_SCL(1);
    delay_us(1);
    IIC_SDA(0);//START:when CLK is high,DATA change form high to low
    delay_us(1);
    IIC_SCL(0);//钳住I2C总线，准备发送或接收数据
}


/**
* @brief 产生IIC停止信号
* @param mode 选择IIC输出模块
* @return void
*/
void IIC_Stop(void)
{
    SDA_OUT();//sda线输出
    IIC_SCL(0);
    IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
    delay_us(1);
    IIC_SCL(1);
    IIC_SDA(1);//发送I2C总线结束信号
    delay_us(1);
}


/**
* @brief 等待应答信号到来
* @param mode 选择IIC输出模块
* @return 1，接收应答失败
*					0，接收应答成功
*/
uint8_t IIC_Wait_Ack(void)
{
    uint8_t flag=0;
    uint8_t ucErrTime=0;
    SDA_IN();      //SDA设置为输入

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
    IIC_SCL(0);//时钟输出0
    return flag;
}


/**
* @brief 产生ACK应答
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
* @brief 不产生ACK应答
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
* @brief IIC发送一个字节
* @param mode 选择IIC输出模块
*				 txd
* @return void
*/
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    IIC_SCL(0);//拉低时钟开始数据传输
    for(t=0; t<8; t++) {
        IIC_SDA((txd&0x80)>>7);
        txd<<=1;
        delay_us(1);   //对TEA5767这三个延时都是必须的
        IIC_SCL(1);
        delay_us(1);
        IIC_SCL(0);
        delay_us(1);
    }
}


/**
* @brief IIC读取一个字节
* @param mode 选择IIC输出模块
*				 ack	1，发送ACK
*							0，发送nACK
* @return receive	读取到的字节
*/
uint8_t IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDA_IN();//SDA设置为输入
    for(i=0; i<8; i++ ) {
        IIC_SCL(0);
        delay_us(1);
        IIC_SCL(1);
        receive<<=1;
        if(READ_SDA())receive++;
        delay_us(1);
    }
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK
    return receive;
}

/**
  * @brief  SCL 输出内容，不同的io请查阅相应的寄存器
  * @param  const char x
  * @retval void
  */
void IIC_SCL(const char x)
{
    /* ip暑输出高低电平 */
    (x!=0)?HAL_GPIO_WritePin(SCL_GPIOx,SCL_PIN,GPIO_PIN_SET):\
    HAL_GPIO_WritePin(SCL_GPIOx,SCL_PIN,GPIO_PIN_RESET);
}

/**
  * @brief  SDA 输出，不同的io请查阅相应的寄存器
  * @param  const char x
  * @retval void
  */
void IIC_SDA(const char x)
{
    /* ip暑输出高低电平 */
    (x!=0)?HAL_GPIO_WritePin(SDA_GPIOx,SDA_PIN,GPIO_PIN_SET):\
    HAL_GPIO_WritePin(SDA_GPIOx,SDA_PIN,GPIO_PIN_RESET);
}

/**
  * @brief  修改SDA为输入模式，不同的io请查阅相应的寄存器
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
  * @brief  修改SDA为输出模式，不同的IO请查阅相应的寄存器
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
  * @brief  读IO，直接照抄的hal库read
  * @param  void
  * @retval GPIO_PinState
  */
GPIO_PinState READ_SDA(void)
{
	return HAL_GPIO_ReadPin(SDA_GPIOx, SDA_PIN);    
}
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/

