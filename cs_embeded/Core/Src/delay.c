#include "delay.h"

void Delay_us(uint16_t us)
{ 
    uint16_t counter=0;
    HAL_TIM_Base_Start(&tim);
	  __HAL_TIM_SET_COUNTER(&tim,counter);
    while(counter<us)
    {
	  counter=__HAL_TIM_GET_COUNTER(&tim);
	}
    
    HAL_TIM_Base_Stop(&htim3);
}
	
void Delay_ms(uint16_t ms)
{
    for (uint16_t i = 0; i < ms; i++)
    {
       Delay_us(1000);
    }
}
