#include "main.h"
#include "gpio.h"

volatile uint8_t Key_Num;


uint8_t Key_GetNum(void)
{
		uint8_t temp;
		if( Key_Num )  //如果说Key_Num有东西，那么送出去
		{
				temp = Key_Num;
				Key_Num = 0;
			  
			  return temp;
		}

			return 0 ;
		
}

uint8_t Key_GetState(void)
{
//		if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin) == 0)
//		{
//				return 1 ;
//		}	

//		if(HAL_GPIO_ReadPin(KEYS_GPIO_Port,KEYS_Pin) == 0)
//		{
//				return 2 ;
//		}	
//		
//		if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin) == 0)
//		{
//				return 3 ;
//		}					
//		
		return 0;
}	


void key_Tick(void) //申请定时中断调用 1ms进入1次
{
		static uint8_t count;
		static uint8_t CurrState,PrevState;
	
		count++;
		if(count >= 20)
		{
				 PrevState = CurrState;         // 存个档。按下等于各个按键该有的值。松手是0
			CurrState = Key_GetState();		
			
				if( CurrState == 0 && PrevState != 0 )
				{
						Key_Num = PrevState ;
				
				}	
		}	
}	

