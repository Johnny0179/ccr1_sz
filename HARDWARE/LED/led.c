#include "led.h" 
#include "glb_reg.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

//初始化PF9和PF10为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOF时钟
  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;//LED0和LED1对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIO	
	GPIO_SetBits(GPIOE,GPIO_Pin_2 | GPIO_Pin_3);//GPIOF9,F10设置高，灯灭
	
	/*	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOF时钟
  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;//LED0和LED1对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化GPIO	
	GPIO_SetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);//GPIOF9,F10设置高，灯灭
	*/

}

void HardWare_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOG, ENABLE);//使能GPIO时钟
	
	//输出配置
	//GPIOA PA8
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//LED0和LED1对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO		
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);// 输出不导通	
	
	//PC6,PC7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;//LED0和LED1对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO		
	GPIO_ResetBits(GPIOC,GPIO_Pin_6|GPIO_Pin_7);// 输出不导通	
	
	
	//PG3,PG5,PG8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_8;//LED0和LED1对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化GPIO		
	GPIO_ResetBits(GPIOG,GPIO_Pin_8);// 输出不导通	
	GPIO_SetBits(GPIOG,GPIO_Pin_3 | GPIO_Pin_5); //2路电源使能默认是使能关闭的
	
	
	
	//GPIOE  PE2，PE3//LED灯
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;//LED0和LED1对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIO	
	//GPIO_SetBits(GPIOE,GPIO_Pin_2 | GPIO_Pin_3);//GPIOF9,F10设置高，灯灭		
	

  //输入配置
	//PA15
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//LED0和LED1对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//GPIO_Mode_OUT;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO		

	//PD0，PD3，PD7
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_3|GPIO_Pin_7;//LED0和LED1对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//GPIO_Mode_OUT;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIO		
	

	//PG6,PG7
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_12;//LED0和LED1对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//GPIO_Mode_OUT;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化GPIO		
	

	
	
}

void HandleGPIO(void)
{
	u16 tmp = 0;
/////////////////////////////////////////////////////////////
// DI 采样，暂时不做滤波	
	if(DI0!=0)
	{
		tmp |= 0x0001;
	}

	if(DI1 !=0)
	{
		tmp |= 0x0002;
	}
	
	if(DI2 !=0)
	{
		tmp |= 0x0004;
	}	

	if(DI3 !=0)
	{
		tmp |= 0x0008;
	}	
	
	if(DI4 !=0)
	{
		tmp |= 0x0010;
	}	
	
	if(DI5 !=0)
	{
		tmp |= 0x0020;
	}	

	if(DI6 !=0)
	{
		tmp |= 0x0040;
	}		
	
	if(DI7 !=0)
	{
		tmp |= 0x0080;
	}		
	
	//REG_DI = tmp;
	ptrMotionBlk->Di = (~tmp)&0x00ff; //20171213
	//REG_DI = (~tmp)&0x00ff; //更新DI寄存器
/////////////////////////////////////////////////////////////	

/////////////////////////////////////////////////////////////	
//DO 输出
	if((ptrMotionBlk->Do & 0x0001) != 0)
	{
		DO0 = 1;
	}
	else
	{
		DO0 = 0;
	}
	
	if((ptrMotionBlk->Do & 0x0002) != 0)
	{
		DO1 = 1;
	}
	else
	{
		DO1 = 0;
	}	
	
	
	if((ptrMotionBlk->Do & 0x0004) != 0)
	{
		DO2 = 1;
	}
	else
	{
		DO2 = 0;
	}	
	
	if((ptrMotionBlk->Do & 0x0008) != 0)
	{
		DO3 = 1;
	}
	else
	{
		DO3 = 0;
	}		
/////////////////////////////////////////////////////////////
	
/////////////////////////////////////////////////////////////
	//电源使能
	if((ptrCfgMotionPar->pwr_en & 0x0001) != 0)
	{
		EN_5V_OUT = 1;
	}
	else
	{
		EN_5V_OUT = 0;
	}	
	
	if((ptrCfgMotionPar->pwr_en & 0x0002) != 0)
	{
		EN_12V_OUT = 1;
	}
	else
	{
		EN_12V_OUT = 0;
	}		
	
	
	
	
	
}








