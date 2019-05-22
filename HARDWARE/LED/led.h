#ifndef __LED_H
#define __LED_H
#include "sys.h"

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

//输出端口定义
//LED端口定义
//#define LED0 PFout(9)	// DS0
//#define LED1 PFout(10)	// DS1	

#define LED0 PEout(2)	// DS0
#define LED1 PEout(3)	// DS1	

#define DO0 PCout(6)
#define DO1 PGout(8)
#define DO2 PAout(8)
#define DO3 PCout(7)

#define EN_5V_OUT PGout(3)
#define EN_12V_OUT PGout(5)

//输入端口定义
#define DI0 		PGin(7)   	//
#define DI1 		PGin(6)   	//
#define DI2 		PAin(15)   	//
#define DI3 		PDin(0)   	//
#define DI4 		PDin(3)   	//
#define DI5 		PDin(7)   	//
#define DI6 		PGin(10)   	//
#define DI7 		PGin(12)   	//

extern void LED_Init(void);//初始化
extern void HardWare_Init(void);
extern void HandleGPIO(void);



#endif
