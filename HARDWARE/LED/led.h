#ifndef __LED_H
#define __LED_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//����˿ڶ���
//LED�˿ڶ���
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

//����˿ڶ���
#define DI0 		PGin(7)   	//
#define DI1 		PGin(6)   	//
#define DI2 		PAin(15)   	//
#define DI3 		PDin(0)   	//
#define DI4 		PDin(3)   	//
#define DI5 		PDin(7)   	//
#define DI6 		PGin(10)   	//
#define DI7 		PGin(12)   	//

extern void LED_Init(void);//��ʼ��
extern void HardWare_Init(void);
extern void HandleGPIO(void);



#endif
