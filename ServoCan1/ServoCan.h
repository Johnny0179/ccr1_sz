#ifndef __ServoCAN__
#define    __ServoCAN__

#include "stm32f4xx.h"
#include "includes.h"

 #ifndef Para
#define Para dGlbReg
#endif

//CAN1����RX0�ж�ʹ��
#define CAN1_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.		

extern u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��

extern u8 CAN1_Send_Frame(u16 CobId,u8 Rtr,u8 Len,u8* msg);	//��������

extern u8 CAN1_Receive_Msg(u8 *buf);							//��������

extern void CanInit(void);
extern void MotorInit(void);
extern void HandleCan(void);



#endif
