#ifndef __ServoCAN__
#define    __ServoCAN__

#include "stm32f4xx.h"
#include "includes.h"

 #ifndef Para
#define Para dGlbReg
#endif

//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.		

extern u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化

extern u8 CAN1_Send_Frame(u16 CobId,u8 Rtr,u8 Len,u8* msg);	//发送数据

extern u8 CAN1_Receive_Msg(u8 *buf);							//接收数据

extern void CanInit(void);
extern void MotorInit(void);
extern void HandleCan(void);



#endif
