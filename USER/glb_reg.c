// ȫ�ֱ���

#include "glb_reg.h"

volatile u16 dGlbReg [HOLDING_REG_END];

//SERV_PAR1 * const ptrServPar = (SERV_PAR1 *)(&Para[ADDR_PAR_BASIC]);  //?????,????A4




const u16  Bit[16] = {
	0x0001<<0,	
	0x0001<<1,
	0x0001<<2,
	0x0001<<3,	
	0x0001<<4,	
	0x0001<<5,
	0x0001<<6,
	0x0001<<7,
	0x0001<<8,
	0x0001<<9,
	0x0001<<10,
	0x0001<<11,
	0x0001<<12,
	0x0001<<13,
	0x0001<<14,
	0x0001<<15	
}; 


u16 SlaveIdAddr = 1;
u16 DebugFlag = 0;

SERV_PAR_TYPE * const ptrServ[8] = {(SERV_PAR_TYPE *)(&Para[SERV0_BASE]),(SERV_PAR_TYPE *)(&Para[SERV1_BASE]),(SERV_PAR_TYPE *)(&Para[SERV2_BASE]),(SERV_PAR_TYPE *)(&Para[SERV3_BASE]),\
																		(SERV_PAR_TYPE *)(&Para[SERV4_BASE]),(SERV_PAR_TYPE *)(&Para[SERV5_BASE]),(SERV_PAR_TYPE *)(&Para[SERV6_BASE]),(SERV_PAR_TYPE *)(&Para[SERV7_BASE])};

MOTION_VAR_TYPE * const ptrMotionBlk = (MOTION_VAR_TYPE *) (&Para[MOTION_STATE_BASE]);
																		
CFG_MOTION_PAR_TYPE * const ptrCfgMotionPar = (CFG_MOTION_PAR_TYPE *) (&Para[CFG_MOTION_PAR_BASE]);


 



