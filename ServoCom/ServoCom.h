#ifndef __ServoCOM__
#define    __ServoCOM__

#include "stm32f4xx.h"
#include "includes.h"

 #ifndef Para
#define Para dGlbReg
#endif

#define RS485_TX_EN		PGout(8)	//485模式控制.0,接收;1,发送.

#define SrvCOM_IDLE 0
#define SrvCOM_TX_FRAME 1
#define SrvCOM_TX 2
#define SrvCOM_RX 3
#define SrvCOM_RX_FRAME 4


#define RX_TIMEOUT 20

volatile typedef struct{
u8 tx[256];
u8 rx[256];
u16 tx_cnt;
u16 tx_end;
u16 rx_cnt;
u16 rx_end;
u16 err;
u16 state;
u16 tx_flag;
u16 timeout;
u16 timeout_en; 
}COM2TxRxTYPE;

volatile typedef struct{
u16 cmd;
u16 localpar_index; //本地参数表地址
u16 axis_id;
u16 srvpar_addr;//伺服参数表地址
u16 dat_len; //1：8bit ,2:16 bit, 4:32 bit
}ComTaskTYPE;


extern OS_EVENT *sem_SrvCom_rx;
extern OS_EVENT *sem_SrvCom_tx;
extern COM2TxRxTYPE Com2TxRx;
extern ComTaskTYPE ComTask;
extern const ComTaskTYPE ComTaskTable[];
extern volatile u16 ComTaskTableIndex;

extern void PackFrameRd(void);
extern void PackFrameWr(void);
extern void UnPackFrameRd(void);
extern void UnPackFrameWr(void);

extern void USART2_Init(u32 bound);
extern void ServoCom_Init(void);
extern void ServoComRW(void);

#endif



