#ifndef __ServoCAN__
#define    __ServoCAN__

#include "stm32f4xx.h"
#include "includes.h"

 #ifndef Para
#define Para dGlbReg
#endif

enum enum_nodeState {
  Initialisation  = 0x00, 
  Disconnected    = 0x01,
  Connecting      = 0x02,
  Preparing       = 0x02,
  Stopped         = 0x04,
  Operational     = 0x05,
  Pre_operational = 0x7F,
  Unknown_state   = 0x0F
};
typedef enum enum_nodeState e_nodeState;


#define SERV_ON_PRE 0x0006
#define SERV_ON 0x000f
#define SERV_OFF 0x0000

#define SERV_ABS_POSSET 0x003F
#define SERV_REL_POSSET 0x007F
#define SERV_HALT_BIT       0x0100

#define TIME_INTERVAL_US 2000


/*  Function Codes 
   ---------------
  defined in the canopen DS301 
*/
#define NMT	   ((u16)0x0<<7)
#define SYNC       ((u16)0x1<<7)
#define TIME_STAMP ((u16)0x2<<7)
#define PDO1tx     ((u16)0x3<<7)
#define PDO1rx     ((u16)0x4<<7)
#define PDO2tx     ((u16)0x5<<7)
#define PDO2rx     ((u16)0x6<<7)
#define PDO3tx     ((u16)0x7<<7)
#define PDO3rx     ((u16)0x8<<7)
#define PDO4tx     ((u16)0x9<<7)
#define PDO4rx     ((u16)0xA<<7)
#define SDOtx      ((u16)0xB<<7)
#define SDOrx      ((u16)0xC<<7)
#define NODE_GUARD ((u16)(0xE)<<7)
#define LSS 	   ((u16)0xF<<7)

/* NMT Command Specifier, sent by master to change a slave state */
/* ------------------------------------------------------------- */
/* Should not be modified */
#define NMT_Start_Node              0x01
#define NMT_Stop_Node               0x02
#define NMT_Enter_PreOperational    0x80
#define NMT_Reset_Node              0x81
#define NMT_Reset_Comunication      0x82

//#define Message_Initializer {0,0,0,{0,0,0,0,0,0,0,0}}


#define Rd_STA_TRQ_POS RX_PDO1
#define Rd_STA_TRQ_SPD RX_PDO2
#define Rd_SPD_POS 		RX_PDO3
#define Rd_STA_ERR_TRQ_MODE 	RX_PDO4

extern OS_EVENT *sem_SrvCAN_rx;

extern u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化

extern u8 CAN1_Send_Frame(u16 CobId,u8 Rtr,u8 Len,u8* msg);	//发送数据

extern u8 CAN1_Receive_Msg(u8 *buf);							//接收数据

extern u8 NMT_Start(u8 SlaveID);
extern u8 NMT_Stop(u8 SlaveID);
extern u8 NMT_PreSTA(u8 SlaveID);
extern u8 NMT_RstNode(u8 SlaveID);
extern u8 NMT_RstComm(u8 SlaveID);
extern u8 CMD_SYNC(void);

extern u8 TX_PDO1(u8 SlaveID);//extern u8 TX_PDO1(u8 SlaveID,u8 *msg);
extern u8 TX_PDO2(u8 SlaveID);//extern u8 TX_PDO2(u8 SlaveID,u8 *msg);
extern u8 TX_PDO3(u8 SlaveID);//extern u8 TX_PDO3(u8 SlaveID,u8 *msg);
extern u8 TX_PDO4(u8 SlaveID);//extern u8 TX_PDO4(u8 SlaveID,u8 *msg);

extern u8 SetMotorAbsPos(u8 SlaveID, s32 AbsPos);
extern u8 SetMotorRelPos(u8 SlaveID, s32 RelPos);
extern u8 SetMotorCurrentLimit(u8 SlaveID, s16 MaxCurrent);
extern u8 SetMotorSpd(u8 SlaveID, s32 Spd);
extern u8 SetMotorCtrlword(u8 SlaveID, u16 Ctrlword);
extern u8 SetServOn(u8 SlaveID);
extern u8 SetServRdy(u8 SlaveID);
extern u8 SetServOff(u8 SlaveID);
extern void StartMotor(u8 SlaveID);
extern void StopMotor(u8 SlaveID);

extern u8 RX_PDO1(u8 SlaveID);
extern u8 RX_PDO2(u8 SlaveID);
extern u8 RX_PDO3(u8 SlaveID);
extern u8 RX_PDO4(u8 SlaveID);
extern u8 Rd_NodeGuard(u8 SlaveID);
extern u8 Sdo_WrU8(u8 SlaveID,u16 index,u8 subindex,u32 data); //写1字节
extern u8 Sdo_WrU16(u8 SlaveID,u16 index,u8 subindex,u32 data); //写2字节
extern u8 Sdo_WrU24(u8 SlaveID,u16 index,u8 subindex,u32 data); //写3字节，实际只用3byte
extern u8 Sdo_WrU32(u8 SlaveID,u16 index,u8 subindex,u32 data); //写4字节
extern u8 Sdo_Rd(u8 SlaveID,u16 index,u8 subindex); //读1字节
extern void ProcessSDOrx(CanRxMsg *m);
extern u8 ProcessSDOtx(CanRxMsg *m);
extern void canDispatch(CanRxMsg *m);

extern void CanInit(void);
extern void MotorInit(void);
extern void HandleCan(void);

//void ProcessSDOrx(CanRxMsg *m);
//u8 ProcessSDOtx(CanRxMsg *m);
//void canDispatch(CanRxMsg *m);



#endif
