/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : Servo_can.c
* Author             : 
* Date First Issued  : 27/04/2010
* Description        : UART的硬件配置和modbus协议处理
********************************************************************************/



#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "includes.h"
#include "glb_reg.h"
#include "ServoCan.h"



#define CAN_BAUD_500Kbps_brp 6  //波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
#define CAN_BAUD_1Mbps_brp 3



//信号量,用于SDO，RPDO的信号量
OS_EVENT *sem_SrvCAN_rx;
OS_EVENT *sem_SrvCAN_tx;

/*
//Can 指令发送命令 :
//对象字典索引定义
#define ModeOperation_Idx (u16)0x6060
#define ModeOperation_SubIdx 0x00
#define PPM_MODE 1

#define MaxPVelocity_Idx (u16)0x607F
#define MaxPVelocity_SubIdx 0x00

#define PAcceleration_Idx (u16)0x6083
#define PAcceleration_SubIdx 0x00

#define PDeceleration_Idx (u16)0x6084
#define PDeceleration_SubIdx 0x00

#define QuickStopDec_Idx (u16)0x6085
#define QuickStopDec_SubIdx 0x00

#define MotionPType_Idx (u16)0x6086  //linear or Sin2C 加减速方式
#define MotionPType_SubIdx 0x00

#define CtrlWord_Idx (u16)0x6040  
#define CtrlWord_SubIdx 0x00

#define StatusWord_Idx (u16)0x6041  
#define StatusWord_SubIdx 0x00

#define TargetSpd_Idx (u16)0x60FF  
#define TargetSpd_SubIdx 0x00

#define TargetPos_Idx (u16)0x607A  
#define TargetPos_SubIdx 0x00

#define OutCurrentLimit_Idx (u16)0x3001
#define OutCurrentLimit_SubIdx 02
*/




//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//则波特率为:42M/((6+7+1)*6)=500Kbps
//返回值:0,初始化OK;
//    其他,初始化失败; 


u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure; 
   	NVIC_InitTypeDef  NVIC_InitStructure;

    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
		
    
		//配置过滤器
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		

	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		
    // 后面有空配置一下发送邮箱为空的中断。
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

#if 1
	  CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);//FIFO0消息挂号中断允许.		
    // 后面有空配置一下发送邮箱为空的中断。
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);				
#endif

	return 0;
}  




//中断服务函数			    
void CAN1_RX0_IRQHandler(void)
{
	  		CanRxMsg RxMessage ;
		OSIntEnter();     ////如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	

		if(CAN_GetITStatus(CAN1,CAN_IT_FMP0) != RESET)
		{
			CAN_Receive(CAN1, 0, &RxMessage);
			canDispatch(&RxMessage);
			CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
		}  	

		OSIntExit();     ////如果SYSTEM_SUPPORT_OS为真，则需要支持OS.		
}

#if 1
void CAN1_TX_IRQHandler(void)
{
	  
		OSIntEnter();     ////如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
		if(CAN_GetITStatus(CAN1,CAN_IT_TME) != RESET)
		{		
			OSSemPost(sem_SrvCAN_tx);
			CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
		}  	

		OSIntExit();     ////如果SYSTEM_SUPPORT_OS为真，则需要支持OS.		
}
#endif


//printf("StdId:%x\r\n",RxMessage.StdId);
//	for(i=0;i<8;i++)
//	printf("rxbuf[%d]:%x\r\n",i,RxMessage.Data[i]);
//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
//死等发完
u8 CAN1_Send_Frame(u16 CobId,u8 Rtr,u8 Len,u8* msg)
{	
  u8 mbox;
	u8 err;
  u16 i;
  CanTxMsg TxMessage;
	
  TxMessage.StdId=CobId;	 // 标准标识符,0x601,0x581
  TxMessage.ExtId= 0;//0x12;	 // 设置扩展标示符（29位）,暂时没用
  TxMessage.IDE=CAN_ID_STD;		  // 标准帧，不使用扩展标识符
	
	if(Rtr == 0)
		TxMessage.RTR= CAN_RTR_DATA;////;		  // 消息类型为数据帧，或
	else
		TxMessage.RTR = CAN_RTR_Remote; //RTR 请求帧
	
  TxMessage.DLC=Len;							 // 发送两帧信息
  for(i=0;i<Len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息    


	OSSemPend(sem_SrvCAN_tx,0,&err); //申请信号量资源
	
  mbox= CAN_Transmit(CAN1, &TxMessage); 
#if 1	
	i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		
#else
		if(mbox == CAN_TxStatus_NoMailBox)	
				return 1;	//err //还可以设置自动重传
		else 
				return 0; //OK   

#endif
}


//1.NMT命令
u8 NMT_Start(u8 SlaveID)
{
	u8 msg[8];	
	msg[0] = NMT_Start_Node;
	msg[1] = SlaveID;
	return CAN1_Send_Frame(NMT,0,2,msg);
}

u8 NMT_Stop(u8 SlaveID)
{
	u8 msg[8];	
	msg[0] = NMT_Stop_Node;
	msg[1] = SlaveID;
	return CAN1_Send_Frame(NMT,0,2,msg);
}

u8 NMT_PreSTA(u8 SlaveID)
{
	u8 msg[8];	
	msg[0] = NMT_Enter_PreOperational;
	msg[1] = SlaveID;
	return CAN1_Send_Frame(NMT,0,2,msg);
}

u8 NMT_RstNode(u8 SlaveID)
{
	u8 msg[8];	
	msg[0] = NMT_Reset_Node;
	msg[1] = SlaveID;
	return CAN1_Send_Frame(NMT,0,2,msg);
}

u8 NMT_RstComm(u8 SlaveID)
{
	u8 msg[8];	
	msg[0] = NMT_Reset_Comunication;
	msg[1] = SlaveID;
	return CAN1_Send_Frame(NMT,0,2,msg);
}

//2.同步命令
u8 CMD_SYNC(void)
{
	return CAN1_Send_Frame((u16)SYNC,0,0,(void*)NULL);
}

//3.TPDO 命令. 主站发送数据给从站
u8 TX_PDO1(u8 SlaveID)
{
	//控制字CtrlWord
	u8 msg[8];
	msg[0] = (ptrServ[SlaveID]->CtrlWord)&0xff;
	msg[1] = (ptrServ[SlaveID]->CtrlWord >>8)&0xff;
	return CAN1_Send_Frame((PDO1rx + SlaveID),0,2,msg);//	PDO1Rx 为 0x200,
}

u8 TX_PDO2(u8 SlaveID)
{
	//控制字CtrlWord与位置POS_Set
	u8 msg[8];
	msg[0] = (ptrServ[SlaveID]->CtrlWord)&0xff;
	msg[1] = (ptrServ[SlaveID]->CtrlWord >>8)&0xff;

	msg[2] = (ptrServ[SlaveID]->PosSV)&0xff;
	msg[3] = (ptrServ[SlaveID]->PosSV >>8)&0xff;
	msg[4] = (ptrServ[SlaveID]->PosSV >>16)&0xff;
	msg[5] = (ptrServ[SlaveID]->PosSV >>24)&0xff;
	
	
	return CAN1_Send_Frame((PDO2rx + SlaveID),0,6,msg);	
}

u8 TX_PDO3(u8 SlaveID)
{
	//速度
	u8 msg[8];
	msg[0] = (ptrServ[SlaveID]->SpdSV)&0xff;
	msg[1] = (ptrServ[SlaveID]->SpdSV >>8)&0xff;
	msg[2] = (ptrServ[SlaveID]->SpdSV >>16)&0xff;
	msg[3] = (ptrServ[SlaveID]->SpdSV >>24)&0xff;	

	
	return CAN1_Send_Frame((PDO3rx + SlaveID),0,4,msg);	
}

u8 TX_PDO4(u8 SlaveID)
{
	//最大电流限制
	u8 msg[8];
	//msg[0] = (ptrServ[SlaveID]->CtrlWord)&0xff;
//	msg[1] = (ptrServ[SlaveID]->CtrlWord >>8)&0xff;
	
  //MaxCurrenLimit 为16bit
	msg[0] = (ptrServ[SlaveID]->MaxCurrenLimit)&0xff;
	msg[1] = (ptrServ[SlaveID]->MaxCurrenLimit >>8)&0xff;
	msg[2] = 0;//(ptrServ[SlaveID]->MaxCurrenLimit >>16)&0xff;
	msg[3] = 0;//(ptrServ[SlaveID]->MaxCurrenLimit >>24)&0xff;			
	
	return CAN1_Send_Frame((PDO4rx + SlaveID),0,4,msg);	
}

u8 SetMotorCtrlword(u8 SlaveID, u16 Ctrlword)
{
	ptrServ[SlaveID]->CtrlWord = Ctrlword;
	return TX_PDO1(SlaveID);
}

void StartMotor(u8 SlaveID)
{
//		NMT_Start(0);	
//		delay_us(TIME_INTERVAL_US);	
//		delay_us(TIME_INTERVAL_US);	
//		delay_us(TIME_INTERVAL_US);		
		SetMotorCtrlword(SlaveID,0x0006);			
		delay_us(TIME_INTERVAL_US);		
		SetMotorCtrlword(SlaveID,0x000F);	
}

void StopMotor(u8 SlaveID)
{
		SetMotorCtrlword(SlaveID,0x0000);
		//delay_us(TIME_INTERVAL_US);	
	//	delay_us(TIME_INTERVAL_US);
	//	NMT_PreSTA(0);	
}

u8 SetServOn(u8 SlaveID)
{ //SlaveID = 0，是否相当于广播
	ptrServ[SlaveID]->CtrlWord = 0x000F;
	return TX_PDO1(SlaveID);
}

u8 SetServRdy(u8 SlaveID)
{ //伺服使能的中间过渡态，伺服使能前需要调用一下
	ptrServ[SlaveID]->CtrlWord = 0x0006;
	return TX_PDO1(SlaveID);
}

u8 SetServOff(u8 SlaveID)
{
	ptrServ[SlaveID]->CtrlWord = 0x0000;
	return TX_PDO1(SlaveID);
}

u8 SetMotorAbsPos(u8 SlaveID, s32 AbsPos)
{
	ptrServ[SlaveID]->CtrlWord = SERV_ABS_POSSET;//SERV_REL_POSSET;//
	ptrServ[SlaveID]->PosSV = AbsPos;
	return TX_PDO2(SlaveID);	
}

u8 SetMotorRelPos(u8 SlaveID, s32 RelPos)
{
	ptrServ[SlaveID]->CtrlWord = SERV_REL_POSSET;//
	ptrServ[SlaveID]->PosSV = RelPos;
	return TX_PDO2(SlaveID);	
}


u8 SetMotorSpd(u8 SlaveID, s32 Spd)
{
	//ptrServ[SlaveID]->CtrlWord = 0x000F;
	ptrServ[SlaveID]->SpdSV = Spd;
	return TX_PDO3(SlaveID);		
}

u8 SetMotorCurrentLimit(u8 SlaveID, s16 MaxCurrent)
{
	//ptrServ[SlaveID]->CtrlWord = 0x000F;
	ptrServ[SlaveID]->MaxCurrenLimit = MaxCurrent;
	return TX_PDO4(SlaveID);	
}



/*
u8 TX_PDO1(u8 SlaveID,u8 *msg)
{
	return CAN1_Send_Frame((PDO1tx + SlaveID),0,8,msg);//	PDO1tx 为 0x180,
}

u8 TX_PDO2(u8 SlaveID,u8 *msg)
{
	return CAN1_Send_Frame((PDO2tx + SlaveID),0,8,msg);	
}

u8 TX_PDO3(u8 SlaveID,u8 *msg)
{
	return CAN1_Send_Frame((PDO3tx + SlaveID),0,8,msg);	
}

u8 TX_PDO4(u8 SlaveID,u8 *msg)
{
	return CAN1_Send_Frame((PDO4tx + SlaveID),0,8,msg);	
}
*/

//4.RPDO,主站读从站数据，用RTR方式，不用主动上报方式，主要原因，总线上挂7个电机比较多，若主动上报，不成功导致重复发送的概率大,而且不是每个时刻，都需要知道每个电机的状态，
u8 RX_PDO1(u8 SlaveID)
{//还需要等待接收,接收放在中断里
	return CAN1_Send_Frame((PDO1tx + SlaveID),1,0,(void*)NULL); //注意,RTR 为 1，请求帧 。PDO1tx 为 0x180,
}

u8 RX_PDO2(u8 SlaveID)
{
	return CAN1_Send_Frame((PDO2tx + SlaveID),1,0,(void*)NULL);	//PDO1rx 为 0x280,
}

u8 RX_PDO3(u8 SlaveID)
{
	return CAN1_Send_Frame((PDO3tx + SlaveID),1,0,(void*)NULL);	//PDO1rx 为 0x380,
}

u8 RX_PDO4(u8 SlaveID)
{
	return CAN1_Send_Frame((PDO4tx + SlaveID),1,0,(void*)NULL);	//PDO1rx 为 0x480,
}



//5.询问状态命令
u8 Rd_NodeGuard(u8 SlaveID)
{
	return CAN1_Send_Frame((NODE_GUARD + SlaveID),1,0,(void*)NULL);//	cob-id 为 0x700 + ID,

}


//6.SDO 命令
//通用 sdo 写
u8 Sdo_WrU8(u8 SlaveID,u16 index,u8 subindex,u32 data) //写1字节
{
	u8 msg[8];	
	msg[0] = 0x2F;//区别在这里
	msg[1] = index&0xff;
  msg[2] = (index>>8)&0xff;
	msg[3] = subindex;
	msg[4] = data & 0xff;	
	msg[5] = msg[6] = msg[7] = 0;
	return CAN1_Send_Frame((SDOrx+SlaveID),0,8,msg);	//SDOrx为0x600,这个主要是对主站来说,所以用0x600 ,从站应答
}

u8 Sdo_WrU16(u8 SlaveID,u16 index,u8 subindex,u32 data) //写2字节
{
	u8 msg[8];	
	msg[0] = 0x2B;//区别在这里
	msg[1] = index&0xff;
  msg[2] = (index>>8)&0xff;
	msg[3] = subindex;
	msg[4] = data&0xff;
	msg[5] = (data>>8)&0xff;
	msg[6] = msg[7] = 0;
	return CAN1_Send_Frame((SDOrx+SlaveID),0,8,msg);	//SDOrx为0x600,这个主要是对主站来说,所以用0x600	
}

u8 Sdo_WrU24(u8 SlaveID,u16 index,u8 subindex,u32 data) //写3字节，实际只用3byte
{
	u8 msg[8];	
	msg[0] = 0x27;//区别在这里
	msg[1] = index&0xff;
  msg[2] = (index>>8)&0xff;
	msg[3] = subindex;
	msg[4] = data&0xff;
	msg[5] = (data>>8)&0xff;
	msg[6] = (data>>16)&0xff;	
	msg[7] = 0;
	//msg[7] = (Dword>>24)&0xff;
	
	return CAN1_Send_Frame((SDOrx+SlaveID),0,8,msg);	//SDOrx为0x600,这个主要是对主站来说,所以用0x600		
}

u8 Sdo_WrU32(u8 SlaveID,u16 index,u8 subindex,u32 data) //写4字节
{
	u8 msg[8];	
	msg[0] = 0x23;//区别在这里
	msg[1] = index&0xff;
  msg[2] = (index>>8)&0xff;
	msg[3] = subindex;
	msg[4] = data&0xff;
	msg[5] = (data>>8)&0xff;
	msg[6] = (data>>16)&0xff;	
	msg[7] = (data>>24)&0xff;	
	return CAN1_Send_Frame((SDOrx+SlaveID),0,8,msg);	//SDOrx为0x600,这个主要是对主站来说,所以用0x600		
}

//SDO 对应的具体配置命令


//u8 

//sdo 读
u8 Sdo_Rd(u8 SlaveID,u16 index,u8 subindex) //读1字节
{ //读数据不按字节分，只需提供地址就可以了.但是从站返回帧就要根据字节来处理
	u8 msg[8];	
	msg[0] = 0x40;//区别在这里
	msg[1] = index&0xff;
  msg[2] = (index>>8)&0xff;
	msg[3] = subindex;
	msg[4] = 0;
	msg[5] = 0;
	msg[6] = 0;	
	msg[7] = 0;	
	return CAN1_Send_Frame((SDOrx + SlaveID),0,8,msg);	//SDOrx为0x600,这个主要是对主站来说,所以用0x600		,接收cob-id为0x580	
}

//SDO 读书节接收处理,数据根据地址译码
void DecodeWrite(u8 SlaveID, u8 cmd, u16 index, u8 subindex) //
{
	;//decode and write global Var
}
void ProcessSDOrx(CanRxMsg *m)
{
	u8 SlaveID = m->StdId & 0x7F;
	u8 cmd_code = m->Data[0];
	u16 index = (m->Data[2]<<8)|m->Data[1];
	u8 subindex = m->Data[3];
	DecodeWrite(SlaveID,cmd_code,index,subindex);
}


//SDO 写应答处
u8 ProcessSDOtx(CanRxMsg *m)
{	//

	//u8 SlaveID = m->StdId & 0xFF;
	u8 cmd_code = m->Data[0];
	if(cmd_code == 0x60)
		return 0; //OK
	else //0x80 
		return 1;//err

}



/*!                                                                                                
**                                                                                                 
**                                                                                                 
** @param d                                                                                        
** @param m                                                                                        
**/  
void canDispatch(CanRxMsg *m)
{	
	u16 cob_id = m->StdId & (~0x007F);//(m->StdId >>7)&0x0F; 
	u16 SlaveId = (m->StdId&0x7F);
	 switch(cob_id)
	{

		case PDO1tx:	//0x180 + //这些为接收 ,//状态，转接，位置		
			ptrServ[SlaveId]->StatusWord = (u16)(m->Data[1]<<8)|m->Data[0];
		  ptrServ[SlaveId]->TrqPV = (s16)((m->Data[3]<<8)|m->Data[2]);
		  ptrServ[SlaveId]->PosPV = (s32)((m->Data[7]<<24)|(m->Data[6]<<16)|(m->Data[5]<<8)|m->Data[4]);
		  OSSemPost(sem_SrvCAN_rx);
			break;
		case PDO2tx://0x280 		
			ptrServ[SlaveId]->StatusWord = (m->Data[1]<<8)|m->Data[0];
		  ptrServ[SlaveId]->TrqPV = (s16)((m->Data[3]<<8)|m->Data[2]);
		  ptrServ[SlaveId]->SpdPV = (s32)((m->Data[7]<<24)|(m->Data[6]<<16)|(m->Data[5]<<8)|m->Data[4]);			
		  OSSemPost(sem_SrvCAN_rx);
			break;
		case PDO3tx:	//0x380	
			ptrServ[SlaveId]->SpdPV = (s32)((m->Data[3]<<24)|m->Data[2]<<16|(m->Data[1]<<8)|m->Data[0]);
		  ptrServ[SlaveId]->PosPV = (s32)((m->Data[7]<<24)|(m->Data[6]<<16)|(m->Data[5]<<8)|m->Data[4]);		
      OSSemPost(sem_SrvCAN_rx);		
			break;
		case PDO4tx:	//0x480
			ptrServ[SlaveId]->StatusWord = (u16)(m->Data[1]<<8)|m->Data[0];
			ptrServ[SlaveId]->ServErr = (u16)((m->Data[3]<<8)|m->Data[2]);
		  ptrServ[SlaveId]->TrqPV = (s16)((m->Data[5]<<8)|m->Data[4]);
		  ptrServ[SlaveId]->CtrlMode = m->Data[6];
			 OSSemPost(sem_SrvCAN_rx);
				break;
			
		case PDO1rx: //0x200		
		case PDO2rx: //0x300
		case PDO3rx: //0x400
		case PDO4rx:	//0x500	
			  // 主站发送给从站的PDO,没返回数据不需要处理
				//proceedPDO(d,m);
			break;
		case SDOtx://0x581,主站接收，从站发送，
			ProcessSDOrx(m);
		  OSSemPost(sem_SrvCAN_rx);
			
		  
		case SDOrx:	//0x600,主站发送，从站接收		
		   ProcessSDOtx(m); //查看应答是否正确，起始需要定义读状态.
		   OSSemPost(sem_SrvCAN_rx);
			break;
		case NODE_GUARD://0x700
				ptrServ[SlaveId]->ServSTA = (m->Data[0])&0x7F; //0,or 0x04(0x84),or 0x05(0x85),or 0x7F(0xFF)
		    OSSemPost(sem_SrvCAN_rx);
				//proceedNODE_GUARD(d,m);
			break;
		default:
			break;
	}
}
	




/*
u8 Sdo_RdU16(u8 SlaveID,u16 index,u8 subindex) //读2字节
{	
}

u8 Sdo_RdU24(u8 SlaveID,u16 index,u8 subindex) //读3字节，实际只用3byte
{	
}

u8 Sdo_RdU32(u8 SlaveID,u16 index,u8 subindex) //读4字节
{	
}
*/

// 7. LSS命令 暂时没用

// 8. TIME_STAMP命令 暂时没用

//接收命令的处理






//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
	
	 // printf("rx data!");
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}

void Wait(u32 time)
{
	u32 i = time;
	while(i!=0)
	{
		i--;
	}
}

void CanInit(void)
{
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,CAN_BAUD_1Mbps_brp,CAN_Mode_Normal); //brp 需设置
		sem_SrvCAN_tx = OSSemCreate(3);
		sem_SrvCAN_rx = OSSemCreate(0);
}



void MotorInit(void)
{
	u32 i,j;

#if 0	
	//可在厂家软件上配置
	NMT_RstNode(0);	//复位所有节点
	Wait(10000);
	NMT_PreSTA(0); //所有节点进入预操作状态
	Wait(10000);
	
	//接下来配置电机工作参数，电机1~4为抱紧电机，
	//电机1
	Sdo_WrU8(1,ModeOperation_Idx,ModeOperation_SubIdx,PPM_MODE); //设置模式为位置PPM模式
	Wait(10000);
	Sdo_WrU32(1,MaxPVelocity_Idx,MaxPVelocity_SubIdx,8000); //设置最大速度
	Wait(10000);
	Sdo_WrU32(1,PAcceleration_Idx,PAcceleration_SubIdx,1000); //设置加速度1000
	Wait(10000);
	Sdo_WrU32(1,PDeceleration_Idx,PDeceleration_SubIdx,1000); //设置减速度1000
	Wait(10000);
	Sdo_WrU32(1,QuickStopDec_Idx,QuickStopDec_SubIdx,3000);//设置紧急减速读为3000
	Wait(10000);
	Sdo_WrU16(1,MotionPType_Idx,MotionPType_SubIdx,0); // 设置运动的方式为linear 非 sin2C
	Wait(10000);
	Sdo_WrU16(1,OutCurrentLimit_Idx,OutCurrentLimit_SubIdx,5000);//	
	Wait(10000);
  	//u8 Sdo_WrU8(u8 SlaveID,u16 index,u8 subindex,u32 data)
	
	//电机2
	//需要看变量是16为或32为或8位的，分别用不同的函数
	Sdo_WrU8(2,ModeOperation_Idx,ModeOperation_SubIdx,PPM_MODE); //设置模式为位置PPM模式
	Wait(10000);
	Sdo_WrU32(2,MaxPVelocity_Idx,MaxPVelocity_SubIdx,8000); //设置最大速度
	Wait(10000);
	Sdo_WrU32(2,PAcceleration_Idx,PAcceleration_SubIdx,1000); //设置加速度1000
	Wait(10000);
	Sdo_WrU32(2,PDeceleration_Idx,PDeceleration_SubIdx,1000); //设置减速度1000
	Wait(10000);
	Sdo_WrU32(2,QuickStopDec_Idx,QuickStopDec_SubIdx,3000); //设置紧急减速读为3000
	Wait(10000);
	Sdo_WrU16(2,MotionPType_Idx,MotionPType_SubIdx,0);     // 设置运动的方式为linear 非 sin2C
	Wait(10000);
	Sdo_WrU16(2,OutCurrentLimit_Idx,OutCurrentLimit_SubIdx,5000);// 设置最大输出电流5000，额定的电流就为5000
	Wait(10000);

	//电机3
	//需要看变量是16为或32为或8位的，分别用不同的函数
	Sdo_WrU8(3,ModeOperation_Idx,ModeOperation_SubIdx,PPM_MODE); //设置模式为位置PPM模式
	Wait(10000);
	Sdo_WrU32(3,MaxPVelocity_Idx,MaxPVelocity_SubIdx,8000); //设置最大速度
	Wait(10000);
	Sdo_WrU32(3,PAcceleration_Idx,PAcceleration_SubIdx,1000); //设置加速度1000
	Wait(10000);
	Sdo_WrU32(3,PDeceleration_Idx,PDeceleration_SubIdx,1000); //设置减速度1000
	Wait(10000);
	Sdo_WrU32(3,QuickStopDec_Idx,QuickStopDec_SubIdx,3000); //设置紧急减速读为3000
	Wait(10000);
	Sdo_WrU16(3,MotionPType_Idx,MotionPType_SubIdx,0);     // 设置运动的方式为linear 非 sin2C
	Wait(10000);
	Sdo_WrU16(3,OutCurrentLimit_Idx,OutCurrentLimit_SubIdx,5000);// 设置最大输出电流5000，额定的电流就为5000
	Wait(10000);

	//电机4
	//需要看变量是16为或32为或8位的，分别用不同的函数
	Sdo_WrU8(4,ModeOperation_Idx,ModeOperation_SubIdx,PPM_MODE); //设置模式为位置PPM模式
	Wait(10000);
	Sdo_WrU32(4,MaxPVelocity_Idx,MaxPVelocity_SubIdx,8000); //设置最大速度
	Wait(10000);
	Sdo_WrU32(4,PAcceleration_Idx,PAcceleration_SubIdx,1000); //设置加速度1000
	Wait(10000);
	Sdo_WrU32(4,PDeceleration_Idx,PDeceleration_SubIdx,1000); //设置减速度1000
	Wait(10000);
	Sdo_WrU32(4,QuickStopDec_Idx,QuickStopDec_SubIdx,3000); //设置紧急减速读为3000
	Wait(10000);
	Sdo_WrU16(4,MotionPType_Idx,MotionPType_SubIdx,0);     // 设置运动的方式为linear 非 sin2C
	Wait(10000);
	Sdo_WrU16(4,OutCurrentLimit_Idx,OutCurrentLimit_SubIdx,5000);// 设置最大输出电流5000，额定的电流就为5000
	Wait(10000);


	//电机5~7为升降电机
	//电机5
	//需要看变量是16为或32为或8位的，分别用不同的函数
	Sdo_WrU8(5,ModeOperation_Idx,ModeOperation_SubIdx,PPM_MODE); //设置模式为位置PPM模式
	Wait(10000);
	Sdo_WrU32(5,MaxPVelocity_Idx,MaxPVelocity_SubIdx,8000); //设置最大速度
	Wait(10000);
	Sdo_WrU32(5,PAcceleration_Idx,PAcceleration_SubIdx,1000); //设置加速度1000
	Wait(10000);
	Sdo_WrU32(5,PDeceleration_Idx,PDeceleration_SubIdx,1000); //设置减速度1000
	Wait(10000);
	Sdo_WrU32(5,QuickStopDec_Idx,QuickStopDec_SubIdx,3000); //设置紧急减速读为3000
	Wait(10000);
	Sdo_WrU16(5,MotionPType_Idx,MotionPType_SubIdx,0);     // 设置运动的方式为linear 非 sin2C
	Wait(10000);
	//Sdo_WrU16(5,OutCurrentLimit_Idx,OutCurrentLimit_SubIdx,15000);// 

	//电机6
	Sdo_WrU8(6,ModeOperation_Idx,ModeOperation_SubIdx,PPM_MODE); //设置模式为位置PPM模式
	Wait(10000);
	Sdo_WrU32(6,MaxPVelocity_Idx,MaxPVelocity_SubIdx,8000); //设置最大速度
	Wait(10000);
	Sdo_WrU32(6,PAcceleration_Idx,PAcceleration_SubIdx,1000); //设置加速度1000
	Wait(10000);
	Sdo_WrU32(6,PDeceleration_Idx,PDeceleration_SubIdx,1000); //设置减速度1000
	Wait(10000);
	Sdo_WrU32(6,QuickStopDec_Idx,QuickStopDec_SubIdx,3000); //设置紧急减速读为3000
	Wait(10000);
	Sdo_WrU16(6,MotionPType_Idx,MotionPType_SubIdx,0);     // 设置运动的方式为linear 非 sin2C
	Wait(10000);
	//Sdo_WrU16(6,OutCurrentLimit_Idx,OutCurrentLimit_SubIdx,15000);// 
	
	//电机7
	Sdo_WrU8(7,ModeOperation_Idx,ModeOperation_SubIdx,PPM_MODE); //设置模式为位置PPM模式
	Wait(10000);
	Sdo_WrU32(7,MaxPVelocity_Idx,MaxPVelocity_SubIdx,8000); //设置最大速度
	Wait(10000);
	Sdo_WrU32(7,PAcceleration_Idx,PAcceleration_SubIdx,1000); //设置加速度1000
	Wait(10000);
	Sdo_WrU32(7,PDeceleration_Idx,PDeceleration_SubIdx,1000); //设置减速度1000
	Wait(10000);
	Sdo_WrU32(7,QuickStopDec_Idx,QuickStopDec_SubIdx,3000); //设置紧急减速读为3000
	Wait(10000);
	Sdo_WrU16(7,MotionPType_Idx,MotionPType_SubIdx,0);     // 设置运动的方式为linear 非 sin2C
	Wait(10000);
	//Sdo_WrU16(7,OutCurrentLimit_Idx,OutCurrentLimit_SubIdx,15000);// 	
#endif

  NMT_Start(0); //启动所有的节点

  Wait(10000);
	
	for(i=1;i<=7;i++)
	{
		u8 err;
		for(j=0;j<5;j++)  //失败最多试5次
		{
			Rd_NodeGuard(i);
			OSSemPend(sem_SrvCAN_rx,0,&err);
			if(ptrServ[i]->ServSTA == Operational)
			{
				//j=100;//直接使其跳出
				break;
			}
		}
		//后面增加初始化失败的寄存器在上面.
	}
	
	  Wait(1000);
		//4个电机同事抱紧。		
		//伺服预势能
		SetMotorCtrlword(1,SERV_ON_PRE);
		SetMotorCtrlword(2,SERV_ON_PRE);
		SetMotorCtrlword(3,SERV_ON_PRE);
		SetMotorCtrlword(4,SERV_ON_PRE);
		SetMotorCtrlword(5,SERV_ON_PRE);
		SetMotorCtrlword(6,SERV_ON_PRE);
		SetMotorCtrlword(7,SERV_ON_PRE);	
		CMD_SYNC();//发送同步，使得发送的数据有效
	
		//伺服势能
		SetMotorCtrlword(1,SERV_ON);
		SetMotorCtrlword(2,SERV_ON);
		SetMotorCtrlword(3,SERV_ON);
		SetMotorCtrlword(4,SERV_ON);
		SetMotorCtrlword(5,SERV_ON);
		SetMotorCtrlword(6,SERV_ON);
		SetMotorCtrlword(7,SERV_ON);			
		CMD_SYNC();//发送同步，使得发送的数据有效				


	
}

void HandleCan(void)
{
	//CanInit();
	//MotorInit();
	u8 msg[8] = {0,1,2,3,4,5,6,7};	
	u16 cob_id = 0;
	u8 slave_Id = 1;
	
	cob_id = Para[5];
	slave_Id = Para[11];
	if(Para[6] !=0)
	{
		switch(Para[6])
		{
			case 1:
				NMT_Start(slave_Id);
				break;
			case 2:
				CMD_SYNC();
				break;
			case 3:
				NMT_PreSTA(slave_Id);
				//Rd_NodeGuard(slave_Id);
				break;
			case 4:
				TX_PDO1(slave_Id);
				break;
			case 5:
				RX_PDO1(slave_Id);				
				break;
			case 6:
				Sdo_WrU32(slave_Id,1000,0,0x55AA1234);
				break;	
			case 7:
				Sdo_WrU24(slave_Id,1000,0,0x55AA1234);
				break;
			case 8:
				Sdo_WrU16(slave_Id,1000,0,0x55AA1234);
				break;
			case 9:
				Sdo_WrU8(slave_Id,1000,0,0x55AA1234);
				break;
			case 10:
				Sdo_Rd(slave_Id,0x6041,0);
				break;
			case 11:
				Rd_STA_TRQ_POS(slave_Id);
				break;
			case 12:
				Rd_STA_TRQ_SPD(slave_Id);
				break;		
			case 13:
				Rd_SPD_POS(slave_Id);
				break;		
			case 14:
				Rd_STA_ERR_TRQ_MODE(slave_Id);
				break;	
			case 15:
				SetMotorCtrlword(slave_Id,Para[7]);
				break;
			case 16:
				SetMotorAbsPos(slave_Id,Para[8]);
				break;		
			case 17:
				SetMotorSpd(slave_Id,Para[9]);
				break;		
			case 18:
				SetMotorCurrentLimit(slave_Id,Para[10]);
				break;			
			case 20:
				TX_PDO2(1);
				TX_PDO2(2);
			  TX_PDO2(3);
				TX_PDO2(4);
				TX_PDO2(5);
			  TX_PDO2(6);
				TX_PDO2(7);
				TX_PDO2(8);
			  TX_PDO2(9);			
				break;
			case 100:
				//u8 CAN1_Send_Frame(u16 CobId,u8 Rtr,u8 Len,u8* msg)
				CAN1_Send_Frame(cob_id,0,8,msg);
				break;
			default:		
				break;
		}
	}
	Para[6] = 0;
}


