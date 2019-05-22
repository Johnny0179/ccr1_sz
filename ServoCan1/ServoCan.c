/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : Servo_can.c
* Author             : 
* Date First Issued  : 27/04/2010
* Description        : UART的硬件配置和modbus协议处理
********************************************************************************/

#if 1 

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "includes.h"
#include "glb_reg.h"
#include "ServoCan.h"


#define CAN_BAUD_500Kbps_brp 6  //波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
#define CAN_BAUD_1Mbps_brp 3

#define POS_MODE 1  //1:位置模式，0速度模式

//Can 指令发送命令 :
u8 TimeNum=0,MotorStopSign=0,MotorStartSign=0;


//NMT命令 ，第2个参视Ω谬是节点ID
u8 PreStateCode[8] = {0x80,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
u8 StartStateCode[8] = {0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
u8 StopStateCode[8] = {0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
u8 ResetNodeCode[8] =  {0x81,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
u8 ResetComCode[8] =  {0x81,0x01,0x00,0x00,0x00,0x00,0x00,0x00};

u8 ModeOperation_POS[8]=			{0x2f,0x60,0x60,0x00,0x01,0x00,0x00,0x00};//设置模式，位置
u8 ModeOperation[8]=			{0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00};//设置模式，速度


u8 MaxPVelocity[8]=			{0x23,0x7f,0x60,0x00,0x40,0x1f,0x00,0x00};//最大速度8000rpm
u8 PAcceleration[8]=			{0x23,0x83,0x60,0x00,0xe8,0x03,0x00,0x00};//加速度1000rpm/s
u8 PDeceleration[8]=			{0x23,0x84,0x60,0x00,0xb8,0x0b,0x00,0x00};//减速度3000rpm/s
u8 QuickStopDeceleration[8]=	{0x23,0x85,0x60,0x00,0xa0,0x0f,0x00,0x00};//紧急减速度4000rpm/s
u8 MotionPType[8]=				{0x2b,0x86,0x60,0x00,0x00,0x00,0x00};

u8 ShutDown[8]=				{0x2b,0x40,0x60,0x00,0x06,0x00,0x00};    //控制字（关）
u8 SwitchOn[8]=				{0x2b,0x40,0x60,0x00,0x0f,0x00,0x00};    //控制字（开）

u8 TargetVelocity[8]=				{0x23,0xff,0x60,0x00,0xe8,0x03,0x00,0x00};//设置速度值1000
u8 TargetPos[8] = {0x23,0x7a,0x60,0x00,0xe8,0x03,0x00,0x00}; //设置位置值，1000
u8 RunMotor[8]=				{0x2b,0x40,0x60,0x00,0x0f,0x00,0x00};		//启动电机
u8 RunMotor_POS[8]=				{0x2b,0x40,0x60,0x00,0x7f,0x00,0x00};		//启动电机相对位置指令
u8 StopMotor[8]=				{0x2b,0x40,0x60,0x00,0x0f,0x01,0x00};		//停止电机
u8 DisMotor[8]=				{0x2b,0x40,0x60,0x00,0x0b,0x00,0x00};		//急停



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
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
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
		
#if CAN1_RX0_INT_ENABLE
	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
	int i=0;
    CAN_Receive(CAN1, 0, &RxMessage);
	printf("StdId:%x\r\n",RxMessage.StdId);
	for(i=0;i<8;i++)
	printf("rxbuf[%d]:%x\r\n",i,RxMessage.Data[i]);
}
#endif

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
#if 0
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x601;	 // 标准标识符为0
  TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
  TxMessage.IDE=CAN_ID_STD;		  // 使用扩展标识符
  TxMessage.RTR=CAN_RTR_DATA;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		

}
#endif


u8 CAN1_Send_Frame(u16 CobId,u8 Rtr,u8 Len,u8* msg)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=CobId;	 // 标准标识符,0x601,0x581
  TxMessage.ExtId= 0;//0x12;	 // 设置扩展标示符（29位）,暂时没用
  TxMessage.IDE=CAN_ID_STD;		  // 标准帧，不使用扩展标识符
  TxMessage.RTR= Rtr;//CAN_RTR_DATA//CAN_RTR_Remote;		  // 消息类型为数据帧，或
  TxMessage.DLC=Len;							 // 发送两帧信息
  for(i=0;i<Len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);    
	i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		

}



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

void CanInit(void)
{
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,CAN_BAUD_1Mbps_brp,CAN_Mode_Normal); //brp 需设置
}

// 应用调用

void MotorInit(void)
{
	//u8 CAN1_Send_Frame(u16 CobId,u8 Rtr,u8 Len,u8* msg);//
	
	CAN1_Send_Frame(0x000,CAN_RTR_DATA,2,ResetNodeCode);
	printf("enter  ResetNodeCode \n");
	
	CAN1_Send_Frame(0x000,CAN_RTR_DATA,2,PreStateCode);
	printf("enter pre state \n");

#if POS_MODE
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,ModeOperation_POS);
	printf("config pos_mode! \n");
#else
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,ModeOperation);
	printf("config speed_mode! \n");
#endif
	
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,MaxPVelocity);
	printf("config MaxPVelocity! \n");
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,PAcceleration);
	printf("config PAcceleration! \n");
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,PDeceleration);
	printf("config PDeceleration! \n");
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,QuickStopDeceleration);
	printf("config QuickStopDeceleration! \n");
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,MotionPType);
	printf("config MotionPType! \n");
	
	CAN1_Send_Frame(0x000,CAN_RTR_DATA,2,StartStateCode);
	printf("start node  state \n");
	
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,ShutDown);
	printf("config ShutDown! \n");
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,SwitchOn);	
	printf("config SwitchOn! \n");
}

void EnableMotor(void)
{
	//CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,MotionPType);
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,ShutDown);
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,SwitchOn);
}

void DisableMotor(void)
{
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,DisMotor);
}


void SwitchOffMotor(void)
{
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,ShutDown);
}


void SetSpeed(s32 spd)
{
	u8 Velocity[8]= {0x23,0xff,0x60,0x00,0x00,0x00,0x00,0x00};
	Velocity[4] = (u8)(spd&0xff);
	Velocity[5] = (u8)((spd>>8)&0xff);
	Velocity[6] = (u8)((spd>>16)&0xff);
	Velocity[7] = (u8)((spd>>24)&0xff);
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,Velocity);
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,RunMotor);		
}

void SetPos(s32 pos)
{
	u8 pos_var[8]= {0x23,0x7a,0x60,0x00,0x00,0x00,0x00,0x00};
	pos_var[4] = (u8)(pos&0xff);
	pos_var[5] = (u8)((pos>>8)&0xff);
	pos_var[6] = (u8)((pos>>16)&0xff);
	pos_var[7] = (u8)((pos>>24)&0xff);
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,pos_var);
	CAN1_Send_Frame(0x601,CAN_RTR_DATA,8,RunMotor_POS);		
}

void HandleCan(void)
{
	//CanInit();
	//MotorInit();

		switch(Para[1])
		{
			case 1:
				SetPos(1000);//发送8个字节 	 
				printf("set pos:1000 !\n");				
				break;
			case 2:
				SetPos(-1000);//发送8个字节 	 
				printf("set pos:-1000 !\n");				
				break;
			case 3:
				SetSpeed(1000);//发送8个字节 	 
				printf("set speed:1000 !\n");			
				break;
			case 4:
				SetSpeed(-1000);//发送8个字节 	 
				printf("set speed:-1000 !\n");				
				break;	
			case 5:
				SwitchOffMotor();
				//DisableMotor();//发送8个字节 	 
				printf("Switch off motor !\n");				
				break;
			case 6:
				EnableMotor();//发送8个字节 	
				printf("Enable motor !\n");						
				break;
			default:
					break;
		}
		Para[1] = 0;	
}


#endif
