/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : rs485.c
* Author             : 
* Date First Issued  : 27/04/2010
* Description        : UART的硬件配置和modbus协议处理
********************************************************************************/

#if 0

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "includes.h"
#include "glb_reg.h"
#include "ServoCom.h"


OS_EVENT *sem_SrvCom_rx;
OS_EVENT *sem_SrvCom_tx;

volatile u16 ComTaskTableIndex = 0;

COM2TxRxTYPE Com2TxRx;
#define LOCAL_PAR_BASEADDR 50

#define POS_SET_EN_ADDR (LOCAL_PAR_BASEADDR + 6)
#define TRQ_EN_ADDR_BASE (LOCAL_PAR_BASEADDR + 0) //使能开始地址
#define POS_SET_ADDR_BASE (LOCAL_PAR_BASEADDR + 8)
#define POS_FB_ADDR_BASE 	(LOCAL_PAR_BASEADDR + 24)

#define SRVCmdRD 0x02
#define SRVCmdWR 0x03
#define ComTaskTableIndexMax	18  //下面是18条读写任务
const ComTaskTYPE ComTaskTable[] = {
//cmd,	localpar_index	,			axis_id,	srvpar_addr,	dat_en,		
SRVCmdWR,	LOCAL_PAR_BASEADDR + 0,		1,			562,				1,	//1轴转矩使能
SRVCmdWR,	LOCAL_PAR_BASEADDR + 1,		2,			562,				1,	
SRVCmdWR,	LOCAL_PAR_BASEADDR + 2,		3,			562,				1,	
SRVCmdWR,	LOCAL_PAR_BASEADDR + 3,		4,			562,				1,		
SRVCmdWR,	LOCAL_PAR_BASEADDR + 4,		5,			562,				1,
SRVCmdWR,	LOCAL_PAR_BASEADDR + 5,		6,			562,				1, 	//6轴转矩使能
SRVCmdWR,	LOCAL_PAR_BASEADDR + 8,		1,			596,				4,	//1轴伺服位置设定
SRVCmdWR,	LOCAL_PAR_BASEADDR + 10,	2,			596,				4,	//2轴伺服位置设定
SRVCmdWR,	LOCAL_PAR_BASEADDR + 12,	3,			596,				4,	//3轴伺服位置设定
SRVCmdWR,	LOCAL_PAR_BASEADDR + 14,	4,			596,				4,	//4轴伺服位置设定
SRVCmdWR,	LOCAL_PAR_BASEADDR + 16,	5,			596,				4,	//5轴伺服位置设定
SRVCmdWR,	LOCAL_PAR_BASEADDR + 18,	6,			596,				4,	//6轴伺服位置设定
SRVCmdRD,	LOCAL_PAR_BASEADDR + 24,	1,			611,				4,	//1轴伺服位置状态
SRVCmdRD,	LOCAL_PAR_BASEADDR + 26,	2,			611,				4,	//1轴伺服位置状态
SRVCmdRD,	LOCAL_PAR_BASEADDR + 28,	3,			611,				4,	//1轴伺服位置状态
SRVCmdRD,	LOCAL_PAR_BASEADDR + 30,	4,			611,				4,	//1轴伺服位置状态
SRVCmdRD,	LOCAL_PAR_BASEADDR + 32,	5,			611,				4,	//1轴伺服位置状态
SRVCmdRD,	LOCAL_PAR_BASEADDR + 34,	6,			611,				4,	//1轴伺服位置状态
};


 const uint16_t crc_table[256] = { 0x0000,
    0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
    0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
    0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
    0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
    0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
    0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
    0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
    0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
    0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
    0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
    0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
    0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
    0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
    0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
    0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
    0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
    0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
    0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
    0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
    0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
    0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
    0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
    0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
    0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
    0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
    0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
    0x820D, 0x8207, 0x0202 };


//初始化IO 串口2
//bound:波特率	  
void USART2_Init(u32 bound)
{  	 
	
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //使能GPIOG时钟,PG8
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
	
  //串口2引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
	
	//USART2    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2与GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA2，PA3
	
	//PG8推挽输出，485模式控制  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //GPIOG8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure); //初始化PG8
	

   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
  USART_Cmd(USART2, ENABLE);  //使能串口 2
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
#if 1	
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);// 放在发送完指令时才使能接收中断
	USART_ITConfig(USART2, USART_IT_TC, ENABLE);//发送中断

	//Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif	
	
	RS485_TX_EN=0;				//默认为接收模式	
}

/*
typedef char                int8_t;
typedef short int           int16_t;
typedef int                 int32_t;

typedef unsigned char       uint8_t;
typedef unsigned short int  uint16_t;
typedef unsigned int        uint32_t;
*/

//unsigned short updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
  uint16_t i, j;
		
  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
 
  return crc_accum;
	
}


void ServoCom_Init(void)
{
	
	sem_SrvCom_tx= OSSemCreate(0);
	sem_SrvCom_rx= OSSemCreate(0);
	Com2TxRx.tx_cnt = 0;
	Com2TxRx.tx_end = 0;
	Com2TxRx.rx_cnt = 0;
	Com2TxRx.rx_end = 0;
	Com2TxRx.err = 0;
	Com2TxRx.state = SrvCOM_IDLE;
	Com2TxRx.tx_flag = 0;
	Com2TxRx.timeout = 0;
	Com2TxRx.timeout_en = 0;
	
	ComTaskTableIndex = 0;

	//ComTask 初始化
	USART2_Init(115200);	
}


// 读发送包打包封装
void PackFrameRd(void)
{
	#define PACK_LEN (u16)7 
	
	u16 crc_len;
	u16 crc ;
	
	Com2TxRx.tx[0] = 0xff;
	Com2TxRx.tx[1] = 0xff;
	Com2TxRx.tx[2] = 0xfd;
	Com2TxRx.tx[3] = 0x00;
	Com2TxRx.tx[4] = ComTaskTable[ComTaskTableIndex].axis_id;
	Com2TxRx.tx[5] = (u8)(PACK_LEN&0xff); //写命令帧的长度都为7 = cmd(1) + addr(2)+len(2)+ crc(2) ,返回的长度 = ComTaskTable[ComTaskTableIndex].dat_len + 4
	Com2TxRx.tx[6] = (u8)((PACK_LEN>>8)&0xff);
	Com2TxRx.tx[7] = ComTaskTable[ComTaskTableIndex].cmd;
	Com2TxRx.tx[8] = (u8)(ComTaskTable[ComTaskTableIndex].srvpar_addr&0xff);
	Com2TxRx.tx[9] = (u8)((ComTaskTable[ComTaskTableIndex].srvpar_addr>>8)&0xff);
	Com2TxRx.tx[10] = (u8)(ComTaskTable[ComTaskTableIndex].dat_len&0xff);
	Com2TxRx.tx[11] = (u8)((ComTaskTable[ComTaskTableIndex].dat_len>>8)&0xff);	
	crc_len = ((Com2TxRx.tx[6]<<8)+Com2TxRx.tx[5]) + 5;	
	crc = updateCRC(0,(u8 *)(&Com2TxRx.tx[0]),crc_len);
	Com2TxRx.tx[12] = (u8)(crc&0xff);
	Com2TxRx.tx[13] = (u8)((crc>>8)&0xff);		
	
	Com2TxRx.tx_cnt = 0;
	Com2TxRx.tx_end = crc_len + 2;
	Com2TxRx.tx_flag = 1;
	Com2TxRx.err = 0;	
	
}



//写命令发送打包
void PackFrameWr(void)
{


	u8 * ptr;
	u16 crc_len;
	u16 crc ;	
	u16 i;
	
	Com2TxRx.tx[0] = 0xff;
	Com2TxRx.tx[1] = 0xff;
	Com2TxRx.tx[2] = 0xfd;
	Com2TxRx.tx[3] = 0x00;
	Com2TxRx.tx[4] = ComTaskTable[ComTaskTableIndex].axis_id;
	Com2TxRx.tx[5] = (u8)((ComTaskTable[ComTaskTableIndex].dat_len + 5)&0xff); //packet_len L
	Com2TxRx.tx[6] = (u8)(((ComTaskTable[ComTaskTableIndex].dat_len + 5)>>8)&0xff);
	Com2TxRx.tx[7] = ComTaskTable[ComTaskTableIndex].cmd;  //cmd_id
	Com2TxRx.tx[8] = (u8)(ComTaskTable[ComTaskTableIndex].srvpar_addr&0xff);  //servo_par_addr
	Com2TxRx.tx[9] = (u8)((ComTaskTable[ComTaskTableIndex].srvpar_addr>>8)&0xff);
	
	ptr = (u8 *)(&Para[ComTaskTable[ComTaskTableIndex].localpar_index]);
	for(i=0;i<ComTaskTable[ComTaskTableIndex].dat_len;i++)
	{
		Com2TxRx.tx[10 + i] = *ptr++;
	}
	
	crc_len = ((Com2TxRx.tx[6]<<8)+Com2TxRx.tx[5]) + 5;	
	crc = updateCRC(0,(u8 *)(&Com2TxRx.tx[0]),crc_len);
	
	Com2TxRx.tx[crc_len] = (u8)(crc&0xff);  //crc
	Com2TxRx.tx[crc_len + 1] = (u8)((crc>>8)&0xff);		
	
	Com2TxRx.tx_cnt = 0;
	Com2TxRx.tx_end = crc_len + 2;
	Com2TxRx.tx_flag = 1;
	Com2TxRx.err = 0;	

}


void PackFrame(void)
{
	//printf("index,cmd:%d,%x\n",ComTaskTableIndex,ComTaskTable[ComTaskTableIndex].cmd);
	switch(ComTaskTable[ComTaskTableIndex].cmd)
	{
		case SRVCmdWR:
			PackFrameWr();
			break;
		case SRVCmdRD:
			//printf("pck_rd\n");
			PackFrameRd();
			break;		
		default:
			break;		
	}
}


//读回数据，解析包
void UnPackFrameRd(void)
{
	u16 i;
	u16 err = 0;
	u16 Packet_len = 0;
	u16 crc = 0;
	u8 * ptr; 
	//判断帧头
	if((Com2TxRx.rx[0] != 0xff)||(Com2TxRx.rx[1] != 0xff)||(Com2TxRx.rx[2] != 0xfd))
	{
		err |= 0x80;
	}	
	Packet_len = (Com2TxRx.rx[6]<<8) + Com2TxRx.rx[5];
  crc = (Com2TxRx.rx[Packet_len +6]<<8) + Com2TxRx.rx[Packet_len +5]; //从数据包取出校验码
	if(updateCRC(0,(u8 *)(&Com2TxRx.rx[0]),(Packet_len+5)) != crc)   //crc = updateCRC(0,(u8 *)(&Com2TxRx.rx[0]),(Packet_len+5));
	{ //校验码错
		err |= 0x40; 
	}
	
	if(err ==0)
	{
		ptr = (u8 *)(&Para[ComTaskTable[ComTaskTableIndex].localpar_index]);
		for(i=0;i< ComTaskTable[ComTaskTableIndex].dat_len;i++)
		{
			*ptr++ = Com2TxRx.rx[9 + i];
		}
	}		
	
	Com2TxRx.err = err;	
	Para[LOCAL_PAR_BASEADDR + 21] = err;
	
	for(i=0;i<8;i++)
	{
		Com2TxRx.rx[i] = 0; //接收缓冲区清0
	}
	
	
}

//写数据 返回包，解析
void UnPackFrameWr(void)
{
	u16 i;
	u16 err = 0;
	u16 Packet_len = 0;
	u16 crc = 0;
	
	if((Com2TxRx.rx[0] != 0xff)||(Com2TxRx.rx[1] != 0xff)||(Com2TxRx.rx[2] != 0xfd))
	{//判断帧头
		err |= 0x80;
	}	
	Packet_len = (Com2TxRx.rx[6]<<8) + Com2TxRx.rx[5];
  crc = (Com2TxRx.rx[Packet_len +6]<<8) + Com2TxRx.rx[Packet_len +5]; //从数据包取出校验码
	
	if(updateCRC(0,(u8 *)(&Com2TxRx.rx[0]),(Packet_len+5)) != crc)   //crc = updateCRC(0,(u8 *)(&Com2TxRx.rx[0]),(Packet_len+5));
	{ //校验码错
		err |= 0x40; 
	}
	
	if((Com2TxRx.rx[Packet_len + 4] & 0x80) != 0)  //可以根据此设置重发机制
	{
		err |= (Com2TxRx.rx[Packet_len + 4] & 0x7F);
	}
	Com2TxRx.err = err;
	
	Para[LOCAL_PAR_BASEADDR + 20] = err;
	
	for(i=0;i<8;i++)
	{
		Com2TxRx.rx[i] = 0; //接收缓冲区清0
	}	

}

void UnPackFrame(void)
{
	//printf("index,cmd:%d,%x\n",ComTaskTableIndex,ComTaskTable[ComTaskTableIndex].cmd);
	switch(ComTaskTable[ComTaskTableIndex].cmd)
	{
		case SRVCmdWR:
			  UnPackFrameWr();				
			break;
		case SRVCmdRD:
			  //printf("UnPckRd\n");
			  UnPackFrameRd();
			break;		
		default:
			break;		
	}
}

void LoadFirstPosSet(void)
{
	//static u16 TrqEnLast[6] = {0,0,0,0,0,0};
	u16 i;

	if(Para[POS_SET_EN_ADDR] !=0)
	{
		for(i=0;i<6;i++)
		{				
			*((u32 *)(&Para[POS_SET_ADDR_BASE]) + i) = *((u32 *)(&Para[POS_FB_ADDR_BASE]) + i);			
		}	
	}
	/*
	for(i=0;i<6;i++)
	{
		if(Para[TRQ_EN_ADDR_BASE + i] != TrqEnLast[i])
		{			
			TrqEnLast[i] = Para[TRQ_EN_ADDR_BASE + i]; //复制
			*((u32 *)(&Para[POS_SET_ADDR_BASE]) + i) = *((u32 *)(&Para[POS_FB_ADDR_BASE]) + i);			
		}
	}
	*/
	
}
#define POS_SET_ADDR_BASE (LOCAL_PAR_BASEADDR + 8)
#define POS_FB_ADDR_BASE 	(LOCAL_PAR_BASEADDR + 24)

void ServoComRW(void)
{
	u8 err;
	
	//printf("s:%d \n",Com2TxRx.state);
	switch(Com2TxRx.state)
	{
		case SrvCOM_IDLE:
			    LoadFirstPosSet();
					Com2TxRx.state = SrvCOM_TX_FRAME;
		     // ComTaskTableIndex = 0;// for test
			break;
		case SrvCOM_TX_FRAME:		
				 RS485_TX_EN = 1; //至发送使能
			   PackFrame();
		     //PackFrameRd();
		     
		     Com2TxRx.state = SrvCOM_TX;
				 if(Com2TxRx.tx_cnt < Com2TxRx.tx_end)
				 {				
						USART2->DR = Com2TxRx.tx[Com2TxRx.tx_cnt++];  //触发第一次写
				 }	
				 
			break;
		case SrvCOM_TX:			
			OSSemPend(sem_SrvCom_tx,0,&err);
		  Com2TxRx.state = SrvCOM_RX;
		  //delete
			//RS485_TX_EN = 0; //还是放在中断里及时一些
			break;
		case SrvCOM_RX:
			OSSemPend(sem_SrvCom_rx,0,&err);
      Com2TxRx.state = SrvCOM_RX_FRAME;		
			break;
		case SrvCOM_RX_FRAME:
			 UnPackFrame();//
		   //UnPackFrameRd();
		   
		  ComTaskTableIndex ++; 
		   if(ComTaskTableIndexMax == ComTaskTableIndex)
			 {
					ComTaskTableIndex = 0;
			 }			 
			 Com2TxRx.state = SrvCOM_IDLE;
			break;
		default:
			break;
	}
}

void USART2_IRQHandler(void)
{	
	 
	
	OSIntEnter();     ////如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//接收到数据
	{	 	
		u8 res;	
	  res =USART_ReceiveData(USART2);//;读取接收到的数据USART2->DR		
		if(Com2TxRx.tx_flag == 0)
		{
				Com2TxRx.rx[Com2TxRx.rx_cnt++] = res;      			
				Com2TxRx.timeout = RX_TIMEOUT;
		
			Com2TxRx.timeout_en = 1;
			
			//TIM_ClearITPendingBit(TIM3,TIM_IT_Update);//
      //TIM_SetCounter(TIM3,0);//
			//TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
      //TIM_Cmd(TIM3,ENABLE);//
		}		
	}  	
	if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)
	{							
		if(Com2TxRx.tx_flag !=0)
		{		
			if(Com2TxRx.tx_cnt < Com2TxRx.tx_end)
			{				
				USART2->DR = Com2TxRx.tx[Com2TxRx.tx_cnt++];  //触发第一次写
			}
			else
			{
				OSSemPost(sem_SrvCom_tx);
				Com2TxRx.rx_cnt = 0;
				Com2TxRx.rx_end = 250;// 这个10 不准，要在接收到长度字节时修正
				Com2TxRx.tx_flag = 0;
				Com2TxRx.err = 0;		
				Com2TxRx.timeout = 5000;//RX_TIMEOUT;
				Com2TxRx.timeout_en = 1;
				USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
				RS485_TX_EN = 0;				
			}
		}
		USART_ClearITPendingBit(USART2, USART_IT_TC);
	}	
	
	OSIntExit();     ////如果SYSTEM_SUPPORT_OS为真，则需要支持OS.	
}

#endif



