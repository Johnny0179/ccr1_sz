/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : Servo_can.c
* Author             : 
* Date First Issued  : 27/04/2010
* Description        : UART��Ӳ�����ú�modbusЭ�鴦��
********************************************************************************/

#if 1 

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "includes.h"
#include "glb_reg.h"
#include "ServoCan.h"


#define CAN_BAUD_500Kbps_brp 6  //������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
#define CAN_BAUD_1Mbps_brp 3

#define POS_MODE 1  //1:λ��ģʽ��0�ٶ�ģʽ

//Can ָ������� :
u8 TimeNum=0,MotorStopSign=0,MotorStartSign=0;


//NMT���� ����2�����Ӧ����ǽڵ�ID
u8 PreStateCode[8] = {0x80,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
u8 StartStateCode[8] = {0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
u8 StopStateCode[8] = {0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
u8 ResetNodeCode[8] =  {0x81,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
u8 ResetComCode[8] =  {0x81,0x01,0x00,0x00,0x00,0x00,0x00,0x00};

u8 ModeOperation_POS[8]=			{0x2f,0x60,0x60,0x00,0x01,0x00,0x00,0x00};//����ģʽ��λ��
u8 ModeOperation[8]=			{0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00};//����ģʽ���ٶ�


u8 MaxPVelocity[8]=			{0x23,0x7f,0x60,0x00,0x40,0x1f,0x00,0x00};//����ٶ�8000rpm
u8 PAcceleration[8]=			{0x23,0x83,0x60,0x00,0xe8,0x03,0x00,0x00};//���ٶ�1000rpm/s
u8 PDeceleration[8]=			{0x23,0x84,0x60,0x00,0xb8,0x0b,0x00,0x00};//���ٶ�3000rpm/s
u8 QuickStopDeceleration[8]=	{0x23,0x85,0x60,0x00,0xa0,0x0f,0x00,0x00};//�������ٶ�4000rpm/s
u8 MotionPType[8]=				{0x2b,0x86,0x60,0x00,0x00,0x00,0x00};

u8 ShutDown[8]=				{0x2b,0x40,0x60,0x00,0x06,0x00,0x00};    //�����֣��أ�
u8 SwitchOn[8]=				{0x2b,0x40,0x60,0x00,0x0f,0x00,0x00};    //�����֣�����

u8 TargetVelocity[8]=				{0x23,0xff,0x60,0x00,0xe8,0x03,0x00,0x00};//�����ٶ�ֵ1000
u8 TargetPos[8] = {0x23,0x7a,0x60,0x00,0xe8,0x03,0x00,0x00}; //����λ��ֵ��1000
u8 RunMotor[8]=				{0x2b,0x40,0x60,0x00,0x0f,0x00,0x00};		//�������
u8 RunMotor_POS[8]=				{0x2b,0x40,0x60,0x00,0x7f,0x00,0x00};		//����������λ��ָ��
u8 StopMotor[8]=				{0x2b,0x40,0x60,0x00,0x0f,0x01,0x00};		//ֹͣ���
u8 DisMotor[8]=				{0x2b,0x40,0x60,0x00,0x0b,0x00,0x00};		//��ͣ



//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024; tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ42M,�������CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//������Ϊ:42M/((6+7+1)*6)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��; 


u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
		
    
		//���ù�����
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN1_RX0_INT_ENABLE
	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN1_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
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

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
#if 0
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x601;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=CAN_ID_STD;		  // ʹ����չ��ʶ��
  TxMessage.RTR=CAN_RTR_DATA;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;
  return 0;		

}
#endif


u8 CAN1_Send_Frame(u16 CobId,u8 Rtr,u8 Len,u8* msg)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=CobId;	 // ��׼��ʶ��,0x601,0x581
  TxMessage.ExtId= 0;//0x12;	 // ������չ��ʾ����29λ��,��ʱû��
  TxMessage.IDE=CAN_ID_STD;		  // ��׼֡����ʹ����չ��ʶ��
  TxMessage.RTR= Rtr;//CAN_RTR_DATA//CAN_RTR_Remote;		  // ��Ϣ����Ϊ����֡����
  TxMessage.DLC=Len;							 // ������֡��Ϣ
  for(i=0;i<Len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);    
	i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;
  return 0;		

}



//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
	
	 // printf("rx data!");
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}

void CanInit(void)
{
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,CAN_BAUD_1Mbps_brp,CAN_Mode_Normal); //brp ������
}

// Ӧ�õ���

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
				SetPos(1000);//����8���ֽ� 	 
				printf("set pos:1000 !\n");				
				break;
			case 2:
				SetPos(-1000);//����8���ֽ� 	 
				printf("set pos:-1000 !\n");				
				break;
			case 3:
				SetSpeed(1000);//����8���ֽ� 	 
				printf("set speed:1000 !\n");			
				break;
			case 4:
				SetSpeed(-1000);//����8���ֽ� 	 
				printf("set speed:-1000 !\n");				
				break;	
			case 5:
				SwitchOffMotor();
				//DisableMotor();//����8���ֽ� 	 
				printf("Switch off motor !\n");				
				break;
			case 6:
				EnableMotor();//����8���ֽ� 	
				printf("Enable motor !\n");						
				break;
			default:
					break;
		}
		Para[1] = 0;	
}


#endif
