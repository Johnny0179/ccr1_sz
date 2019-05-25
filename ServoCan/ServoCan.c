#include "ServoCan.h"
#include "delay.h"
#include "glb_reg.h"
#include "includes.h"
#include "sys.h"
#include "usart.h"

#define CAN_BAUD_500Kbps_brp \
  6  //������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
#define CAN_BAUD_1Mbps_brp 3

//�ź���,����SDO��RPDO���ź���
OS_EVENT *sem_SrvCAN_rx;
OS_EVENT *sem_SrvCAN_tx;

/*
//Can ָ������� :
//�����ֵ���������
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

#define MotionPType_Idx (u16)0x6086  //linear or Sin2C �Ӽ��ٷ�ʽ
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

// CAN��ʼ��
// tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
// tbs2:ʱ���?2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
// tbs1:ʱ���?1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
// brp :�����ʷ�Ƶ��.��Χ:1~1024; tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
// mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
// Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ42M,�������CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//������Ϊ:42M/((6+7+1)*6)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��;

u8 CAN1_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode) {
  GPIO_InitTypeDef GPIO_InitStructure;
  CAN_InitTypeDef CAN_InitStructure;
  CAN_FilterInitTypeDef CAN_FilterInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  //ʹ�����ʱ��?
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //ʹ��PORTAʱ��

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);  //ʹ��CAN1ʱ��

  //��ʼ��GPIO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //���ù���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //�������?
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // 100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
  GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA11,PA12

  //���Ÿ���ӳ������
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11,
                   GPIO_AF_CAN1);  // GPIOA11����ΪCAN1
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12,
                   GPIO_AF_CAN1);  // GPIOA12����ΪCAN1

  // CAN��Ԫ����
  CAN_InitStructure.CAN_TTCM = DISABLE;  //��ʱ�䴥��ͨ��ģʽ
  CAN_InitStructure.CAN_ABOM =
      DISABLE;  //�����Զ����߹���
  CAN_InitStructure.CAN_AWUM =
      DISABLE;  //˯��ģʽͨ����������(���CAN->MCR��SLEEPλ)
  CAN_InitStructure.CAN_NART =
      ENABLE;  //��ֹ�����Զ�����
  CAN_InitStructure.CAN_RFLM =
      DISABLE;  //���Ĳ�����,�µĸ��Ǿɵ�
  CAN_InitStructure.CAN_TXFP =
      DISABLE;  //���ȼ��ɱ��ı�ʶ������
  CAN_InitStructure.CAN_Mode = mode;  //ģʽ����
  CAN_InitStructure.CAN_SJW =
      tsjw;  //����ͬ����Ծ����(Tsjw)Ϊtsjw+1��ʱ�䵥λ
             // CAN_SJW_1tq~CAN_SJW_4tq
  CAN_InitStructure.CAN_BS1 = tbs1;  // Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  CAN_InitStructure.CAN_BS2 =
      tbs2;  // Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  CAN_InitStructure.CAN_Prescaler = brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1
  CAN_Init(CAN1, &CAN_InitStructure);     // ��ʼ��CAN1

  //���ù�����
  CAN_FilterInitStructure.CAN_FilterNumber = 0;  //������0
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;  // 32λ
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;                ////32λID
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;  // 32λMASK
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment =
      CAN_Filter_FIFO0;  //������0������FIFO0
  CAN_FilterInitStructure.CAN_FilterActivation =
      ENABLE;  //���������?0
  CAN_FilterInit(&CAN_FilterInitStructure);  //�˲�����ʼ��

  CAN_ITConfig(CAN1, CAN_IT_FMP0,
               ENABLE);  // FIFO0��Ϣ�Һ��ж�����.
  // �����п�����һ�·�������Ϊ�յ��жϡ�
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
      1;  // �����ȼ�Ϊ1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  // �����ȼ�Ϊ0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

#if 1
  CAN_ITConfig(CAN1, CAN_IT_TME,
               ENABLE);  // FIFO0��Ϣ�Һ��ж�����.
  // �����п�����һ�·�������Ϊ�յ��жϡ�
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
      1;  // �����ȼ�Ϊ1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  // �����ȼ�Ϊ0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif

  return 0;
}

//�жϷ�����
void CAN1_RX0_IRQHandler(void) {
  CanRxMsg RxMessage;
  OSIntEnter();  ////���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.

  if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
    CAN_Receive(CAN1, 0, &RxMessage);
    canDispatch(&RxMessage);
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
  }

  OSIntExit();  ////���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
}

#if 1
void CAN1_TX_IRQHandler(void) {
  OSIntEnter();  ////���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
  if (CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET) {
    OSSemPost(sem_SrvCAN_tx);
    CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
  }

  OSIntExit();  ////���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
}
#endif

// printf("StdId:%x\r\n",RxMessage.StdId);
//	for(i=0;i<8;i++)
//	printf("rxbuf[%d]:%x\r\n",i,RxMessage.Data[i]);
// can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)
// len:���ݳ���(����?8)
// msg:����ָ��,����?8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
//���ȷ���
u8 CAN1_Send_Frame(u16 CobId, u8 Rtr, u8 Len, u8 *msg) {
  u8 mbox;
  u8 err;
  u16 i;
  CanTxMsg TxMessage;

  TxMessage.StdId = CobId;  // ��׼��ʶ��,0x601,0x581
  TxMessage.ExtId =
      0;  // 0x12;	 //
          // ������չ��ʾ����29λ��,��ʱû��
  TxMessage.IDE =
      CAN_ID_STD;  // ��׼֡����ʹ����չ��ʶ��

  if (Rtr == 0)
    TxMessage.RTR =
        CAN_RTR_DATA;  ////;		  //
                       ///��Ϣ����Ϊ����֡����
  else
    TxMessage.RTR = CAN_RTR_Remote;  // RTR ����֡

  TxMessage.DLC = Len;  // ������֡��Ϣ
  for (i = 0; i < Len; i++) TxMessage.Data[i] = msg[i];  // ��һ֡��Ϣ

  OSSemPend(sem_SrvCAN_tx, 0, &err);  //�����ź�����Դ

  mbox = CAN_Transmit(CAN1, &TxMessage);
#if 1
  i = 0;
  while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
    i++;  //�ȴ����ͽ���
  if (i >= 0XFFF) return 1;
  return 0;
#else
  if (mbox == CAN_TxStatus_NoMailBox)
    return 1;  // err //�����������Զ��ش�
  else
    return 0;  // OK

#endif
}

// 1.NMT����
u8 NMT_Start(u8 SlaveID) {
  u8 msg[8];
  msg[0] = NMT_Start_Node;
  msg[1] = SlaveID;
  return CAN1_Send_Frame(NMT, 0, 2, msg);
}

u8 NMT_Stop(u8 SlaveID) {
  u8 msg[8];
  msg[0] = NMT_Stop_Node;
  msg[1] = SlaveID;
  return CAN1_Send_Frame(NMT, 0, 2, msg);
}

u8 NMT_PreSTA(u8 SlaveID) {
  u8 msg[8];
  msg[0] = NMT_Enter_PreOperational;
  msg[1] = SlaveID;
  return CAN1_Send_Frame(NMT, 0, 2, msg);
}

u8 NMT_RstNode(u8 SlaveID) {
  u8 msg[8];
  msg[0] = NMT_Reset_Node;
  msg[1] = SlaveID;
  return CAN1_Send_Frame(NMT, 0, 2, msg);
}

u8 NMT_RstComm(u8 SlaveID) {
  u8 msg[8];
  msg[0] = NMT_Reset_Comunication;
  msg[1] = SlaveID;
  return CAN1_Send_Frame(NMT, 0, 2, msg);
}

// 2.ͬ������
u8 CMD_SYNC(void) { return CAN1_Send_Frame((u16)SYNC, 0, 0, (void *)NULL); }

// 3.TPDO ����. ��վ�������ݸ���վ
u8 TX_PDO1(u8 SlaveID) {
  //������CtrlWord
  u8 msg[8];
  msg[0] = (ptrServ[SlaveID]->CtrlWord) & 0xff;
  msg[1] = (ptrServ[SlaveID]->CtrlWord >> 8) & 0xff;
  return CAN1_Send_Frame((PDO1rx + SlaveID), 0, 2, msg);  //	PDO1Rx Ϊ 0x200,
}

u8 TX_PDO2(u8 SlaveID) {
  //������CtrlWord��λ��POS_Set
  u8 msg[8];
  msg[0] = (ptrServ[SlaveID]->CtrlWord) & 0xff;
  msg[1] = (ptrServ[SlaveID]->CtrlWord >> 8) & 0xff;

  msg[2] = (ptrServ[SlaveID]->PosSV) & 0xff;
  msg[3] = (ptrServ[SlaveID]->PosSV >> 8) & 0xff;
  msg[4] = (ptrServ[SlaveID]->PosSV >> 16) & 0xff;
  msg[5] = (ptrServ[SlaveID]->PosSV >> 24) & 0xff;

  return CAN1_Send_Frame((PDO2rx + SlaveID), 0, 6, msg);
}

u8 TX_PDO3(u8 SlaveID) {
  //�ٶ�
  u8 msg[8];
  msg[0] = (ptrServ[SlaveID]->SpdSV) & 0xff;
  msg[1] = (ptrServ[SlaveID]->SpdSV >> 8) & 0xff;
  msg[2] = (ptrServ[SlaveID]->SpdSV >> 16) & 0xff;
  msg[3] = (ptrServ[SlaveID]->SpdSV >> 24) & 0xff;

  return CAN1_Send_Frame((PDO3rx + SlaveID), 0, 4, msg);
}

u8 TX_PDO4(u8 SlaveID) {
  //����������
  u8 msg[8];
  // msg[0] = (ptrServ[SlaveID]->CtrlWord)&0xff;
  //	msg[1] = (ptrServ[SlaveID]->CtrlWord >>8)&0xff;

  // MaxCurrenLimit Ϊ16bit
  msg[0] = (ptrServ[SlaveID]->MaxCurrenLimit) & 0xff;
  msg[1] = (ptrServ[SlaveID]->MaxCurrenLimit >> 8) & 0xff;
  msg[2] = 0;  //(ptrServ[SlaveID]->MaxCurrenLimit >>16)&0xff;
  msg[3] = 0;  //(ptrServ[SlaveID]->MaxCurrenLimit >>24)&0xff;

  return CAN1_Send_Frame((PDO4rx + SlaveID), 0, 4, msg);
}

u8 SetMotorCtrlword(u8 SlaveID, u16 Ctrlword) {
  ptrServ[SlaveID]->CtrlWord = Ctrlword;
  return TX_PDO1(SlaveID);
}

void StartMotor(u8 SlaveID) {
  //		NMT_Start(0);
  //		delay_us(TIME_INTERVAL_US);
  //		delay_us(TIME_INTERVAL_US);
  //		delay_us(TIME_INTERVAL_US);
  SetMotorCtrlword(SlaveID, 0x0006);
  delay_us(TIME_INTERVAL_US);
  SetMotorCtrlword(SlaveID, 0x000F);
}

void StopMotor(u8 SlaveID) {
  SetMotorCtrlword(SlaveID, 0x0000);
  // delay_us(TIME_INTERVAL_US);
  //	delay_us(TIME_INTERVAL_US);
  //	NMT_PreSTA(0);
}

u8 SetServOn(u8 SlaveID) {  // SlaveID = 0���Ƿ��൱�ڹ㲥
  ptrServ[SlaveID]->CtrlWord = 0x000F;
  return TX_PDO1(SlaveID);
}

u8 SetServRdy(
    u8 SlaveID) {  //�ŷ�ʹ�ܵ��м�����?���ŷ�ʹ��ǰ��Ҫ����һ��
  ptrServ[SlaveID]->CtrlWord = 0x0006;
  return TX_PDO1(SlaveID);
}

u8 SetServOff(u8 SlaveID) {
  ptrServ[SlaveID]->CtrlWord = 0x0000;
  return TX_PDO1(SlaveID);
}

u8 SetMotorAbsPos(u8 SlaveID, s32 AbsPos) {
  ptrServ[SlaveID]->CtrlWord = SERV_ABS_POSSET;  // SERV_REL_POSSET;//
  ptrServ[SlaveID]->PosSV = AbsPos;
  return TX_PDO2(SlaveID);
}

u8 SetMotorRelPos(u8 SlaveID, s32 RelPos) {
  ptrServ[SlaveID]->CtrlWord = SERV_REL_POSSET;  //
  ptrServ[SlaveID]->PosSV = RelPos;
  return TX_PDO2(SlaveID);
}

u8 SetMotorSpd(u8 SlaveID, s32 Spd) {
  // ptrServ[SlaveID]->CtrlWord = 0x000F;
  ptrServ[SlaveID]->SpdSV = Spd;
  return TX_PDO3(SlaveID);
}

u8 SetMotorCurrentLimit(u8 SlaveID, s16 MaxCurrent) {
  // ptrServ[SlaveID]->CtrlWord = 0x000F;
  ptrServ[SlaveID]->MaxCurrenLimit = MaxCurrent;
  return TX_PDO4(SlaveID);
}

/*
u8 TX_PDO1(u8 SlaveID,u8 *msg)
{
        return CAN1_Send_Frame((PDO1tx + SlaveID),0,8,msg);//	PDO1tx Ϊ 0x180,
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

// 4.RPDO,��վ����վ���ݣ���RTR��ʽ�����������ϱ���ʽ����Ҫԭ�������Ϲ�7������Ƚ϶�?�������ϱ������ɹ������ظ����͵ĸ��ʴ�,���Ҳ���ÿ��ʱ�̣�����Ҫ֪��ÿ�������״�?��
u8 RX_PDO1(
    u8 SlaveID) {  //����Ҫ�ȴ�����,���շ����ж���
  return CAN1_Send_Frame(
      (PDO1tx + SlaveID), 1, 0,
      (void *)
          NULL);  //ע��,RTR Ϊ 1������֡ ��PDO1tx Ϊ 0x180,
}

u8 RX_PDO2(u8 SlaveID) {
  return CAN1_Send_Frame((PDO2tx + SlaveID), 1, 0,
                         (void *)NULL);  // PDO1rx Ϊ 0x280,
}

u8 RX_PDO3(u8 SlaveID) {
  return CAN1_Send_Frame((PDO3tx + SlaveID), 1, 0,
                         (void *)NULL);  // PDO1rx Ϊ 0x380,
}

u8 RX_PDO4(u8 SlaveID) {
  return CAN1_Send_Frame((PDO4tx + SlaveID), 1, 0,
                         (void *)NULL);  // PDO1rx Ϊ 0x480,
}

// 5.ѯ��״̬����
u8 Rd_NodeGuard(u8 SlaveID) {
  return CAN1_Send_Frame((NODE_GUARD + SlaveID), 1, 0,
                         (void *)NULL);  //	cob-id Ϊ 0x700 + ID,
}

// 6.SDO ����
//ͨ�� sdo д
u8 Sdo_WrU8(u8 SlaveID, u16 index, u8 subindex, u32 data)  //д1�ֽ�
{
  u8 msg[8];
  msg[0] = 0x2F;  //����������
  msg[1] = index & 0xff;
  msg[2] = (index >> 8) & 0xff;
  msg[3] = subindex;
  msg[4] = data & 0xff;
  msg[5] = msg[6] = msg[7] = 0;
  // SDOrxΪ0x600,������?�Ƕ���վ��˵,������0x600
  // ,��վӦ��
  return CAN1_Send_Frame((SDOrx + SlaveID), 0, 8, msg);
}

u8 Sdo_WrU16(u8 SlaveID, u16 index, u8 subindex, u32 data)  //д2�ֽ�
{
  u8 msg[8];
  msg[0] = 0x2B;  //����������
  msg[1] = index & 0xff;
  msg[2] = (index >> 8) & 0xff;
  msg[3] = subindex;
  msg[4] = data & 0xff;
  msg[5] = (data >> 8) & 0xff;
  msg[6] = msg[7] = 0;
  return CAN1_Send_Frame(
      (SDOrx + SlaveID), 0, 8,
      msg);  // SDOrxΪ0x600,������?�Ƕ���վ��˵,������0x600
}

u8 Sdo_WrU24(u8 SlaveID, u16 index, u8 subindex,
             u32 data)  //д3�ֽڣ�ʵ��ֻ��3byte
{
  u8 msg[8];
  msg[0] = 0x27;  //����������
  msg[1] = index & 0xff;
  msg[2] = (index >> 8) & 0xff;
  msg[3] = subindex;
  msg[4] = data & 0xff;
  msg[5] = (data >> 8) & 0xff;
  msg[6] = (data >> 16) & 0xff;
  msg[7] = 0;
  // msg[7] = (Dword>>24)&0xff;

  return CAN1_Send_Frame(
      (SDOrx + SlaveID), 0, 8,
      msg);  // SDOrxΪ0x600,������?�Ƕ���վ��˵,������0x600
}

u8 Sdo_WrU32(u8 SlaveID, u16 index, u8 subindex, u32 data)  //д4�ֽ�
{
  u8 msg[8];
  msg[0] = 0x23;  //����������
  msg[1] = index & 0xff;
  msg[2] = (index >> 8) & 0xff;
  msg[3] = subindex;
  msg[4] = data & 0xff;
  msg[5] = (data >> 8) & 0xff;
  msg[6] = (data >> 16) & 0xff;
  msg[7] = (data >> 24) & 0xff;
  return CAN1_Send_Frame(
      (SDOrx + SlaveID), 0, 8,
      msg);  // SDOrxΪ0x600,������?�Ƕ���վ��˵,������0x600
}

// SDO ��Ӧ�ľ�����������

// u8

// sdo ��
u8 Sdo_Rd(u8 SlaveID, u16 index, u8 subindex)  //��1�ֽ�
{  //�����ݲ����ֽڷ֣�ֻ���ṩ��ַ�Ϳ�����.���Ǵ�վ����֡��Ҫ�����ֽ�������
  u8 msg[8];
  msg[0] = 0x40;  //����������
  msg[1] = index & 0xff;
  msg[2] = (index >> 8) & 0xff;
  msg[3] = subindex;
  msg[4] = 0;
  msg[5] = 0;
  msg[6] = 0;
  msg[7] = 0;
  return CAN1_Send_Frame(
      (SDOrx + SlaveID), 0, 8,
      msg);  // SDOrxΪ0x600,������?�Ƕ���վ��˵,������0x600
             // ,����cob-idΪ0x580
}

// SDO ����ڽ��մ���?,���ݸ��ݵ�ַ����
void DecodeWrite(u8 SlaveID, u8 cmd, u16 index, u8 subindex)  //
{
  ;  // decode and write global Var
}
void ProcessSDOrx(CanRxMsg *m) {
  u8 SlaveID = m->StdId & 0x7F;
  u8 cmd_code = m->Data[0];
  u16 index = (m->Data[2] << 8) | m->Data[1];
  u8 subindex = m->Data[3];
  DecodeWrite(SlaveID, cmd_code, index, subindex);
}

// SDO дӦ��
u8 ProcessSDOtx(CanRxMsg *m) {  //

  // u8 SlaveID = m->StdId & 0xFF;
  u8 cmd_code = m->Data[0];
  if (cmd_code == 0x60)
    return 0;  // OK
  else         // 0x80
    return 1;  // err
}

/*!
**
**
** @param d
** @param m
**/
void canDispatch(CanRxMsg *m) {
  u16 cob_id = m->StdId & (~0x007F);  //(m->StdId >>7)&0x0F;
  u16 SlaveId = (m->StdId & 0x7F);
  switch (cob_id) {
    case PDO1tx:  // 0x180 + //��ЩΪ����
                  // ,//״̬��ת�ӣ�λ��
      ptrServ[SlaveId]->StatusWord = (u16)(m->Data[1] << 8) | m->Data[0];
      ptrServ[SlaveId]->TrqPV = (s16)((m->Data[3] << 8) | m->Data[2]);
      ptrServ[SlaveId]->PosPV = (s32)((m->Data[7] << 24) | (m->Data[6] << 16) |
                                      (m->Data[5] << 8) | m->Data[4]);
      OSSemPost(sem_SrvCAN_rx);
      break;
    case PDO2tx:  // 0x280
      ptrServ[SlaveId]->StatusWord = (m->Data[1] << 8) | m->Data[0];
      ptrServ[SlaveId]->TrqPV = (s16)((m->Data[3] << 8) | m->Data[2]);
      ptrServ[SlaveId]->SpdPV = (s32)((m->Data[7] << 24) | (m->Data[6] << 16) |
                                      (m->Data[5] << 8) | m->Data[4]);
      OSSemPost(sem_SrvCAN_rx);
      break;
    case PDO3tx:  // 0x380
      ptrServ[SlaveId]->SpdPV = (s32)((m->Data[3] << 24) | m->Data[2] << 16 |
                                      (m->Data[1] << 8) | m->Data[0]);
      ptrServ[SlaveId]->PosPV = (s32)((m->Data[7] << 24) | (m->Data[6] << 16) |
                                      (m->Data[5] << 8) | m->Data[4]);
      OSSemPost(sem_SrvCAN_rx);
      break;
    case PDO4tx:  // 0x480
      ptrServ[SlaveId]->StatusWord = (u16)(m->Data[1] << 8) | m->Data[0];
      ptrServ[SlaveId]->ServErr = (u16)((m->Data[3] << 8) | m->Data[2]);
      ptrServ[SlaveId]->TrqPV = (s16)((m->Data[5] << 8) | m->Data[4]);
      ptrServ[SlaveId]->CtrlMode = m->Data[6];
      OSSemPost(sem_SrvCAN_rx);
      break;

    case PDO1rx:  // 0x200
    case PDO2rx:  // 0x300
    case PDO3rx:  // 0x400
    case PDO4rx:  // 0x500
                  // ��վ���͸���վ��PDO,û�������ݲ���Ҫ����
                  // proceedPDO(d,m);
      break;
    case SDOtx:  // 0x581,��վ���գ���վ���ͣ�
      ProcessSDOrx(m);
      OSSemPost(sem_SrvCAN_rx);

    case SDOrx:  // 0x600,��վ���ͣ���վ����
      ProcessSDOtx(
          m);  //�鿴Ӧ���Ƿ���ȷ����ʼ��Ҫ�����״�?.
      OSSemPost(sem_SrvCAN_rx);
      break;
    case NODE_GUARD:  // 0x700
      ptrServ[SlaveId]->ServSTA =
          (m->Data[0]) & 0x7F;  // 0,or 0x04(0x84),or 0x05(0x85),or 0x7F(0xFF)
      OSSemPost(sem_SrvCAN_rx);
      // proceedNODE_GUARD(d,m);
      break;
    default:
      break;
  }
}

/*
u8 Sdo_RdU16(u8 SlaveID,u16 index,u8 subindex) //��2�ֽ�
{
}

u8 Sdo_RdU24(u8 SlaveID,u16 index,u8 subindex)
//��3�ֽڣ�ʵ��ֻ��3byte
{
}

u8 Sdo_RdU32(u8 SlaveID,u16 index,u8 subindex) //��4�ֽ�
{
}
*/

// 7. LSS���� ��ʱû��

// 8. TIME_STAMP���� ��ʱû��

//��������Ĵ���?

// can�ڽ������ݲ�ѯ
// buf:���ݻ�����;
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 CAN1_Receive_Msg(u8 *buf) {
  u32 i;
  CanRxMsg RxMessage;
  if (CAN_MessagePending(CAN1, CAN_FIFO0) == 0)
    return 0;  //û�н��յ�����,ֱ���˳�

  // printf("rx data!");
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);  //��ȡ����
  for (i = 0; i < RxMessage.DLC; i++) buf[i] = RxMessage.Data[i];
  return RxMessage.DLC;
}

void Wait(u32 time) {
  u32 i = time;
  while (i != 0) {
    i--;
  }
}

void CanInit(void) {
  CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_6tq, CAN_BS1_7tq, CAN_BAUD_1Mbps_brp,
                 CAN_Mode_Normal);  // brp ������
  sem_SrvCAN_tx = OSSemCreate(3);
  sem_SrvCAN_rx = OSSemCreate(0);
}

void MotorInit(void) {
  u32 i, j;

#if 0	
	//���ڳ�������������
	NMT_RstNode(0);	//��λ���нڵ�
	Wait(10000);
	NMT_PreSTA(0); //���нڵ����Ԥ����״�?
	Wait(10000);
	
	//���������õ���������������1~4Ϊ���������?
	//���?1
	Sdo_WrU8(1,ModeOperation_Idx,ModeOperation_SubIdx,PPM_MODE); //����ģʽΪλ��PPMģʽ
	Wait(10000);
	Sdo_WrU32(1,MaxPVelocity_Idx,MaxPVelocity_SubIdx,8000); //��������ٶ�?
	Wait(10000);
	Sdo_WrU32(1,PAcceleration_Idx,PAcceleration_SubIdx,1000); //���ü��ٶ�1000
	Wait(10000);
	Sdo_WrU32(1,PDeceleration_Idx,PDeceleration_SubIdx,1000); //���ü��ٶ�1000
	Wait(10000);
	Sdo_WrU32(1,QuickStopDec_Idx,QuickStopDec_SubIdx,3000);//���ý������ٶ�Ϊ3000
	Wait(10000);
	Sdo_WrU16(1,MotionPType_Idx,MotionPType_SubIdx,0); // �����˶��ķ�ʽΪlinear �� sin2C
	Wait(10000);
	Sdo_WrU16(1,OutCurrentLimit_Idx,OutCurrentLimit_SubIdx,5000);//	
	Wait(10000);
  	//u8 Sdo_WrU8(u8 SlaveID,u16 index,u8 subindex,u32 data)
	
	//���?2
	//��Ҫ��������16Ϊ��32Ϊ��8λ�ģ��ֱ��ò�ͬ�ĺ���
	Sdo_WrU8(2,ModeOperation_Idx,ModeOperation_SubIdx,PPM_MODE); //����ģʽΪλ��PPMģʽ
	Wait(10000);
	Sdo_WrU32(2,MaxPVelocity_Idx,MaxPVelocity_SubIdx,8000); //��������ٶ�?
	Wait(10000);
	Sdo_WrU32(2,PAcceleration_Idx,PAcceleration_SubIdx,1000); //���ü��ٶ�1000
	Wait(10000);
	Sdo_WrU32(2,PDeceleration_Idx,PDeceleration_SubIdx,1000); //���ü��ٶ�1000
	Wait(10000);
	Sdo_WrU32(2,QuickStopDec_Idx,QuickStopDec_SubIdx,3000); //���ý������ٶ�Ϊ3000
	Wait(10000);
	Sdo_WrU16(2,MotionPType_Idx,MotionPType_SubIdx,0);     // �����˶��ķ�ʽΪlinear �� sin2C
	Wait(10000);
	Sdo_WrU16(2,OutCurrentLimit_Idx,OutCurrentLimit_SubIdx,5000);// ��������������5000����ĵ�����Ϊ5000
	Wait(10000);

	//���?3
	//��Ҫ��������16Ϊ��32Ϊ��8λ�ģ��ֱ��ò�ͬ�ĺ���
	Sdo_WrU8(3,ModeOperation_Idx,ModeOperation_SubIdx,PPM_MODE); //����ģʽΪλ��PPMģʽ
	Wait(10000);
	Sdo_WrU32(3,MaxPVelocity_Idx,MaxPVelocity_SubIdx,8000); //��������ٶ�?
	Wait(10000);
	Sdo_WrU32(3,PAcceleration_Idx,PAcceleration_SubIdx,1000); //���ü��ٶ�1000
	Wait(10000);
	Sdo_WrU32(3,PDeceleration_Idx,PDeceleration_SubIdx,1000); //���ü��ٶ�1000
	Wait(10000);
	Sdo_WrU32(3,QuickStopDec_Idx,QuickStopDec_SubIdx,3000); //���ý������ٶ�Ϊ3000
	Wait(10000);
	Sdo_WrU16(3,MotionPType_Idx,MotionPType_SubIdx,0);     // �����˶��ķ�ʽΪlinear �� sin2C
	Wait(10000);
	Sdo_WrU16(3,OutCurrentLimit_Idx,OutCurrentLimit_SubIdx,5000);// ��������������5000����ĵ�����Ϊ5000
	Wait(10000);

	//���?4
	//��Ҫ��������16Ϊ��32Ϊ��8λ�ģ��ֱ��ò�ͬ�ĺ���
	Sdo_WrU8(4,ModeOperation_Idx,ModeOperation_SubIdx,PPM_MODE); //����ģʽΪλ��PPMģʽ
	Wait(10000);
	Sdo_WrU32(4,MaxPVelocity_Idx,MaxPVelocity_SubIdx,8000); //��������ٶ�?
	Wait(10000);
	Sdo_WrU32(4,PAcceleration_Idx,PAcceleration_SubIdx,1000); //���ü��ٶ�1000
	Wait(10000);
	Sdo_WrU32(4,PDeceleration_Idx,PDeceleration_SubIdx,1000); //���ü��ٶ�1000
	Wait(10000);
	Sdo_WrU32(4,QuickStopDec_Idx,QuickStopDec_SubIdx,3000); //���ý������ٶ�Ϊ3000
	Wait(10000);
	Sdo_WrU16(4,MotionPType_Idx,MotionPType_SubIdx,0);     // �����˶��ķ�ʽΪlinear �� sin2C
	Wait(10000);
	Sdo_WrU16(4,OutCurrentLimit_Idx,OutCurrentLimit_SubIdx,5000);// ��������������5000����ĵ�����Ϊ5000
	Wait(10000);


	//���?5~7Ϊ�������?
	//���?5
	//��Ҫ��������16Ϊ��32Ϊ��8λ�ģ��ֱ��ò�ͬ�ĺ���
	Sdo_WrU8(5,ModeOperation_Idx,ModeOperation_SubIdx,PPM_MODE); //����ģʽΪλ��PPMģʽ
	Wait(10000);
	Sdo_WrU32(5,MaxPVelocity_Idx,MaxPVelocity_SubIdx,8000); //��������ٶ�?
	Wait(10000);
	Sdo_WrU32(5,PAcceleration_Idx,PAcceleration_SubIdx,1000); //���ü��ٶ�1000
	Wait(10000);
	Sdo_WrU32(5,PDeceleration_Idx,PDeceleration_SubIdx,1000); //���ü��ٶ�1000
	Wait(10000);
	Sdo_WrU32(5,QuickStopDec_Idx,QuickStopDec_SubIdx,3000); //���ý������ٶ�Ϊ3000
	Wait(10000);
	Sdo_WrU16(5,MotionPType_Idx,MotionPType_SubIdx,0);     // �����˶��ķ�ʽΪlinear �� sin2C
	Wait(10000);
	//Sdo_WrU16(5,OutCurrentLimit_Idx,OutCurrentLimit_SubIdx,15000);// 

	//���?6
	Sdo_WrU8(6,ModeOperation_Idx,ModeOperation_SubIdx,PPM_MODE); //����ģʽΪλ��PPMģʽ
	Wait(10000);
	Sdo_WrU32(6,MaxPVelocity_Idx,MaxPVelocity_SubIdx,8000); //��������ٶ�?
	Wait(10000);
	Sdo_WrU32(6,PAcceleration_Idx,PAcceleration_SubIdx,1000); //���ü��ٶ�1000
	Wait(10000);
	Sdo_WrU32(6,PDeceleration_Idx,PDeceleration_SubIdx,1000); //���ü��ٶ�1000
	Wait(10000);
	Sdo_WrU32(6,QuickStopDec_Idx,QuickStopDec_SubIdx,3000); //���ý������ٶ�Ϊ3000
	Wait(10000);
	Sdo_WrU16(6,MotionPType_Idx,MotionPType_SubIdx,0);     // �����˶��ķ�ʽΪlinear �� sin2C
	Wait(10000);
	//Sdo_WrU16(6,OutCurrentLimit_Idx,OutCurrentLimit_SubIdx,15000);// 
	
	//���?7
	Sdo_WrU8(7,ModeOperation_Idx,ModeOperation_SubIdx,PPM_MODE); //����ģʽΪλ��PPMģʽ
	Wait(10000);
	Sdo_WrU32(7,MaxPVelocity_Idx,MaxPVelocity_SubIdx,8000); //��������ٶ�?
	Wait(10000);
	Sdo_WrU32(7,PAcceleration_Idx,PAcceleration_SubIdx,1000); //���ü��ٶ�1000
	Wait(10000);
	Sdo_WrU32(7,PDeceleration_Idx,PDeceleration_SubIdx,1000); //���ü��ٶ�1000
	Wait(10000);
	Sdo_WrU32(7,QuickStopDec_Idx,QuickStopDec_SubIdx,3000); //���ý������ٶ�Ϊ3000
	Wait(10000);
	Sdo_WrU16(7,MotionPType_Idx,MotionPType_SubIdx,0);     // �����˶��ķ�ʽΪlinear �� sin2C
	Wait(10000);
	//Sdo_WrU16(7,OutCurrentLimit_Idx,OutCurrentLimit_SubIdx,15000);//
#endif

  NMT_Start(0);  //�������еĽڵ�

  Wait(10000);

  for (i = 1; i <= 7; i++) {
    u8 err;
    for (j = 0; j < 5; j++)  //ʧ�������?5��
    {
      Rd_NodeGuard(i);
      OSSemPend(sem_SrvCAN_rx, 0, &err);
      if (ptrServ[i]->ServSTA == Operational) {
        // j=100;//ֱ��ʹ������
        break;
      }
    }
    //�������ӳ�ʼ��ʧ�ܵļĴ���������.
  }

  Wait(1000);
  // 4������?�±�����
  //�ŷ�Ԥ����
  SetMotorCtrlword(1, SERV_ON_PRE);
  SetMotorCtrlword(2, SERV_ON_PRE);
  SetMotorCtrlword(3, SERV_ON_PRE);
  SetMotorCtrlword(4, SERV_ON_PRE);
  SetMotorCtrlword(5, SERV_ON_PRE);
  SetMotorCtrlword(6, SERV_ON_PRE);
  SetMotorCtrlword(7, SERV_ON_PRE);
  CMD_SYNC();  //����ͬ����ʹ�÷��͵�������Ч

  //�ŷ�����
  SetMotorCtrlword(1, SERV_ON);
  SetMotorCtrlword(2, SERV_ON);
  SetMotorCtrlword(3, SERV_ON);
  SetMotorCtrlword(4, SERV_ON);
  SetMotorCtrlword(5, SERV_ON);
  SetMotorCtrlword(6, SERV_ON);
  SetMotorCtrlword(7, SERV_ON);
  CMD_SYNC();  //����ͬ����ʹ�÷��͵�������Ч
}

void HandleCan(void) {
  // CanInit();
  // MotorInit();
  u8 msg[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  u16 cob_id = 0;
  u8 slave_Id = 1;

  cob_id = Para[5];
  slave_Id = Para[11];
  if (Para[6] != 0) {
    switch (Para[6]) {
      case 1:
        NMT_Start(slave_Id);
        break;
      case 2:
        CMD_SYNC();
        break;
      case 3:
        NMT_PreSTA(slave_Id);
        // Rd_NodeGuard(slave_Id);
        break;
      case 4:
        TX_PDO1(slave_Id);
        break;
      case 5:
        RX_PDO1(slave_Id);
        break;
      case 6:
        Sdo_WrU32(slave_Id, 1000, 0, 0x55AA1234);
        break;
      case 7:
        Sdo_WrU24(slave_Id, 1000, 0, 0x55AA1234);
        break;
      case 8:
        Sdo_WrU16(slave_Id, 1000, 0, 0x55AA1234);
        break;
      case 9:
        Sdo_WrU8(slave_Id, 1000, 0, 0x55AA1234);
        break;
      case 10:
        Sdo_Rd(slave_Id, 0x6041, 0);
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
        SetMotorCtrlword(slave_Id, Para[7]);
        break;
      case 16:
        SetMotorAbsPos(slave_Id, Para[8]);
        break;
      case 17:
        SetMotorSpd(slave_Id, Para[9]);
        break;
      case 18:
        SetMotorCurrentLimit(slave_Id, Para[10]);
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
        // u8 CAN1_Send_Frame(u16 CobId,u8 Rtr,u8 Len,u8* msg)
        CAN1_Send_Frame(cob_id, 0, 8, msg);
        break;
      default:
        break;
    }
  }
  Para[6] = 0;
}
