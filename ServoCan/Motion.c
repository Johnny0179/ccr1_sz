/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
 * File Name          : Motion.c
 * Author             :
 * Date First Issued  : 27/04/2010
 * Description        : UART��Ӳ�����ú�modbusЭ�鴦��
 ********************************************************************************/

#include "Motion.h"
#include "ServoCan.h"
#include "delay.h"
#include "glb_reg.h"
#include "includes.h"
#include "sys.h"
#include "usart.h"

u16 CalcuNextState(u16 CurrentState, u16 dir);
void Motion_Open_M1M2(u16 *state);
void Motion_Open_M3M4(u16 *state);
void Motion_Close_M1M2(u16 *state);
void Motion_Close_M3M4(u16 *state);
void Motion_Extend_M5M6M7(u16 *state);
void Motion_Shorten_M5M6M7(u16 *state);
void Motion_Home_M1M2(u16 *state);
void Motion_Home_M3M4(u16 *state);
void Motion_Home_M5M6M7(u16 *state);
void Motion_Home_All(u16 *state);

void MotionInit(void) {
  u16 i;

  ptrMotionBlk->MotionCtrlWord = 0;
  ptrMotionBlk->MotionCmdCode = 0;
  ptrMotionBlk->MotionPause = 0;
  ptrMotionBlk->MotionMode = 0;
  ptrMotionBlk->MotionDir = 1;
  ptrMotionBlk->SysState = 0;
  ptrMotionBlk->MotionState = S_MOTION_IDLE;
  ptrMotionBlk->MotionStateLast = S_MOTION_IDLE;
  ptrMotionBlk->MotionStateNext = S_MOTION_IDLE;
  ptrMotionBlk->SmallState = 0;
  ptrMotionBlk->StatusWord = 0;
  ptrMotionBlk->FuncInnerState = 0;
  ptrMotionBlk->ErrCode = 0;
  ptrMotionBlk->CmdRepState = 0;
  ptrMotionBlk->DebugSlaveId = 1;
  ptrMotionBlk->DebugMaxIqSet = 1300;
  ptrMotionBlk->DebugSpdSet = 500;
  ptrMotionBlk->DebugPosSet = 500;
  ptrCfgMotionPar->PosFactor = 100;
  ptrCfgMotionPar->PosOpenLen = 3000 * 0.7;

  ptrCfgMotionPar->ClimbLenSameFlag = 1;
  ptrCfgMotionPar->PosClimbLen = 1500 * 0.7;
  ptrCfgMotionPar->PosClimbLen2 = 1500 * 0.7;
  ptrCfgMotionPar->PosClimbLen3 = 1500 * 0.7;

  ptrCfgMotionPar->HomingSpd = 1000;  // rpm
  ptrCfgMotionPar->HomingSpdM5M6M7 = 200;
  ptrCfgMotionPar->CloseSpd = 5000;
  ptrCfgMotionPar->ClimbSpd = RATED_SPD2 / 2;
  ptrCfgMotionPar->Iq1Limit = RATED_IQ1 * 7 / 8;  //��ʱ�޸�Ϊ1/2;
  ptrCfgMotionPar->DeltaPos = 20;                 //�����?
  ptrCfgMotionPar->DeltaPosM5M6M7 = 2000;
  ptrCfgMotionPar->Iq2Limit = RATED_IQ2;
  ptrMotionBlk->StepsLeft = 0;     // N��һͣ
  ptrMotionBlk->StepsPreSet = 1;   // N��һͣ
  ptrMotionBlk->AutoCycleEn = 0;   //
  ptrMotionBlk->AutoCycleNum = 1;  //
  ptrMotionBlk->AutocycleLeft = 0;
  ptrMotionBlk->AutoCycleState = 0;
  ptrMotionBlk->StartNodeFlag = 0;
  ptrMotionBlk->M1orM3OpenState = 0;
  ptrCfgMotionPar->PosOpenLen2 = 2500;
  ptrCfgMotionPar->CompCoff = 128;
  ptrMotionBlk->Pos6Pos5Err = 10000;
  ptrMotionBlk->Pos7Pos5Err = 20000;
  ptrCfgMotionPar->acc1 = 20000;
  ptrCfgMotionPar->acc2 = 10000;

  for (i = 1; i <= 7; i++) {
    if (i <= 4) {
      ptrServ[i]->MaxcurrentLocked =
          (s32)(ptrCfgMotionPar->Iq1Limit) * 1000 / RATED_IQ1 -
          50;  //�����λ���?�صİٷֱ�
    } else {
      ptrServ[i]->MaxcurrentLocked =
          (s32)(ptrCfgMotionPar->Iq2Limit) * 1000 / RATED_IQ2 -
          20;  // 0.85 //�����λ���?�صİٷֱ�
    }
    ptrServ[i]->SlaveID = i;
    ptrServ[i]->PosLocked = 0;
  }

  //	delay_ms(2000);
  //	delay_ms(2000);
  delay_ms(2000);
  delay_ms(2000);
  NMT_Start(0);  //�����ڵ�

  // StartMotor(2);		//�������нڵ�
}

void VarClear(void) {
  u16 i;

  ptrMotionBlk->MotionCtrlWord = 0;
  ptrMotionBlk->MotionCmdCode = 0;
  ptrMotionBlk->MotionPause = 0;
  ptrMotionBlk->MotionMode = 0;
  // ptrMotionBlk->MotionDir = 1;
  ptrMotionBlk->MotionState = S_MOTION_IDLE;
  ptrMotionBlk->MotionStateLast = S_MOTION_IDLE;
  ptrMotionBlk->MotionStateNext = S_MOTION_IDLE;
  ptrMotionBlk->SmallState = 0;
  ptrMotionBlk->StatusWord = 0;
  ptrMotionBlk->FuncInnerState = 0;
  ptrMotionBlk->ErrCode = 0;
  ptrMotionBlk->CmdRepState = 0;

  ptrMotionBlk->DebugSlaveId = 1;
  ptrMotionBlk->DebugMaxIqSet = 1300;
  ptrMotionBlk->DebugSpdSet = 500;
  ptrMotionBlk->DebugPosSet = 500;

  ptrCfgMotionPar->PosFactor = 100;
  ptrCfgMotionPar->DeltaPos = 20;  //�����?
  ptrCfgMotionPar->DeltaPosM5M6M7 = 2000;

  ptrMotionBlk->AutoCycleNum = 1;  //
  ptrMotionBlk->StepsPreSet = 1;   // N��һͣ
  ptrMotionBlk->MotionDir = 1;

  ptrMotionBlk->StepsLeft = 0;    // N��һͣ
  ptrMotionBlk->AutoCycleEn = 0;  //
  ptrMotionBlk->AutocycleLeft = 0;
  ptrMotionBlk->AutoCycleState = 0;
  ptrMotionBlk->StartNodeFlag = 0;
  ptrMotionBlk->M1orM3OpenState = 0;

  ptrCfgMotionPar->CompCoff = 128;
  ptrMotionBlk->Pos6Pos5Err = 10000;
  ptrMotionBlk->Pos7Pos5Err = 20000;

  for (i = 1; i <= 7; i++) {
    if (i <= 4) {
      ptrServ[i]->MaxcurrentLocked =
          (s32)(ptrCfgMotionPar->Iq1Limit) * 1000 / RATED_IQ1 -
          50;  //�����λ���?�صİٷֱ�
    } else {
      ptrServ[i]->MaxcurrentLocked =
          (s32)(ptrCfgMotionPar->Iq2Limit) * 1000 / RATED_IQ2 -
          20;  // 0.85 //�����λ���?�صİٷֱ�
    }
    ptrServ[i]->SlaveID = i;
    ptrServ[i]->PosLocked = 0;
  }

  //	delay_ms(2000);
  //	delay_ms(2000);
  delay_ms(2000);
  delay_ms(2000);
  NMT_Start(0);  //�����ڵ�

  // StartMotor(2);		//�������нڵ�
}

void ReadPos(u16 ID) {
  u8 err;
  /*if(ID ==0)
  { //read all servo
          ptrServ[1]->PosPV_Last = ptrServ[1]->PosPV; //����
          ptrServ[2]->PosPV_Last = ptrServ[2]->PosPV; //����
          ptrServ[3]->PosPV_Last = ptrServ[3]->PosPV; //����
          ptrServ[4]->PosPV_Last = ptrServ[4]->PosPV; //����
          ptrServ[5]->PosPV_Last = ptrServ[5]->PosPV; //����
          ptrServ[6]->PosPV_Last = ptrServ[6]->PosPV; //����
          ptrServ[7]->PosPV_Last = ptrServ[7]->PosPV; //����
          RX_PDO1(1);
          OSSemPend(sem_SrvCAN_rx,500,&err);
          RX_PDO1(2);
          OSSemPend(sem_SrvCAN_rx,500,&err);
          RX_PDO1(3);
          OSSemPend(sem_SrvCAN_rx,500,&err);
          RX_PDO1(4);
          OSSemPend(sem_SrvCAN_rx,500,&err);
          RX_PDO1(5);
          OSSemPend(sem_SrvCAN_rx,500,&err);
          RX_PDO1(6);
          OSSemPend(sem_SrvCAN_rx,500,&err);
          RX_PDO1(7);
          OSSemPend(sem_SrvCAN_rx,500,&err);
          

  }
  else */
  {
    ptrServ[ID]->PosPV_Last = ptrServ[ID]->PosPV;  //����
    RX_PDO1(ID);
    OSSemPend(sem_SrvCAN_rx, 100, &err);
  }
}

void AutoMovePragram(u16 ctrl, u16 setCycle) {
  //����������
  static u16 state = 0;
  //	static u16 cycle_left = 0;
  static u16 ctrl_dly = 0;

  switch (state) {
    case 0:
      if ((ptrMotionBlk->MotionState == S_MOTION_IDLE) &&
          (ptrMotionBlk->SysState == SYS_WORKING) && (DebugFlag == 0)) {
        if ((ctrl_dly == 0) && (ctrl == 1))  // rising edge
        {
          ptrMotionBlk->AutocycleLeft = setCycle;
          Para[2] = 0;
          state = 1;
        }
      }
      break;
    case 1:
      if (ctrl == 0) {
        ptrMotionBlk->MotionCmdCode = 2;  // stop
        ptrMotionBlk->AutocycleLeft = 0;
        state = 0;
      } else {
        ptrMotionBlk->MotionCmdCode = 1;
        state = 2;
      }
      break;
    case 2:
      if (ctrl == 0) {
        ptrMotionBlk->MotionCmdCode = 2;  // stop
        ptrMotionBlk->AutocycleLeft = 0;
        state = 0;
      } else if (ptrMotionBlk->MotionState != S_MOTION_IDLE) {
        state = 3;
      }
      break;
    case 3:
      if (ctrl == 0) {
        ptrMotionBlk->MotionCmdCode = 2;  // stop
        ptrMotionBlk->AutocycleLeft = 0;
        state = 0;
      } else if (ptrMotionBlk->MotionState ==
                 S_MOTION_IDLE)  // finish 1 direction
      {
        if (ptrMotionBlk->AutocycleLeft != 0) {
          ptrMotionBlk->AutocycleLeft--;
        }

        if (ptrCfgMotionPar->DirAutoChg != 0) {
          if (ptrMotionBlk->MotionDir == 0) {
            ptrMotionBlk->MotionDir = 1;
          } else {
            ptrMotionBlk->MotionDir = 0;
          }
        }

        if (ptrMotionBlk->AutocycleLeft == 0) {
          state = 0;

          Para[3] = Para[2];  // test
        } else {
          state = 1;
        }
      }
      break;
    default:
      state = 0;
      break;
  }

  ctrl_dly = ctrl;  //
  ptrMotionBlk->AutoCycleState = state;
}

void HandleCmd(void) {
  if (ptrMotionBlk->MotionCmdCode != 0) {
    ptrMotionBlk->CmdRepState = 0;
    switch (ptrMotionBlk->MotionCmdCode) {
      case 0:  // do nothing
        break;
      case 1:  //���� ����
        if (ptrMotionBlk->SysState == SYS_WORKING) {
          if ((ptrMotionBlk->MotionState == S_MOTION_IDLE) &&
              (DebugFlag == 0)) {
            ptrMotionBlk->SmallState = 0;

            ptrMotionBlk->StatusWord &= ~BIT_SOFT_STOP;
            ptrMotionBlk->StepsLeft = ptrMotionBlk->StepsPreSet;  //Ԥ���߶��ٲ�
            if (ptrMotionBlk->MotionDir == 1) {
              ptrMotionBlk->StatusWord |= BIT_DIR;
              ptrMotionBlk->MotionState = S_MOTION_OPEN_M1M2;
            } else {
              ptrMotionBlk->StatusWord &= ~BIT_DIR;
              ptrMotionBlk->MotionState = S_MOTION_OPEN_M3M4;
            }

            ptrMotionBlk->MotionCmdCode = 0;  //������ִ�У�����0
          }
        } else
          ;
        break;
      case 2:                                         //ֹͣ����
        if (ptrMotionBlk->SysState == SYS_WORKING) {  //���Ƕ�ʹ�ܵ����֣�
          ptrMotionBlk->StatusWord |= BIT_SOFT_STOP;  //���ֱ�ס�����������?
          ptrMotionBlk->MotionCmdCode = 0;  //������ִ�У�����0
        } else {
          ptrMotionBlk->MotionCmdCode = 0;  //������ִ�У�����0
        }
        break;

      case 3:  //������ͣ�������ڵ���
        if (ptrMotionBlk->SysState == SYS_WORKING) {  //���Ƕ�ʹ�ܵ����֣�
          ptrMotionBlk->StatusWord |= BIT_IM_PAUSE;  //���ֱ�ס�����������?
          ptrMotionBlk->MotionCmdCode = 0;  //������ִ�У�����0
        }
        break;
      case 4:
        ptrMotionBlk->StatusWord &= ~BIT_SOFT_STOP;
        break;
      case 5:
        ptrMotionBlk->StatusWord &= ~BIT_IM_PAUSE;
        break;
      case 6:
        ptrMotionBlk->MotionState = 0;
      case 7:
        // u8 Sdo_WrU32(u8 SlaveID,u16 index,u8 subindex,u32 data) //д4�ֽ�
        Sdo_WrU32(1, 0x6083, 0, ptrCfgMotionPar->acc1);
        delay_ms(2);
        Sdo_WrU32(1, 0x6084, 0, ptrCfgMotionPar->acc1);
        delay_ms(2);
        Sdo_WrU32(1, 0x6085, 0, ptrCfgMotionPar->acc1);
        delay_ms(2);

        Sdo_WrU32(2, 0x6083, 0, ptrCfgMotionPar->acc1);
        delay_ms(2);
        Sdo_WrU32(2, 0x6084, 0, ptrCfgMotionPar->acc1);
        delay_ms(2);
        Sdo_WrU32(2, 0x6085, 0, ptrCfgMotionPar->acc1);
        delay_ms(2);

        Sdo_WrU32(3, 0x6083, 0, ptrCfgMotionPar->acc1);
        delay_ms(2);
        Sdo_WrU32(3, 0x6084, 0, ptrCfgMotionPar->acc1);
        delay_ms(2);
        Sdo_WrU32(3, 0x6085, 0, ptrCfgMotionPar->acc1);
        delay_ms(2);

        Sdo_WrU32(4, 0x6083, 0, ptrCfgMotionPar->acc1);
        delay_ms(2);
        Sdo_WrU32(4, 0x6084, 0, ptrCfgMotionPar->acc1);
        delay_ms(2);
        Sdo_WrU32(4, 0x6085, 0, ptrCfgMotionPar->acc1);
        delay_ms(2);

        Sdo_WrU32(5, 0x6083, 0, ptrCfgMotionPar->acc2);
        delay_ms(2);
        Sdo_WrU32(5, 0x6084, 0, ptrCfgMotionPar->acc2);
        delay_ms(2);
        Sdo_WrU32(5, 0x6085, 0, ptrCfgMotionPar->acc2);
        delay_ms(2);

        Sdo_WrU32(6, 0x6083, 0, ptrCfgMotionPar->acc2);
        delay_ms(2);
        Sdo_WrU32(6, 0x6084, 0, ptrCfgMotionPar->acc2);
        delay_ms(2);
        Sdo_WrU32(6, 0x6085, 0, ptrCfgMotionPar->acc2);
        delay_ms(2);

        Sdo_WrU32(7, 0x6083, 0, ptrCfgMotionPar->acc2);
        delay_ms(2);
        Sdo_WrU32(7, 0x6084, 0, ptrCfgMotionPar->acc2);
        delay_ms(2);
        Sdo_WrU32(7, 0x6085, 0, ptrCfgMotionPar->acc2);
        delay_ms(2);
        break;
      case 8:
      case 9:
      case 10:
        break;  //Ԥ��
      case 11:  // M1M2����
        if (ptrMotionBlk->SysState == SYS_WORKING) {
          if (ptrMotionBlk->MotionState == S_MOTION_IDLE) {
            ptrMotionBlk->SmallState = 0;

            ptrMotionBlk->StatusWord |= BIT_HOMING_BUSY;

            ptrMotionBlk->MotionState = S_MOTION_HOME_M1M2;

            ptrMotionBlk->MotionCmdCode = 0;  //������ִ�У�����0
          }
        }
        break;
      case 12:  // M3M4 ����
        if (ptrMotionBlk->SysState == SYS_WORKING) {
          if (ptrMotionBlk->MotionState == S_MOTION_IDLE) {
            ptrMotionBlk->SmallState = 0;

            ptrMotionBlk->StatusWord |= BIT_HOMING_BUSY;

            ptrMotionBlk->MotionState = S_MOTION_HOME_M3M4;

            ptrMotionBlk->MotionCmdCode = 0;  //������ִ�У�����0
          }
        }
        break;

      case 13:  // M5M6M7 ����
        if (ptrMotionBlk->SysState == SYS_WORKING) {
          if (ptrMotionBlk->MotionState == S_MOTION_IDLE) {
            ptrMotionBlk->SmallState = 0;

            ptrMotionBlk->StatusWord |= BIT_HOMING_BUSY;

            ptrMotionBlk->MotionState = S_MOTION_HOME_M5M6M7;

            ptrMotionBlk->MotionCmdCode = 0;  //������ִ�У�����0
          }
        }
        break;
      case 14:  // ALL ����
        if (ptrMotionBlk->SysState == SYS_WORKING) {
          if (ptrMotionBlk->MotionState == S_MOTION_IDLE) {
            ptrMotionBlk->SmallState = 0;

            ptrMotionBlk->StatusWord |= BIT_HOMING_BUSY;

            ptrMotionBlk->MotionState = S_MOTION_HOME_ALL;

            ptrMotionBlk->MotionCmdCode = 0;  //������ִ�У�����0
          }
        }
        break;
      case 15:  // M1M2 open
        if (ptrMotionBlk->SysState == SYS_WORKING) {
          if (ptrMotionBlk->MotionState == S_MOTION_IDLE) {
            ptrMotionBlk->SmallState = 0;
            ptrMotionBlk->MotionState = S_SINGLE_OPEN_M1M2;
            ptrMotionBlk->MotionCmdCode = 0;  //������ִ�У�����0
          }
        }
        break;

      case 16:  // M1M2 close
        if (ptrMotionBlk->SysState == SYS_WORKING) {
          if (ptrMotionBlk->MotionState == S_MOTION_IDLE) {
            ptrMotionBlk->SmallState = 0;
            ptrMotionBlk->MotionState = S_SINGLE_CLOSE_M1M2;
            ptrMotionBlk->MotionCmdCode = 0;  //������ִ�У�����0
          }
        }
        break;

      case 17:  // M3M4 open
        if (ptrMotionBlk->SysState == SYS_WORKING) {
          if (ptrMotionBlk->MotionState == S_MOTION_IDLE) {
            ptrMotionBlk->SmallState = 0;
            ptrMotionBlk->MotionState = S_SINGLE_OPEN_M3M4;
            ptrMotionBlk->MotionCmdCode = 0;  //������ִ�У�����0
          }
        }
        break;

      case 18:  // M3M4 close
        if (ptrMotionBlk->SysState == SYS_WORKING) {
          if (ptrMotionBlk->MotionState == S_MOTION_IDLE) {
            ptrMotionBlk->SmallState = 0;
            ptrMotionBlk->MotionState = S_SINGLE_CLOSE_M3M4;
            ptrMotionBlk->MotionCmdCode = 0;  //������ִ�У�����0
          }
        }
        break;
      case 19:
        if (ptrMotionBlk->SysState == SYS_WORKING) {
          if (ptrMotionBlk->MotionState == S_MOTION_IDLE) {
            ptrMotionBlk->SmallState = 0;
            ptrMotionBlk->MotionState = S_SINGLE_EXTEND_M5M6M7;
            ptrMotionBlk->MotionCmdCode = 0;  //������ִ�У�����0
          }
        }
        break;
      case 20:
        if (ptrMotionBlk->SysState == SYS_WORKING) {
          if (ptrMotionBlk->MotionState == S_MOTION_IDLE) {
            ptrMotionBlk->SmallState = 0;
            ptrMotionBlk->MotionState = S_SINGLE_SHORTEN_M5M6M7;
            ptrMotionBlk->MotionCmdCode = 0;  //������ִ�У�����0
          }
        }
        break;
      case 21:
        NMT_Start(0);
        delay_us(TIME_INTERVAL_US);
        delay_us(TIME_INTERVAL_US);
        delay_us(TIME_INTERVAL_US);
        StartMotor(1);  //�������нڵ�
        StartMotor(2);  //�������нڵ�
                        //	StartMotor(3);		//�������нڵ�
                        //	StartMotor(4);		//�������нڵ�
                        //	StartMotor(5);		//�������нڵ�
                        //	StartMotor(6);		//�������нڵ�
                        //	StartMotor(7);		//�������нڵ�

        ptrMotionBlk->MotionCmdCode = 0;
        break;
      case 22:
        StopMotor(1);  //�ر����нڵ�
        StopMotor(2);  //�ر����нڵ�
        StopMotor(3);  //�ر����нڵ�
        StopMotor(4);  //�ر����нڵ�
        StopMotor(5);  //�ر����нڵ�
        StopMotor(6);  //�ر����нڵ�
        StopMotor(7);  //�ر����нڵ�
        delay_us(TIME_INTERVAL_US);
        delay_us(TIME_INTERVAL_US);
        delay_us(TIME_INTERVAL_US);
        NMT_PreSTA(0);

        ptrMotionBlk->MotionCmdCode = 0;
      case 23:
        NMT_Start(0);

        ptrMotionBlk->MotionCmdCode = 0;
        break;
      case 24:
        StartMotor(1);  //�������нڵ�
        StartMotor(2);  //�������нڵ�
        StartMotor(3);  //�������нڵ�
        StartMotor(4);  //�������нڵ�
        StartMotor(5);  //�������нڵ�
        StartMotor(6);  //�������нڵ�
        StartMotor(7);  //�������нڵ�

        ptrMotionBlk->MotionCmdCode = 0;
        break;
      case 25:
        StopMotor(1);  //�ر����нڵ�
        StopMotor(2);  //�ر����нڵ�
        StopMotor(3);  //�ر����нڵ�
        StopMotor(4);  //�ر����нڵ�
        StopMotor(5);  //�ر����нڵ�
        StopMotor(6);  //�ر����нڵ�
        StopMotor(7);  //�ر����нڵ�

        ptrMotionBlk->MotionCmdCode = 0;
        break;
      case 26:
        NMT_PreSTA(0);

        ptrMotionBlk->MotionCmdCode = 0;
        break;

      case 27:
        SetMotorCurrentLimit(ptrMotionBlk->DebugSlaveId,
                             ptrMotionBlk->DebugMaxIqSet);
        ptrMotionBlk->MotionCmdCode = 0;
        break;

      case 28:
        SetMotorSpd(ptrMotionBlk->DebugSlaveId, ptrMotionBlk->DebugSpdSet);
        ptrMotionBlk->MotionCmdCode = 0;
        break;

      case 29:
        SetMotorAbsPos(
            ptrMotionBlk->DebugSlaveId,
            ((ptrServ[ptrMotionBlk->DebugSlaveId]->PosPV) +
             ((s32)ptrMotionBlk->DebugPosSet * ptrCfgMotionPar->PosFactor)));
        delay_us(TIME_INTERVAL_US);
        SetMotorCtrlword(ptrMotionBlk->DebugSlaveId, 0x000F);
        delay_us(TIME_INTERVAL_US);
        CMD_SYNC();
        ptrMotionBlk->MotionCmdCode = 0;
        break;
      case 30:
        SetMotorAbsPos(
            ptrMotionBlk->DebugSlaveId,
            ((ptrServ[ptrMotionBlk->DebugSlaveId]->PosPV) -
             ((s32)ptrMotionBlk->DebugPosSet * ptrCfgMotionPar->PosFactor)));
        delay_us(TIME_INTERVAL_US);
        SetMotorCtrlword(ptrMotionBlk->DebugSlaveId, 0x000F);
        delay_us(TIME_INTERVAL_US);
        CMD_SYNC();
        ptrMotionBlk->MotionCmdCode = 0;
        break;
      case 31:
        ptrMotionBlk->DebugSlaveId = 1;
        SetMotorAbsPos(
            ptrMotionBlk->DebugSlaveId,
            ((ptrServ[ptrMotionBlk->DebugSlaveId]->PosPV) +
             ((s32)ptrMotionBlk->DebugPosSet * ptrCfgMotionPar->PosFactor)));
        delay_us(TIME_INTERVAL_US);
        SetMotorCtrlword(ptrMotionBlk->DebugSlaveId, 0x000F);

        ptrMotionBlk->DebugSlaveId = 2;
        SetMotorAbsPos(
            ptrMotionBlk->DebugSlaveId,
            ((ptrServ[ptrMotionBlk->DebugSlaveId]->PosPV) +
             ((s32)ptrMotionBlk->DebugPosSet * ptrCfgMotionPar->PosFactor)));
        delay_us(TIME_INTERVAL_US);
        SetMotorCtrlword(ptrMotionBlk->DebugSlaveId, 0x000F);
        ptrMotionBlk->MotionCmdCode = 0;
        break;
      case 32:
        ptrMotionBlk->DebugSlaveId = 1;
        SetMotorAbsPos(
            ptrMotionBlk->DebugSlaveId,
            ((ptrServ[ptrMotionBlk->DebugSlaveId]->PosPV) -
             ((s32)ptrMotionBlk->DebugPosSet * ptrCfgMotionPar->PosFactor)));
        delay_us(TIME_INTERVAL_US);
        SetMotorCtrlword(ptrMotionBlk->DebugSlaveId, 0x000F);

        ptrMotionBlk->DebugSlaveId = 2;
        SetMotorAbsPos(
            ptrMotionBlk->DebugSlaveId,
            ((ptrServ[ptrMotionBlk->DebugSlaveId]->PosPV) -
             ((s32)ptrMotionBlk->DebugPosSet * ptrCfgMotionPar->PosFactor)));
        delay_us(TIME_INTERVAL_US);
        SetMotorCtrlword(ptrMotionBlk->DebugSlaveId, 0x000F);
        ptrMotionBlk->MotionCmdCode = 0;
        break;
      case 35:
        ptrMotionBlk->DebugSlaveId = 5;
        SetMotorAbsPos(ptrMotionBlk->DebugSlaveId,
                       ((ptrServ[ptrMotionBlk->DebugSlaveId]->PosPV) +
                        ((s32)ptrMotionBlk->DebugPosSet)));

        delay_us(TIME_INTERVAL_US);
        SetMotorCtrlword(ptrMotionBlk->DebugSlaveId, 0x000F);
        ptrMotionBlk->MotionCmdCode = 0;

        break;
      case 36:
        ptrMotionBlk->DebugSlaveId = 6;
        SetMotorAbsPos(ptrMotionBlk->DebugSlaveId,
                       ((ptrServ[ptrMotionBlk->DebugSlaveId]->PosPV) +
                        ((s32)ptrMotionBlk->DebugPosSet)));

        delay_us(TIME_INTERVAL_US);
        SetMotorCtrlword(ptrMotionBlk->DebugSlaveId, 0x000F);

        ptrMotionBlk->MotionCmdCode = 0;

        break;
      case 37:
        ptrMotionBlk->DebugSlaveId = 7;
        SetMotorAbsPos(ptrMotionBlk->DebugSlaveId,
                       ((ptrServ[ptrMotionBlk->DebugSlaveId]->PosPV) +
                        ((s32)ptrMotionBlk->DebugPosSet)));

        delay_us(TIME_INTERVAL_US);
        SetMotorCtrlword(ptrMotionBlk->DebugSlaveId, 0x000F);

        ptrMotionBlk->MotionCmdCode = 0;
        break;
      default:
        break;
    }

    if (ptrMotionBlk->CmdRepState == 0x0000)  //
    {
      if (ptrMotionBlk->MotionCmdCode == 0)
        ptrMotionBlk->CmdRepState = 0x0004;  //������Ӧ����4
      else {
        ptrMotionBlk->CmdRepState = 0x0002;  //û��Ӧ��2
                                             // ptrMotionBlk->MotionCmdCode = 0;
      }
    }
  }
}

void MotionCtrl(void) {
  u8 SlaveID;
  switch (ptrMotionBlk->MotionState) {
    case S_MOTION_IDLE:
      ptrMotionBlk->MotionState = S_MOTION_IDLE;
      break;

    case S_MOTION_OPEN_M1M2:
      if ((ptrMotionBlk->StatusWord & BIT_ROBOT_EN) != 0) {
        if ((ptrMotionBlk->StatusWord & BIT_IM_PAUSE) == 0) {  //����ִ��
          Motion_Open_M1M2((u16 *)&ptrMotionBlk->SmallState);  //��������
        } else {                                               //��ͣ
          SlaveID = 1;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 2;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          delay_us(TIME_INTERVAL_US);
          SlaveID = 1;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 2;
          SetMotorCtrlword(SlaveID, 0x000F);

          ptrMotionBlk->MotionStateLast =
              ptrMotionBlk->MotionState;  //�´λ��ǴӸ�״̬��ʼ������������óɿ�ѡ��?
          ptrMotionBlk->MotionStateNext = CalcuNextState(
              ptrMotionBlk->MotionState, (ptrMotionBlk->StatusWord & BIT_DIR));
          ptrMotionBlk->MotionState = S_MOTION_IM_PAUSE;
        }

        if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
          ptrMotionBlk->MotionStateLast = ptrMotionBlk->MotionState;

          ptrMotionBlk->MotionStateNext = CalcuNextState(
              ptrMotionBlk->MotionState, (ptrMotionBlk->StatusWord & BIT_DIR));

          if (ptrMotionBlk->MotionMode == 1)  // �ֶ�ģʽ
          {                                   //��ͣģʽ
            ptrMotionBlk->MotionState = S_MOTION_PAUSE;
            // ptrMotionBlk->SmallState = 0; // ����ͣ״̬��0���Ա�ɿ���ĳЩ���?
          } else {  //�Զ�������һ״̬
            ptrMotionBlk->MotionState = ptrMotionBlk->MotionStateNext;
            ptrMotionBlk->SmallState = 0;
          }
        }
      } else
        ;  //������һ��������
      break;

    case S_MOTION_CLOSE_M1M2:
      if ((ptrMotionBlk->StatusWord & BIT_ROBOT_EN) != 0) {
        if ((ptrMotionBlk->StatusWord & BIT_IM_PAUSE) == 0) {  //����ִ��
          Motion_Close_M1M2((u16 *)&ptrMotionBlk->SmallState);
        } else {  //��ͣ
          SlaveID = 1;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 2;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          delay_us(TIME_INTERVAL_US);
          SlaveID = 1;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 2;
          SetMotorCtrlword(SlaveID, 0x000F);

          ptrMotionBlk->MotionStateLast =
              ptrMotionBlk->MotionState;  //�´λ��ǴӸ�״̬��ʼ������������óɿ�ѡ��?
          ptrMotionBlk->MotionStateNext = CalcuNextState(
              ptrMotionBlk->MotionState, (ptrMotionBlk->StatusWord & BIT_DIR));
          ptrMotionBlk->MotionState = S_MOTION_IM_PAUSE;
        }

        if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
          if ((ptrMotionBlk->StatusWord & BIT_DIR) == 0) {
            if (ptrMotionBlk->StepsLeft != 0) {
              ptrMotionBlk->StepsLeft--;
            }
          }

          if ((ptrMotionBlk->StatusWord & BIT_SOFT_STOP) != 0) {
            ptrMotionBlk->MotionStateLast = ptrMotionBlk->MotionState;
            ptrMotionBlk->MotionState = S_MOTION_IDLE;
            ptrMotionBlk->MotionStateNext = S_MOTION_IDLE;
            ptrMotionBlk->SmallState = 0;
          } else if (ptrMotionBlk->StepsLeft == 0) {
            ptrMotionBlk->MotionStateLast = ptrMotionBlk->MotionState;
            ptrMotionBlk->MotionState = S_MOTION_IDLE;
            ptrMotionBlk->MotionStateNext = S_MOTION_IDLE;
            ptrMotionBlk->SmallState = 0;
          } else {
            ptrMotionBlk->MotionStateLast = ptrMotionBlk->MotionState;

            /*if(ptrMotionBlk->MotionDir == 1)  //������ض�λ�ø���һ�·���״�?λ
                    {
                            ptrMotionBlk->StatusWord |= BIT_DIR;
                    }
                    else
                    {
                            ptrMotionBlk->StatusWord &= ~BIT_DIR;
                    }*/
            ptrMotionBlk->MotionStateNext =
                CalcuNextState(ptrMotionBlk->MotionState,
                               (ptrMotionBlk->StatusWord & BIT_DIR));

            if (ptrMotionBlk->MotionMode == 1)  // �ֶ�ģʽ
            {                                   //��ͣģʽ
              ptrMotionBlk->MotionState = S_MOTION_PAUSE;
              // ptrMotionBlk->SmallState = 0; // ����ͣ״̬��0���Ա�ɿ���ĳЩ���?
            } else {  //�Զ�������һ״̬
              ptrMotionBlk->MotionState = ptrMotionBlk->MotionStateNext;
              ptrMotionBlk->SmallState = 0;
            }
          }
        }
      } else
        ;  //������һ��������
      break;

    case S_MOTION_OPEN_M3M4:
      if ((ptrMotionBlk->StatusWord & BIT_ROBOT_EN) != 0) {
        if ((ptrMotionBlk->StatusWord & BIT_IM_PAUSE) == 0) {  //����ִ��
          Motion_Open_M3M4((u16 *)&ptrMotionBlk->SmallState);
        } else {  //��ͣ
          SlaveID = 3;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 4;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          delay_us(TIME_INTERVAL_US);
          SlaveID = 3;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 4;
          SetMotorCtrlword(SlaveID, 0x000F);

          ptrMotionBlk->MotionStateLast =
              ptrMotionBlk->MotionState;  //�´λ��ǴӸ�״̬��ʼ������������óɿ�ѡ��?
          ptrMotionBlk->MotionStateNext = CalcuNextState(
              ptrMotionBlk->MotionState, (ptrMotionBlk->StatusWord & BIT_DIR));
          ptrMotionBlk->MotionState = S_MOTION_IM_PAUSE;
        }

        if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
          ptrMotionBlk->MotionStateLast = ptrMotionBlk->MotionState;

          ptrMotionBlk->MotionStateNext = CalcuNextState(
              ptrMotionBlk->MotionState, (ptrMotionBlk->StatusWord & BIT_DIR));

          if (ptrMotionBlk->MotionMode == 1)  // �ֶ�ģʽ
          {                                   //��ͣģʽ
            ptrMotionBlk->MotionState = S_MOTION_PAUSE;
            // ptrMotionBlk->SmallState = 0; // ����ͣ״̬��0���Ա�ɿ���ĳЩ���?
          } else {  //�Զ�������һ״̬
            ptrMotionBlk->MotionState = ptrMotionBlk->MotionStateNext;
            ptrMotionBlk->SmallState = 0;
          }
        }
      } else
        ;  //������һ��������
      break;

    case S_MOTION_CLOSE_M3M4:
      if ((ptrMotionBlk->StatusWord & BIT_ROBOT_EN) != 0) {
        if ((ptrMotionBlk->StatusWord & BIT_IM_PAUSE) == 0) {  //����ִ��
          Motion_Close_M3M4((u16 *)&ptrMotionBlk->SmallState);
        } else {  //��ͣ
          SlaveID = 3;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 4;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          delay_us(TIME_INTERVAL_US);
          SlaveID = 3;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 4;
          SetMotorCtrlword(SlaveID, 0x000F);

          ptrMotionBlk->MotionStateLast =
              ptrMotionBlk->MotionState;  //�´λ��ǴӸ�״̬��ʼ������������óɿ�ѡ��?
          ptrMotionBlk->MotionStateNext = CalcuNextState(
              ptrMotionBlk->MotionState, (ptrMotionBlk->StatusWord & BIT_DIR));
          ptrMotionBlk->MotionState = S_MOTION_IM_PAUSE;
        }

        if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
          if ((ptrMotionBlk->StatusWord & BIT_DIR) != 0) {
            if (ptrMotionBlk->StepsLeft != 0) {
              ptrMotionBlk->StepsLeft--;
            }
          }

          if ((ptrMotionBlk->StatusWord & BIT_SOFT_STOP) != 0) {
            ptrMotionBlk->MotionStateLast = ptrMotionBlk->MotionState;
            ptrMotionBlk->MotionState = S_MOTION_IDLE;
            ptrMotionBlk->MotionStateNext = S_MOTION_IDLE;
            ptrMotionBlk->SmallState = 0;
          } else if (ptrMotionBlk->StepsLeft == 0) {
            ptrMotionBlk->MotionStateLast = ptrMotionBlk->MotionState;
            ptrMotionBlk->MotionState = S_MOTION_IDLE;
            ptrMotionBlk->MotionStateNext = S_MOTION_IDLE;
            ptrMotionBlk->SmallState = 0;
          } else {
            ptrMotionBlk->MotionStateLast = ptrMotionBlk->MotionState;

            /*if(ptrMotionBlk->MotionDir == 1)  //������ض�λ�ø���һ�·���״�?λ
                    {
                            ptrMotionBlk->StatusWord |= BIT_DIR;
                    }
                    else
                    {
                            ptrMotionBlk->StatusWord &= ~BIT_DIR;
                    }	*/
            ptrMotionBlk->MotionStateNext =
                CalcuNextState(ptrMotionBlk->MotionState,
                               (ptrMotionBlk->StatusWord & BIT_DIR));

            if (ptrMotionBlk->MotionMode == 1)  // �ֶ�ģʽ
            {                                   //��ͣģʽ
              ptrMotionBlk->MotionState = S_MOTION_PAUSE;
              // ptrMotionBlk->SmallState = 0; // ����ͣ״̬��0���Ա�ɿ���ĳЩ���?
            } else {  //�Զ�������һ״̬
              ptrMotionBlk->MotionState = ptrMotionBlk->MotionStateNext;
              ptrMotionBlk->SmallState = 0;
            }
          }
        }
      } else
        ;  //������һ��������
      break;

    case S_MOTION_EXTEND_M5M6M7:
      if ((ptrMotionBlk->StatusWord & BIT_ROBOT_EN) != 0) {
        if ((ptrMotionBlk->StatusWord & BIT_IM_PAUSE) == 0) {  //����ִ��
          Motion_Extend_M5M6M7((u16 *)&ptrMotionBlk->SmallState);
        } else {  //��ͣ
          SlaveID = 5;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 6;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 7;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);

          delay_us(TIME_INTERVAL_US);
          SlaveID = 5;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 6;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 7;
          SetMotorCtrlword(SlaveID, 0x000F);

          ptrMotionBlk->MotionStateLast =
              ptrMotionBlk->MotionState;  //�´λ��ǴӸ�״̬��ʼ������������óɿ�ѡ��?
          ptrMotionBlk->MotionStateNext = CalcuNextState(
              ptrMotionBlk->MotionState, (ptrMotionBlk->StatusWord & BIT_DIR));
          ptrMotionBlk->MotionState = S_MOTION_IM_PAUSE;
        }

        if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
          ptrMotionBlk->MotionStateLast = ptrMotionBlk->MotionState;

          ptrCfgMotionPar->RunSteps =
              ptrCfgMotionPar->RunSteps + 1;  //����������1��

          ptrMotionBlk->MotionStateNext = CalcuNextState(
              ptrMotionBlk->MotionState, (ptrMotionBlk->StatusWord & BIT_DIR));

          if (ptrMotionBlk->MotionMode == 1)  // �ֶ�ģʽ
          {                                   //��ͣģʽ
            ptrMotionBlk->MotionState = S_MOTION_PAUSE;
            // ptrMotionBlk->SmallState = 0; // ����ͣ״̬��0���Ա�ɿ���ĳЩ���?
          } else {  //�Զ�������һ״̬
            ptrMotionBlk->MotionState = ptrMotionBlk->MotionStateNext;
            ptrMotionBlk->SmallState = 0;
          }
        }
      } else
        ;  //������һ��������
      break;

    case S_MOTION_SHORTEN_M5M6M7:
      if ((ptrMotionBlk->StatusWord & BIT_ROBOT_EN) != 0) {
        if ((ptrMotionBlk->StatusWord & BIT_IM_PAUSE) == 0) {  //����ִ��
          Motion_Shorten_M5M6M7((u16 *)&ptrMotionBlk->SmallState);
        } else {  //��ͣ
          SlaveID = 5;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 6;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 7;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);

          delay_us(TIME_INTERVAL_US);
          SlaveID = 5;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 6;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 7;
          SetMotorCtrlword(SlaveID, 0x000F);

          ptrMotionBlk->MotionStateLast =
              ptrMotionBlk->MotionState;  //�´λ��ǴӸ�״̬��ʼ������������óɿ�ѡ��?
          ptrMotionBlk->MotionStateNext = CalcuNextState(
              ptrMotionBlk->MotionState, (ptrMotionBlk->StatusWord & BIT_DIR));
          ptrMotionBlk->MotionState = S_MOTION_IM_PAUSE;
        }

        if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
          ptrMotionBlk->MotionStateLast = ptrMotionBlk->MotionState;

          ptrMotionBlk->MotionStateNext = CalcuNextState(
              ptrMotionBlk->MotionState, (ptrMotionBlk->StatusWord & BIT_DIR));

          if (ptrMotionBlk->MotionMode == 1)  // �ֶ�ģʽ
          {                                   //��ͣģʽ
            ptrMotionBlk->MotionState = S_MOTION_PAUSE;
            // ptrMotionBlk->SmallState = 0; // ����ͣ״̬��0���Ա�ɿ���ĳЩ���?
          } else {  //�Զ�������һ״̬
            ptrMotionBlk->MotionState = ptrMotionBlk->MotionStateNext;
            ptrMotionBlk->SmallState = 0;
          }
        }
      } else
        ;  //������һ��������
      break;

    case S_MOTION_HOME_M1M2:
      if ((ptrMotionBlk->StatusWord & BIT_ROBOT_EN) != 0) {
        if ((ptrMotionBlk->StatusWord & BIT_IM_PAUSE) == 0) {  //����ִ��
          Motion_Home_M1M2((u16 *)&ptrMotionBlk->SmallState);
        } else {  //��ͣ
          SlaveID = 1;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 2;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          delay_us(TIME_INTERVAL_US);
          SlaveID = 1;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 2;
          SetMotorCtrlword(SlaveID, 0x000F);

          ptrMotionBlk->MotionStateLast =
              ptrMotionBlk->MotionState;  //�´λ��ǴӸ�״̬��ʼ������������óɿ�ѡ��?
          ptrMotionBlk->MotionStateNext = CalcuNextState(
              ptrMotionBlk->MotionState, (ptrMotionBlk->StatusWord & BIT_DIR));
          ptrMotionBlk->MotionState = S_MOTION_IM_PAUSE;
        }

        if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
          ptrMotionBlk->MotionState = S_MOTION_IDLE;
          ptrMotionBlk->StatusWord &= ~BIT_HOMING_BUSY;
        }
      } else
        ;  //������һ��������
      break;

    case S_MOTION_HOME_M3M4:
      if ((ptrMotionBlk->StatusWord & BIT_ROBOT_EN) != 0) {
        if ((ptrMotionBlk->StatusWord & BIT_IM_PAUSE) == 0) {  //����ִ��
          Motion_Home_M3M4((u16 *)&ptrMotionBlk->SmallState);
        } else {  //��ͣ
          SlaveID = 3;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 4;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          delay_us(TIME_INTERVAL_US);
          SlaveID = 3;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 4;
          SetMotorCtrlword(SlaveID, 0x000F);

          ptrMotionBlk->MotionStateLast =
              ptrMotionBlk->MotionState;  //�´λ��ǴӸ�״̬��ʼ������������óɿ�ѡ��?
          ptrMotionBlk->MotionStateNext = CalcuNextState(
              ptrMotionBlk->MotionState, (ptrMotionBlk->StatusWord & BIT_DIR));
          ptrMotionBlk->MotionState = S_MOTION_IM_PAUSE;
        }

        if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
          ptrMotionBlk->MotionState = S_MOTION_IDLE;
          ptrMotionBlk->StatusWord &= ~BIT_HOMING_BUSY;
        }
      } else
        ;  //������һ��������
      break;

    case S_MOTION_HOME_M5M6M7:
      if ((ptrMotionBlk->StatusWord & BIT_ROBOT_EN) != 0) {
        if ((ptrMotionBlk->StatusWord & BIT_IM_PAUSE) == 0) {  //����ִ��
          Motion_Home_M5M6M7((u16 *)&ptrMotionBlk->SmallState);
        } else {  //��ͣ
          SlaveID = 5;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 6;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 7;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          delay_us(TIME_INTERVAL_US);
          SlaveID = 5;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 6;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 7;
          SetMotorCtrlword(SlaveID, 0x000F);

          ptrMotionBlk->MotionStateLast =
              ptrMotionBlk->MotionState;  //�´λ��ǴӸ�״̬��ʼ������������óɿ�ѡ��?
          ptrMotionBlk->MotionStateNext = CalcuNextState(
              ptrMotionBlk->MotionState, (ptrMotionBlk->StatusWord & BIT_DIR));
          ptrMotionBlk->MotionState = S_MOTION_IM_PAUSE;
          ptrMotionBlk->StatusWord &= ~BIT_HOMING_BUSY;
        }

        if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
          ptrMotionBlk->MotionState = S_MOTION_IDLE;
          ptrMotionBlk->StatusWord &= ~BIT_HOMING_BUSY;
        }
      } else
        ;  //������һ��������
      break;

    case S_MOTION_HOME_ALL:
      if ((ptrMotionBlk->StatusWord & BIT_ROBOT_EN) != 0) {
        if ((ptrMotionBlk->StatusWord & BIT_IM_PAUSE) == 0) {  //����ִ��
          Motion_Home_All((u16 *)&ptrMotionBlk->SmallState);
        } else {  //��ͣ
          SlaveID = 1;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 2;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 3;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 4;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 5;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 6;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          SlaveID = 7;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          delay_us(TIME_INTERVAL_US);
          SlaveID = 1;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 2;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 3;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 4;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 5;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 6;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 7;
          SetMotorCtrlword(SlaveID, 0x000F);

          ptrMotionBlk->MotionStateLast =
              ptrMotionBlk->MotionState;  //�´λ��ǴӸ�״̬��ʼ������������óɿ�ѡ��?
          ptrMotionBlk->MotionStateNext = CalcuNextState(
              ptrMotionBlk->MotionState, (ptrMotionBlk->StatusWord & BIT_DIR));
          ptrMotionBlk->MotionState = S_MOTION_IM_PAUSE;
        }

        if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
          ptrMotionBlk->MotionState = S_MOTION_IDLE;
          ptrMotionBlk->StatusWord &= ~BIT_HOMING_BUSY;
        }
      } else
        ;  //������һ��������
      break;

    case S_MOTION_PAUSE:  //ִ����һ����������?
      if ((ptrMotionBlk->StatusWord & BIT_SOFT_STOP) != 0) {
        if ((ptrMotionBlk->MotionStateLast == S_MOTION_CLOSE_M1M2) ||
            (ptrMotionBlk->MotionStateLast == S_MOTION_CLOSE_M3M4)) {
          ptrMotionBlk->MotionState = ptrMotionBlk->SmallState = S_MOTION_IDLE;
          return;
        }
      }

      if (ptrMotionBlk->MotionMode == 1)  // �ֶ�ģʽ
      {
        if (ptrMotionBlk->MotionPause == 1) {
          ptrMotionBlk->MotionState = ptrMotionBlk->MotionStateNext;
          ptrMotionBlk->SmallState = 0;
          ptrMotionBlk->MotionPause = 0;
        } else
          ;
      } else {
        ptrMotionBlk->MotionState = ptrMotionBlk->MotionStateNext;
        ptrMotionBlk->SmallState = 0;
      }
      break;

    case S_MOTION_IM_PAUSE:
      if ((ptrMotionBlk->StatusWord & BIT_IM_PAUSE) == 0) {
        ptrMotionBlk->MotionState = ptrMotionBlk->MotionStateLast;
      } else {
        // ptrMotionBlk->MotionState = ptrMotionBlk->MotionState;
        //�ȴ�����ָ�������?
      }

      break;
    case S_MOTION_FAULT:
      break;
    case S_SINGLE_OPEN_M1M2:
      Motion_Open_M1M2((u16 *)&ptrMotionBlk->SmallState);
      if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
        ptrMotionBlk->MotionState = S_MOTION_IDLE;
      }
      break;
    case S_SINGLE_CLOSE_M1M2:
      Motion_Close_M1M2((u16 *)&ptrMotionBlk->SmallState);
      if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
        ptrMotionBlk->MotionState = S_MOTION_IDLE;
      }
      break;
    case S_SINGLE_OPEN_M3M4:
      Motion_Open_M3M4((u16 *)&ptrMotionBlk->SmallState);
      if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
        ptrMotionBlk->MotionState = S_MOTION_IDLE;
      }
      break;
    case S_SINGLE_CLOSE_M3M4:
      Motion_Close_M3M4((u16 *)&ptrMotionBlk->SmallState);
      if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
        ptrMotionBlk->MotionState = S_MOTION_IDLE;
      }
      break;
    case S_SINGLE_EXTEND_M5M6M7:
      Motion_Extend_M5M6M7((u16 *)&ptrMotionBlk->SmallState);
      if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
        ptrMotionBlk->MotionState = S_MOTION_IDLE;
      }
      break;
    case S_SINGLE_SHORTEN_M5M6M7:
      Motion_Shorten_M5M6M7((u16 *)&ptrMotionBlk->SmallState);
      if ((ptrMotionBlk->SmallState & BIT_STEP_FINISH) == BIT_STEP_FINISH) {
        ptrMotionBlk->MotionState = S_MOTION_IDLE;
      }
      break;
    default:
      break;
  }
}

void HandleSysState(void) {
  u16 i, times;

  //����ŷ�״�?
  i = (ptrServ[1]->StatusWord) | (ptrServ[2]->StatusWord) |
      (ptrServ[3]->StatusWord) | (ptrServ[4]->StatusWord) |
      (ptrServ[5]->StatusWord) | (ptrServ[6]->StatusWord) |
      (ptrServ[7]->StatusWord);
  if ((i & 0x0008) != 0)  // �ŷ�״̬��bit3 Ϊ������?
  {
    ptrMotionBlk->ErrCode |= SERVO_ERR;
  }

  switch (ptrMotionBlk->SysState) {
    case SYS_IDLE:
      //��ʼ������

      //
      //����Ҫ��1��delay()��ʱ
      delay_ms(2000);  // 10ms

      // u8 Sdo_WrU32(u8 SlaveID,u16 index,u8 subindex,u32 data) //д4�ֽ�
      Sdo_WrU32(1, 0x6083, 0, ptrCfgMotionPar->acc1);
      delay_ms(2);
      Sdo_WrU32(1, 0x6084, 0, ptrCfgMotionPar->acc1);
      delay_ms(2);
      Sdo_WrU32(1, 0x6085, 0, ptrCfgMotionPar->acc1);
      delay_ms(2);

      Sdo_WrU32(2, 0x6083, 0, ptrCfgMotionPar->acc1);
      delay_ms(2);
      Sdo_WrU32(2, 0x6084, 0, ptrCfgMotionPar->acc1);
      delay_ms(2);
      Sdo_WrU32(2, 0x6085, 0, ptrCfgMotionPar->acc1);
      delay_ms(2);

      Sdo_WrU32(3, 0x6083, 0, ptrCfgMotionPar->acc1);
      delay_ms(2);
      Sdo_WrU32(3, 0x6084, 0, ptrCfgMotionPar->acc1);
      delay_ms(2);
      Sdo_WrU32(3, 0x6085, 0, ptrCfgMotionPar->acc1);
      delay_ms(2);

      Sdo_WrU32(4, 0x6083, 0, ptrCfgMotionPar->acc1);
      delay_ms(2);
      Sdo_WrU32(4, 0x6084, 0, ptrCfgMotionPar->acc1);
      delay_ms(2);
      Sdo_WrU32(4, 0x6085, 0, ptrCfgMotionPar->acc1);
      delay_ms(2);

      Sdo_WrU32(5, 0x6083, 0, ptrCfgMotionPar->acc2);
      delay_ms(2);
      Sdo_WrU32(5, 0x6084, 0, ptrCfgMotionPar->acc2);
      delay_ms(2);
      Sdo_WrU32(5, 0x6085, 0, ptrCfgMotionPar->acc2);
      delay_ms(2);

      Sdo_WrU32(6, 0x6083, 0, ptrCfgMotionPar->acc2);
      delay_ms(2);
      Sdo_WrU32(6, 0x6084, 0, ptrCfgMotionPar->acc2);
      delay_ms(2);
      Sdo_WrU32(6, 0x6085, 0, ptrCfgMotionPar->acc2);
      delay_ms(2);

      Sdo_WrU32(7, 0x6083, 0, ptrCfgMotionPar->acc2);
      delay_ms(2);
      Sdo_WrU32(7, 0x6084, 0, ptrCfgMotionPar->acc2);
      delay_ms(2);
      Sdo_WrU32(7, 0x6085, 0, ptrCfgMotionPar->acc2);
      delay_ms(2);

      ptrMotionBlk->SysState = SYS_INIT;

      break;
    case SYS_INIT:

      if (ptrMotionBlk->ErrCode != 0) {
        ptrMotionBlk->SysState = SYS_FAULT;
      } else  // if((ptrMotionBlk->StatusWord & BIT_INIT_OK) ==
              // BIT_INIT_OK)//��ʼ����ɣ�no err
      {
        times = 0;

        while ((ptrMotionBlk->StartNodeFlag != 0x00FE) && (times < 10)) {
          NMT_Start(0);   //�����ڵ�
          delay_ms(100);  // 10ms
          ptrMotionBlk->StartNodeFlag = 0;
          for (i = 1; i <= 7; i++)  //
          {
            ReadPos(i);
            delay_ms(2);  // 10ms
            if ((ptrServ[i]->StatusWord & BIT_REMOTE_START) != 0) {
              ptrMotionBlk->StartNodeFlag |= (0x0001 << i);
            }
          }
          times++;
        }

        if (times >= 10) {
          for (i = 1; i <= 7; i++) {
            if ((ptrMotionBlk->StartNodeFlag & (0x0001 << i)) == 0) {
              ptrServ[i]->ServErr |= START_FAIL;
            }
          }

          ptrMotionBlk->ErrCode |= START_FAIL;
          ptrMotionBlk->SysState = SYS_FAULT;
        } else {
          ptrMotionBlk->StatusWord |= BIT_INIT_OK;
          ptrMotionBlk->SysState = SYS_RDY;
        }
      }

      break;

    case SYS_RDY:
      if (ptrMotionBlk->ErrCode != 0) {
        ptrMotionBlk->SysState = SYS_FAULT;
      } else if ((ptrMotionBlk->StatusWord & BIT_ROBOT_EN) == 0) {
        if ((ptrMotionBlk->MotionCtrlWord & BIT_ROBOT_EN) != 0)  // ����ʹ������
        {
          StartMotor(1);  //�������нڵ�
          delay_ms(1);
          StartMotor(2);  //�������нڵ�
          delay_ms(1);
          StartMotor(3);  //�������нڵ�
          delay_ms(1);
          StartMotor(4);  //�������нڵ�
          delay_ms(1);

          StartMotor(5);  //�������нڵ�
          delay_ms(1);
          StartMotor(6);  //�������нڵ�
          delay_ms(1);
          StartMotor(7);  //�������нڵ�

          ptrMotionBlk->StatusWord |= BIT_ROBOT_EN;
          ptrMotionBlk->SysState = SYS_WORKING;
          ptrMotionBlk->MotionState = S_MOTION_IDLE;  // S_MOTION_PAUSE;

          if ((ptrMotionBlk->StatusWord & BIT_HOMING_FINISH) == 0) {
            ptrMotionBlk->MotionCmdCode = 14;
          }
        }
      } else
        ;
      break;

    case SYS_WORKING:
      if (ptrMotionBlk->ErrCode != 0) {
        ptrMotionBlk->SysState = SYS_FAULT;
        StopMotor(1);
        delay_ms(1);
        StopMotor(2);
        delay_ms(1);
        StopMotor(3);
        delay_ms(1);
        StopMotor(4);
        delay_ms(1);
        StopMotor(5);
        delay_ms(1);
        StopMotor(6);
        delay_ms(1);
        StopMotor(7);
        delay_ms(1);
        ptrMotionBlk->StatusWord &= ~BIT_ROBOT_EN;
      } else {
        if ((ptrMotionBlk->MotionCtrlWord & BIT_ROBOT_EN) == 0) {  //ʹ�������
          StopMotor(1);
          delay_ms(1);
          StopMotor(2);
          delay_ms(1);
          StopMotor(3);
          delay_ms(1);
          StopMotor(4);
          delay_ms(1);
          StopMotor(5);
          delay_ms(1);
          StopMotor(6);
          delay_ms(1);
          StopMotor(7);
          delay_ms(1);

          ptrMotionBlk->StatusWord &= ~BIT_ROBOT_EN;
          ptrMotionBlk->SysState = SYS_RDY;
          ptrMotionBlk->MotionState = S_MOTION_IDLE;

          ptrMotionBlk->StatusWord &= ~BIT_HOMING_FINISH;  //�����»���
          ptrMotionBlk->AutoCycleEn = 0;                   //��ֹʹ��
        } else {
          //�ȴ���������
        }
      }
      break;
    case SYS_FAULT:
      if (0)  //�����������?
      {
        ptrMotionBlk->SysState =
            SYS_RDY;  //���߸��ݴ��������������SYS_IDLE ״̬��can ����
      } else {
        ptrMotionBlk->SysState = SYS_FAULT;
      }
      break;

    default:
      break;
  }
}

void HandleMotion(void) {
  //	u8 SlaveID;

  if (ptrMotionBlk->SysState >= 2) {
    ReadPos(1);
    ReadPos(2);
    ReadPos(3);
    ReadPos(4);
    ReadPos(5);
    ReadPos(6);
    ReadPos(7);
  }

  //
  MotionCtrl();
  AutoMovePragram(ptrMotionBlk->AutoCycleEn, ptrMotionBlk->AutoCycleNum);
  HandleCmd();
  HandleSysState();
}

u16 CalcuNextState(
    u16 CurrentState,
    u16 dir) {  //���ݵ�ǰ״̬�����򣬼�����һ��״̬
  // ע��������bug��������ʱ�ĸ÷��򣬺���������
  switch (CurrentState) {
    case S_MOTION_OPEN_M1M2:
      if (dir != 0) {
        return (S_MOTION_EXTEND_M5M6M7);
      } else {
        return (S_MOTION_SHORTEN_M5M6M7);
      }
      break;
    case S_MOTION_EXTEND_M5M6M7:
      if (dir != 0) {
        return (S_MOTION_CLOSE_M1M2);
      } else {
        return (S_MOTION_CLOSE_M3M4);
      }
      break;
    case S_MOTION_CLOSE_M1M2:
      if (dir != 0) {
        return (S_MOTION_OPEN_M3M4);
      } else {
        return (S_MOTION_OPEN_M3M4);
      }
      break;
    case S_MOTION_OPEN_M3M4:
      if (dir != 0) {
        return (S_MOTION_SHORTEN_M5M6M7);
      } else {
        return (S_MOTION_EXTEND_M5M6M7);
      }
      break;
    case S_MOTION_SHORTEN_M5M6M7:
      if (dir != 0) {
        return (S_MOTION_CLOSE_M3M4);
      } else {
        return (S_MOTION_CLOSE_M1M2);
      }
      break;
    case S_MOTION_CLOSE_M3M4:
      if (dir != 0) {
        return (S_MOTION_OPEN_M1M2);
      } else {
        return (S_MOTION_OPEN_M1M2);
      }
      break;
    default:
      return (S_MOTION_IDLE);  //��λ��ԭʼ״̬
      break;
  }
}
void Motion_Open_M1M2(u16 *state) {
  u8 SlaveID = 1;
  static u16 finish_flag = 0;
  switch (*state) {
    case 0:
      Para[4] = 0;
      SlaveID = 1;
      // SetMotorSpd(SlaveID,M_OPEN_SPD1); //����ת��
      SetMotorSpd(SlaveID, ptrCfgMotionPar->CloseSpd*SPEED_FACTOR);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, MAX_IQ1);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosLocked -
           ptrCfgMotionPar->PosOpenLen *
               ptrCfgMotionPar->PosFactor));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?

      SlaveID = 2;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->CloseSpd*SPEED_FACTOR);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, MAX_IQ1);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosLocked -
           ptrCfgMotionPar->PosOpenLen *
               ptrCfgMotionPar->PosFactor));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?

      delay_us(TIME_INTERVAL_US);
      SlaveID = 1;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 2;
      SetMotorCtrlword(SlaveID, 0x000F);

      finish_flag = 0;
      *state = 1;  //״̬
      break;
    case 1:
      SlaveID = 1;
      if ((finish_flag & 0x0001) == 0) {
        if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0001;
        }
      } else
        ;

      SlaveID = 2;
      if ((finish_flag & 0x0002) == 0) {
        if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0002;
        }
      } else
        ;

      if (finish_flag == 0x0003) {
        ptrMotionBlk->M1orM3OpenState |= 0x0001;
        *state |= BIT_STEP_FINISH;  // finish
        Para[5] = Para[4];
      }

      break;
    case 2:
      break;
    case 3:
      break;
    default:
      break;
  }
  ptrMotionBlk->FuncInnerState = finish_flag;
}

void Motion_Open_M3M4(u16 *state) {
  u8 SlaveID = 3;
  static u16 finish_flag = 0;

  switch (*state) {
    case 0:
      SlaveID = 3;
      SetMotorSpd(
          SlaveID,
          ptrCfgMotionPar->CloseSpd*SPEED_FACTOR);  //����ת��
                                       // SetMotorSpd(SlaveID,M_OPEN_SPD1);
                                       // //����ת��

      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, MAX_IQ1);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosLocked -
           ptrCfgMotionPar->PosOpenLen *
               ptrCfgMotionPar->PosFactor));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?

      SlaveID = 4;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->CloseSpd*SPEED_FACTOR);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, MAX_IQ1);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosLocked -
           ptrCfgMotionPar->PosOpenLen *
               ptrCfgMotionPar->PosFactor));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?

      delay_us(TIME_INTERVAL_US);
      SlaveID = 3;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 4;
      SetMotorCtrlword(SlaveID, 0x000F);

      finish_flag = 0;
      *state = 1;  //״̬
      break;
    case 1:
      SlaveID = 3;
      if ((finish_flag & 0x0001) == 0) {
        if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0001;
        }
      } else
        ;

      SlaveID = 4;
      if ((finish_flag & 0x0002) == 0) {
        if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0002;
        }
      } else
        ;

      if (finish_flag == 0x0003) {
        ptrMotionBlk->M1orM3OpenState |= 0x0002;
        *state |= BIT_STEP_FINISH;  // finish
      }

      break;
    case 2:
      break;
    case 3:
      break;
    default:
      break;
  }
  ptrMotionBlk->FuncInnerState = finish_flag;
}

void Motion_Close_M1M2(u16 *state) {
  s32 RadiusErr1, RadiusErr2;
  u8 SlaveID = 1;
  static u16 finish_flag = 0;

  switch (*state) {
    case 0:

      SlaveID = 1;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->CloseSpd*SPEED_FACTOR);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq1Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked +
                               1000));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?

      SlaveID = 2;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->CloseSpd*SPEED_FACTOR);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq1Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked +
                               1000));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?

      delay_us(TIME_INTERVAL_US);
      SlaveID = 1;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 2;
      SetMotorCtrlword(SlaveID, 0x000F);
      finish_flag = 0;
      *state = 1;  //״̬
      break;
    case 1:
      SlaveID = 1;
      if ((finish_flag & 0x0001) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             100))  //ǰ��λ��С��100
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          // SetMotorAbsPos(SlaveID,(ptrServ[SlaveID]->PosPV + 20000));
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);  // 20171204
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��

          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0001;
        } else {
          if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
            SetMotorAbsPos(
                SlaveID,
                (ptrServ[SlaveID]->PosPV +
                 (ptrCfgMotionPar->DeltaPos *
                  ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
            delay_us(TIME_INTERVAL_US);
            SetMotorCtrlword(SlaveID, 0x000F);
            ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?
          }
        }
      } else
        ;

      SlaveID = 2;
      if ((finish_flag & 0x0002) == 0) {
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             100))  //ǰ��λ��С��100
        {
          // SetMotorAbsPos(SlaveID,(ptrServ[SlaveID]->PosPV + 20000));
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);  // 20171204
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��

          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0002;
        } else {
          if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
            SetMotorAbsPos(
                SlaveID,
                (ptrServ[SlaveID]->PosPV +
                 (ptrCfgMotionPar->DeltaPos *
                  ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
            delay_us(TIME_INTERVAL_US);
            SetMotorCtrlword(SlaveID, 0x000F);
            ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?
          }
        }
      } else
        ;

      if (finish_flag == 0x0003) {
        //
        RadiusErr1 = ptrServ[1]->PosPV - ptrServ[1]->PosLocked;
        RadiusErr2 = ptrServ[2]->PosPV - ptrServ[2]->PosLocked;
        if (abs(RadiusErr1 - RadiusErr2) < 1000) {  //��������ֱ��λ��
          ptrServ[1]->PosLocked = ptrServ[1]->PosPV;
          ptrServ[2]->PosLocked = ptrServ[2]->PosPV;
        }
        //
        ptrMotionBlk->M1orM3OpenState &= ~0x0001;
        *state |= BIT_STEP_FINISH;  // finish
      }

      break;
    case 2:
      break;
    case 3:
      break;
    default:
      break;
  }
  ptrMotionBlk->FuncInnerState = finish_flag;
}

void Motion_Close_M3M4(u16 *state) {
  s32 RadiusErr1;
  s32 RadiusErr2;

  u8 SlaveID = 3;
  static u16 finish_flag = 0;

  switch (*state) {
    case 0:
      SlaveID = 3;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->CloseSpd*SPEED_FACTOR);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq1Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked +
                               1000));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?

      SlaveID = 4;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->CloseSpd*SPEED_FACTOR);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq1Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked +
                               1000));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?

      delay_us(TIME_INTERVAL_US);
      SlaveID = 3;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 4;
      SetMotorCtrlword(SlaveID, 0x000F);
      finish_flag = 0;
      *state = 1;  //״̬
      break;
    case 1:
      SlaveID = 3;
      if ((finish_flag & 0x0001) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             100))  //ǰ��λ��С��100
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          // SetMotorAbsPos(SlaveID,(ptrServ[SlaveID]->PosPV + 10000));
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);  // 20171204
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��

          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0001;
        } else {
          if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
            SetMotorAbsPos(
                SlaveID,
                (ptrServ[SlaveID]->PosPV +
                 (ptrCfgMotionPar->DeltaPos *
                  ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
            delay_us(TIME_INTERVAL_US);
            SetMotorCtrlword(SlaveID, 0x000F);
            ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?
          }
        }
      } else
        ;

      SlaveID = 4;
      if ((finish_flag & 0x0002) == 0) {
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             100))  //ǰ��λ��С��100
        {
          // SetMotorAbsPos(SlaveID,(ptrServ[SlaveID]->PosPV + 10000));
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);  // 20171204
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��

          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0002;
        } else {
          if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
            SetMotorAbsPos(
                SlaveID,
                (ptrServ[SlaveID]->PosPV +
                 (ptrCfgMotionPar->DeltaPos *
                  ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
            delay_us(TIME_INTERVAL_US);
            SetMotorCtrlword(SlaveID, 0x000F);
            ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?
          }
        }
      } else
        ;

      if (finish_flag == 0x0003) {
        //
        RadiusErr1 = ptrServ[3]->PosPV - ptrServ[3]->PosLocked;
        RadiusErr2 = ptrServ[4]->PosPV - ptrServ[4]->PosLocked;
        if (abs(RadiusErr1 - RadiusErr2) < 1000) {  //��������ֱ��λ��
          ptrServ[3]->PosLocked = ptrServ[3]->PosPV;
          ptrServ[4]->PosLocked = ptrServ[4]->PosPV;
        }
        //
        ptrMotionBlk->M1orM3OpenState &= ~0x0002;
        *state |= BIT_STEP_FINISH;  // finish
      }

      break;
    case 2:
      break;
    case 3:
      break;
    default:
      break;
  }
  ptrMotionBlk->FuncInnerState = finish_flag;
}

void Motion_Extend_M5M6M7(u16 *state) {
  u8 SlaveID = 5;
  u16 ClimbLenTemp = 0;
  static u16 finish_flag = 0;

  static s32 pos_open2_locked = -1;
  s32 pos_left;

  switch (*state) {
    case 0:
#if 0
				SlaveID = 5;
				SetMotorSpd(SlaveID,ptrCfgMotionPar->ClimbSpd); //����M5ת��
				delay_us(TIME_INTERVAL_US);			
			
				SlaveID = 6;
				SetMotorSpd(SlaveID,ptrCfgMotionPar->ClimbSpd); //����M5ת��
				delay_us(TIME_INTERVAL_US);			
			
				SlaveID = 7;
				SetMotorSpd(SlaveID,ptrCfgMotionPar->ClimbSpd); //����M5ת��
				delay_us(TIME_INTERVAL_US);			
			
			  delay_ms(10);
			
				SlaveID = 5;
				SetMotorCurrentLimit(SlaveID,MAX_IQ2);//��������ȵ�������ȵ�����3��
				delay_us(TIME_INTERVAL_US);			
			
				SlaveID = 6;
				SetMotorCurrentLimit(SlaveID,MAX_IQ2);//��������ȵ�������ȵ�����3��
				delay_us(TIME_INTERVAL_US);				

				SlaveID = 7;
				SetMotorCurrentLimit(SlaveID,MAX_IQ2);//��������ȵ�������ȵ�����3��	
				delay_us(TIME_INTERVAL_US);				
			
				delay_ms(10);
				
				SlaveID = 5;
				SetMotorAbsPos(SlaveID,(ptrServ[SlaveID ]->PosLocked + ptrCfgMotionPar->PosClimbLen*ptrCfgMotionPar->PosFactor));			//�����п����Ǵ��ġ��Ǿ��� ��-��	
				ptrServ[SlaveID]->StatusWord &= ~0x0400; //�Ѷ�λ��ɸ��?
				delay_us(TIME_INTERVAL_US);	


				//���Ƹ�2���ƶ��ľ���
				if(ptrCfgMotionPar->ClimbLenSameFlag != 0)
				{//������0					
					ClimbLenTemp =  ptrCfgMotionPar->PosClimbLen; //���Ƹ�1�ĳ���
				}
				else
				{
					ClimbLenTemp =  ptrCfgMotionPar->PosClimbLen2; //���Ƹ�1�ĳ���
				}
				
				SlaveID = 6;
				SetMotorAbsPos(SlaveID,(ptrServ[SlaveID ]->PosLocked + ClimbLenTemp*ptrCfgMotionPar->PosFactor));			//�����п����Ǵ��ġ��Ǿ��� ��-��	
				ptrServ[SlaveID]->StatusWord &= ~0x0400; //�Ѷ�λ��ɸ��?
				delay_us(TIME_INTERVAL_US);				

				//���Ƹ�3���ƶ��ľ���
				if(ptrCfgMotionPar->ClimbLenSameFlag != 0)
				{//������0					
					ClimbLenTemp =  ptrCfgMotionPar->PosClimbLen; //���Ƹ�1�ĳ���
				}
				else
				{
					ClimbLenTemp =  ptrCfgMotionPar->PosClimbLen3; //���Ƹ�1�ĳ���
				}				
				SlaveID = 7;
				SetMotorAbsPos(SlaveID,(ptrServ[SlaveID ]->PosLocked + ClimbLenTemp*ptrCfgMotionPar->PosFactor));			//�����п����Ǵ��ġ��Ǿ��� ��-��	
				ptrServ[SlaveID]->StatusWord &= ~0x0400; //�Ѷ�λ��ɸ��?	
				delay_us(TIME_INTERVAL_US);				
				
			  delay_us(TIME_INTERVAL_US);					
				SlaveID = 5;
				SetMotorCtrlword(SlaveID,0x000F);			
				SlaveID = 6;
				SetMotorCtrlword(SlaveID,0x000F);
				SlaveID = 7;
				SetMotorCtrlword(SlaveID,0x000F);
		
				
        finish_flag = 0;
				*state = 1; //״̬

#else

      SlaveID = 5;
      delay_us(TIME_INTERVAL_US);
      SetMotorSpd(SlaveID, ptrCfgMotionPar->ClimbSpd*SPEED_FACTOR);  //����M5ת��
      delay_us(TIME_INTERVAL_US);
      // SetMotorCurrentLimit(SlaveID,MAX_IQ2);//��������ȵ�������ȵ�����3��
      // delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosLocked +
           ptrCfgMotionPar->PosClimbLen *
               ptrCfgMotionPar->PosFactor));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?
      delay_us(TIME_INTERVAL_US);

      SlaveID = 6;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->ClimbSpd*SPEED_FACTOR);  //����M6ת��
      delay_us(TIME_INTERVAL_US);
      //	SetMotorCurrentLimit(SlaveID,MAX_IQ2);//��������ȵ�������ȵ�����3��
      // delay_us(TIME_INTERVAL_US);

      //���Ƹ�2���ƶ��ľ���
      if (ptrCfgMotionPar->ClimbLenSameFlag != 0) {  //������0
        ClimbLenTemp = ptrCfgMotionPar->PosClimbLen;  //���Ƹ�1�ĳ���
      } else {
        ClimbLenTemp = ptrCfgMotionPar->PosClimbLen2;  //���Ƹ�1�ĳ���
      }
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosLocked +
           ClimbLenTemp *
               ptrCfgMotionPar->PosFactor));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?
      delay_us(TIME_INTERVAL_US);

      SlaveID = 7;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->ClimbSpd*SPEED_FACTOR);  //����M7ת��
      delay_us(TIME_INTERVAL_US);
      // SetMotorCurrentLimit(SlaveID,MAX_IQ2);//��������ȵ�������ȵ�����3��
      // delay_us(TIME_INTERVAL_US);
      //���Ƹ�3���ƶ��ľ���
      if (ptrCfgMotionPar->ClimbLenSameFlag != 0) {  //������0
        ClimbLenTemp = ptrCfgMotionPar->PosClimbLen;  //���Ƹ�1�ĳ���
      } else {
        ClimbLenTemp = ptrCfgMotionPar->PosClimbLen3;  //���Ƹ�1�ĳ���
      }
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosLocked +
           ClimbLenTemp *
               ptrCfgMotionPar->PosFactor));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?
      delay_us(TIME_INTERVAL_US);

      //
      /* 		if ((ptrMotionBlk->M1orM3OpenState & 0x0001) != 0)
      {
              SlaveID = 1;
              SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
      (ptrCfgMotionPar->PosOpenLen + ptrCfgMotionPar->PosOpenLen2) *
      ptrCfgMotionPar->PosFactor)); ptrServ[SlaveID]->StatusWord &= ~0x0400;
              delay_us(TIME_INTERVAL_US);
              SlaveID = 2;
              SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
      (ptrCfgMotionPar->PosOpenLen + ptrCfgMotionPar->PosOpenLen2) *
      ptrCfgMotionPar->PosFactor)); ptrServ[SlaveID]->StatusWord &= ~0x0400;
              pos_open2_locked = 0xffffffff;
      }
      else if ((ptrMotionBlk->M1orM3OpenState & 0x0002) != 0)
      {
              SlaveID = 3;
              SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
      (ptrCfgMotionPar->PosOpenLen + ptrCfgMotionPar->PosOpenLen2) *
      ptrCfgMotionPar->PosFactor)); ptrServ[SlaveID]->StatusWord &= ~0x0400;
              delay_us(TIME_INTERVAL_US);
              SlaveID = 4;
              SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
      (ptrCfgMotionPar->PosOpenLen + ptrCfgMotionPar->PosOpenLen2) *
      ptrCfgMotionPar->PosFactor)); ptrServ[SlaveID]->StatusWord &= ~0x0400;
              pos_open2_locked = 0xffffffff;
      } */
      //
      delay_us(TIME_INTERVAL_US);
      SlaveID = 5;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 6;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 7;
      SetMotorCtrlword(SlaveID, 0x000F);
      //
      //
      if ((ptrMotionBlk->M1orM3OpenState & 0x0001) != 0) {
        SlaveID = 1;
        SetMotorCtrlword(SlaveID, 0x000F);
        SlaveID = 2;
        SetMotorCtrlword(SlaveID, 0x000F);
      } else if ((ptrMotionBlk->M1orM3OpenState & 0x0002) != 0) {
        SlaveID = 3;
        SetMotorCtrlword(SlaveID, 0x000F);
        SlaveID = 4;
        SetMotorCtrlword(SlaveID, 0x000F);
      }
      //
      finish_flag = 0;
      *state = 1;  //״̬

#endif
      break;
    case 1:
      //
      if ((ptrMotionBlk->M1orM3OpenState & 0x0001) != 0) {
        SlaveID = 1;
      } else if ((ptrMotionBlk->M1orM3OpenState & 0x0002) != 0) {
        SlaveID = 3;
      }

      if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {  //��M5��λ�����ж�
        if (pos_open2_locked == 0xffffffff) {
          pos_open2_locked = ptrServ[5]->PosPV -
                             ptrServ[5]->PosLocked;  // pos_open2_locked Ϊ��ֵ
        }
      }
      pos_left = (ptrServ[5]->PosLocked +
                  ptrCfgMotionPar->PosClimbLen * ptrCfgMotionPar->PosFactor) -
                 ptrServ[5]->PosPV;

      // if(pos_left < (s32)pos_open2_locked)
      if (pos_left <
          ((s32)pos_open2_locked + (ptrCfgMotionPar->ClimbSpd / 128) *
                                       (s32)(ptrCfgMotionPar->CompCoff))) {  //
        if ((ptrMotionBlk->M1orM3OpenState & 0x0001) != 0) {  //���ظ�����λ��
          SlaveID = 1;
          SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
                                   (ptrCfgMotionPar->PosOpenLen) *
                                       ptrCfgMotionPar->PosFactor));
          ptrServ[SlaveID]->StatusWord &= ~0x0400;
          delay_us(TIME_INTERVAL_US);
          SlaveID = 2;
          SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
                                   (ptrCfgMotionPar->PosOpenLen) *
                                       ptrCfgMotionPar->PosFactor));
          ptrServ[SlaveID]->StatusWord &= ~0x0400;
          pos_open2_locked = 0xffffffff;

          delay_us(TIME_INTERVAL_US);
          SlaveID = 1;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 2;
          SetMotorCtrlword(SlaveID, 0x000F);
        } else if ((ptrMotionBlk->M1orM3OpenState & 0x0002) != 0) {
          //���ظ�����λ��
          SlaveID = 3;
          SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
                                   (ptrCfgMotionPar->PosOpenLen) *
                                       ptrCfgMotionPar->PosFactor));
          ptrServ[SlaveID]->StatusWord &= ~0x0400;
          delay_us(TIME_INTERVAL_US);
          SlaveID = 4;
          SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
                                   (ptrCfgMotionPar->PosOpenLen) *
                                       ptrCfgMotionPar->PosFactor));
          ptrServ[SlaveID]->StatusWord &= ~0x0400;
          pos_open2_locked = 0xffffffff;

          delay_us(TIME_INTERVAL_US);
          SlaveID = 3;
          SetMotorCtrlword(SlaveID, 0x000F);
          SlaveID = 4;
          SetMotorCtrlword(SlaveID, 0x000F);
        }
        pos_open2_locked = 0;  // ���ü�������ָ��
                               //
      }
      //

      ptrMotionBlk->Pos6Pos5Err = (ptrServ[5]->PosPV - ptrServ[5]->PosLocked) -
                                  (ptrServ[6]->PosPV - ptrServ[6]->PosLocked);
      ptrMotionBlk->Pos7Pos5Err = (ptrServ[5]->PosPV - ptrServ[5]->PosLocked) -
                                  (ptrServ[7]->PosPV - ptrServ[7]->PosLocked);

      Para[290] = ptrMotionBlk->Pos6Pos5Err;
      Para[291] = ptrMotionBlk->Pos7Pos5Err;
      Para[292] = ptrMotionBlk->Pos7Pos5Err - ptrMotionBlk->Pos6Pos5Err;

      if ((abs(ptrMotionBlk->Pos6Pos5Err) > POS_ERR_MAX) ||
          (abs(ptrMotionBlk->Pos7Pos5Err) > POS_ERR_MAX) ||
          (abs(ptrMotionBlk->Pos7Pos5Err - ptrMotionBlk->Pos6Pos5Err) >
           POS_ERR_MAX)) {
        //����1��ֱ���ڴ˶�ʹ��
        //����2���޸����ǵ�ָ�������ֹͣ��

        // ȡ���Ƹ�ͬ������
        delay_us(TIME_INTERVAL_US);
        SlaveID = 5;
        SetMotorAbsPos(SlaveID, (ptrServ[5]->PosPV));

        delay_us(TIME_INTERVAL_US);
        SlaveID = 6;
        SetMotorAbsPos(SlaveID, (ptrServ[5]->PosPV - ptrServ[5]->PosLocked +
                                 ptrServ[6]->PosLocked));

        delay_us(TIME_INTERVAL_US);
        SlaveID = 7;
        SetMotorAbsPos(SlaveID, (ptrServ[5]->PosPV - ptrServ[5]->PosLocked +
                                 ptrServ[7]->PosLocked));

        ptrMotionBlk->ErrCode |= AXIS_SYN_ERR;  //��ͬ������

        delay_ms(5);
      }

      SlaveID = 5;
      if ((finish_flag & (0x0001 << SlaveID)) == 0) {
        if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= (0x0001 << SlaveID);
        }
      } else
        ;

      SlaveID = 6;
      if ((finish_flag & (0x0001 << SlaveID)) == 0) {
        if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= (0x0001 << SlaveID);
        }
      } else
        ;

      SlaveID = 7;
      if ((finish_flag & (0x0001 << SlaveID)) == 0) {
        if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= (0x0001 << SlaveID);
        }
      } else
        ;

      if (finish_flag == ((0x0001 << 5) | (0x0001 << 6) | (0x0001 << 7))) {
        *state |= BIT_STEP_FINISH;  // finish
      }

      break;
    case 2:
      break;
    case 3:
      break;
    default:
      break;
  }
  ptrMotionBlk->FuncInnerState = finish_flag;
}

void Motion_Shorten_M5M6M7(u16 *state) {
  u8 SlaveID = 5;
  static u16 finish_flag = 0;

  static s32 pos_open2_locked = -1;
  s32 pos_left;

  switch (*state) {
    case 0:
#if 0
				SlaveID = 5;
				SetMotorSpd(SlaveID,ptrCfgMotionPar->ClimbSpd); //����M5ת��		
				delay_us(TIME_INTERVAL_US);			
				
				SlaveID = 6;
				SetMotorSpd(SlaveID,ptrCfgMotionPar->ClimbSpd); //����M5ת��
				delay_us(TIME_INTERVAL_US);			
				
				SlaveID = 7;
				SetMotorSpd(SlaveID,ptrCfgMotionPar->ClimbSpd); //����M5ת��
				delay_us(TIME_INTERVAL_US);			
			
			  delay_ms(10);
			
				SlaveID = 5;
				SetMotorCurrentLimit(SlaveID,MAX_IQ2);//��������ȵ�������ȵ�����3��
				delay_us(TIME_INTERVAL_US);			
			
				SlaveID = 6;
				SetMotorCurrentLimit(SlaveID,MAX_IQ2);//��������ȵ�������ȵ�����3��
				delay_us(TIME_INTERVAL_US);				

				SlaveID = 7;
				SetMotorCurrentLimit(SlaveID,MAX_IQ2);//��������ȵ�������ȵ�����3��			
			
				delay_ms(10);			
			
				SlaveID = 5;
				SetMotorAbsPos(SlaveID,(ptrServ[SlaveID ]->PosLocked));			//�����п����Ǵ��ġ��Ǿ��� ��-��	
				ptrServ[SlaveID]->StatusWord &= ~0x0400; //�Ѷ�λ��ɸ��?
				delay_us(TIME_INTERVAL_US);				
			
				SlaveID = 6;
				SetMotorAbsPos(SlaveID,(ptrServ[SlaveID ]->PosLocked));			//�����п����Ǵ��ġ��Ǿ��� ��-��	
				ptrServ[SlaveID]->StatusWord &= ~0x0400; //�Ѷ�λ��ɸ��?
				delay_us(TIME_INTERVAL_US);				
			
				SlaveID = 7;	
				SetMotorAbsPos(SlaveID,(ptrServ[SlaveID ]->PosLocked));			//�����п����Ǵ��ġ��Ǿ��� ��-��	
				ptrServ[SlaveID]->StatusWord &= ~0x0400; //�Ѷ�λ��ɸ��?
				delay_us(TIME_INTERVAL_US);				
			
			
			  delay_us(TIME_INTERVAL_US);					
				SlaveID = 5;
				SetMotorCtrlword(SlaveID,0x000F);			
				SlaveID = 6;
				SetMotorCtrlword(SlaveID,0x000F);
				SlaveID = 7;
				SetMotorCtrlword(SlaveID,0x000F);		
				
        finish_flag = 0;
				*state = 1; //״̬

#else
      SlaveID = 5;
      delay_us(TIME_INTERVAL_US);
      SetMotorSpd(SlaveID, ptrCfgMotionPar->ClimbSpd*SPEED_FACTOR);  //����M5ת��
      delay_us(TIME_INTERVAL_US);
      //	SetMotorCurrentLimit(SlaveID,MAX_IQ2);//��������ȵ�������ȵ�����3��
      // delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID, (ptrServ[SlaveID]->PosLocked));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?
      delay_us(TIME_INTERVAL_US);

      SlaveID = 6;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->ClimbSpd*SPEED_FACTOR);  //����M6ת��
      delay_us(TIME_INTERVAL_US);
      //	SetMotorCurrentLimit(SlaveID,MAX_IQ2);//��������ȵ�������ȵ�����3��
      // delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID, (ptrServ[SlaveID]->PosLocked));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?
      delay_us(TIME_INTERVAL_US);

      SlaveID = 7;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->ClimbSpd*SPEED_FACTOR);  //����M7ת��
      delay_us(TIME_INTERVAL_US);
      //	SetMotorCurrentLimit(SlaveID,MAX_IQ2);//��������ȵ�������ȵ�����3��
      // delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID, (ptrServ[SlaveID]->PosLocked));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?
      delay_us(TIME_INTERVAL_US);

      /* if ((ptrMotionBlk->M1orM3OpenState & 0x0001) != 0)
      {
              SlaveID = 1;
              SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
      (ptrCfgMotionPar->PosOpenLen + ptrCfgMotionPar->PosOpenLen2) *
      ptrCfgMotionPar->PosFactor)); ptrServ[SlaveID]->StatusWord &= ~0x0400;
              delay_us(TIME_INTERVAL_US);
              SlaveID = 2;
              SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
      (ptrCfgMotionPar->PosOpenLen + ptrCfgMotionPar->PosOpenLen2) *
      ptrCfgMotionPar->PosFactor)); ptrServ[SlaveID]->StatusWord &= ~0x0400;
              pos_open2_locked = 0xffffffff;
      }
      else if ((ptrMotionBlk->M1orM3OpenState & 0x0002) != 0)
      {
              SlaveID = 3;
              SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
      (ptrCfgMotionPar->PosOpenLen + ptrCfgMotionPar->PosOpenLen2) *
      ptrCfgMotionPar->PosFactor)); ptrServ[SlaveID]->StatusWord &= ~0x0400;
              delay_us(TIME_INTERVAL_US);
              SlaveID = 4;
              SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
      (ptrCfgMotionPar->PosOpenLen + ptrCfgMotionPar->PosOpenLen2) *
      ptrCfgMotionPar->PosFactor)); ptrServ[SlaveID]->StatusWord &= ~0x0400;
              pos_open2_locked = 0xffffffff;
      } */

      delay_us(TIME_INTERVAL_US);
      SlaveID = 5;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 6;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 7;
      SetMotorCtrlword(SlaveID, 0x000F);

      //
      if ((ptrMotionBlk->M1orM3OpenState & 0x0001) != 0) {
        SlaveID = 1;
        SetMotorCtrlword(SlaveID, 0x000F);
        SlaveID = 2;
        SetMotorCtrlword(SlaveID, 0x000F);
      } else if ((ptrMotionBlk->M1orM3OpenState & 0x0002) != 0) {
        SlaveID = 3;
        SetMotorCtrlword(SlaveID, 0x000F);
        SlaveID = 4;
        SetMotorCtrlword(SlaveID, 0x000F);
      }
      //

      finish_flag = 0;
      *state = 1;  //״̬
#endif
      break;
    case 1:
      //
      if ((ptrMotionBlk->M1orM3OpenState & 0x0001) != 0) {
        SlaveID = 1;
      } else if ((ptrMotionBlk->M1orM3OpenState & 0x0002) != 0) {
        SlaveID = 3;
      }

      if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {  //��M5��λ�����ж�
        if (pos_open2_locked == 0xffffffff) {
          // pos_open2_locked = ptrServ[5]->PosPV - ptrServ[5]->PosLocked;
          // //pos_open2_locked Ϊ��ֵ
          pos_open2_locked =
              (ptrServ[5]->PosLocked +
               ptrCfgMotionPar->PosClimbLen * ptrCfgMotionPar->PosFactor) -
              ptrServ[5]->PosPV;
        }
      }
      // pos_left = (ptrServ[5]->PosLocked +
      // ptrCfgMotionPar->PosClimbLen*ptrCfgMotionPar->PosFactor)
      // -ptrServ[5]->PosPV;
      pos_left = ptrServ[5]->PosPV - ptrServ[5]->PosLocked;

      // if(pos_left < (s32)pos_open2_locked)
      if (pos_left <
          ((s32)pos_open2_locked + (ptrCfgMotionPar->ClimbSpd / 128) *
                                       (s32)(ptrCfgMotionPar->CompCoff))) {  //
        if ((ptrMotionBlk->M1orM3OpenState & 0x0001) != 0) {  //���ظ�����λ��
          // SlaveID = 1;
          // SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
          //                          (ptrCfgMotionPar->PosOpenLen) *
          //                              ptrCfgMotionPar->PosFactor));
          // ptrServ[SlaveID]->StatusWord &= ~0x0400;
          // delay_us(TIME_INTERVAL_US);
          // SlaveID = 2;
          // SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
          //                          (ptrCfgMotionPar->PosOpenLen) *
          //                              ptrCfgMotionPar->PosFactor));
          // ptrServ[SlaveID]->StatusWord &= ~0x0400;
          // pos_open2_locked = 0xffffffff;

          // delay_us(TIME_INTERVAL_US);
          // SlaveID = 1;
          // SetMotorCtrlword(SlaveID, 0x000F);
          // SlaveID = 2;
          // SetMotorCtrlword(SlaveID, 0x000F);
          // delay_us(TIME_INTERVAL_US);
        } else if ((ptrMotionBlk->M1orM3OpenState & 0x0002) != 0) {
          //���ظ�����λ��
          // SlaveID = 3;
          // SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
          //                          (ptrCfgMotionPar->PosOpenLen) *
          //                              ptrCfgMotionPar->PosFactor));
          // ptrServ[SlaveID]->StatusWord &= ~0x0400;
          // delay_us(TIME_INTERVAL_US);
          // SlaveID = 4;
          // SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked -
          //                          (ptrCfgMotionPar->PosFactor) *
          //                              ptrCfgMotionPar->PosFactor));
          // ptrServ[SlaveID]->StatusWord &= ~0x0400;
          // pos_open2_locked = 0xffffffff;

          // delay_us(TIME_INTERVAL_US);
          // SlaveID = 3;
          // SetMotorCtrlword(SlaveID, 0x000F);
          // SlaveID = 4;
          // SetMotorCtrlword(SlaveID, 0x000F);
          // delay_us(TIME_INTERVAL_US);
        }
        pos_open2_locked = 0;  // ���ü�������ָ��
                               //
      }
      //

      ptrMotionBlk->Pos6Pos5Err = (ptrServ[5]->PosPV - ptrServ[5]->PosLocked) -
                                  (ptrServ[6]->PosPV - ptrServ[6]->PosLocked);
      ptrMotionBlk->Pos7Pos5Err = (ptrServ[5]->PosPV - ptrServ[5]->PosLocked) -
                                  (ptrServ[7]->PosPV - ptrServ[7]->PosLocked);

      Para[290] = ptrMotionBlk->Pos6Pos5Err;
      Para[291] = ptrMotionBlk->Pos7Pos5Err;
      Para[292] = ptrMotionBlk->Pos7Pos5Err - ptrMotionBlk->Pos6Pos5Err;

      if ((abs(ptrMotionBlk->Pos6Pos5Err) > POS_ERR_MAX) ||
          (abs(ptrMotionBlk->Pos7Pos5Err) > POS_ERR_MAX) ||
          (abs(ptrMotionBlk->Pos7Pos5Err - ptrMotionBlk->Pos6Pos5Err) >
           POS_ERR_MAX)) {
        //����1��ֱ���ڴ˶�ʹ��
        //����2���޸����ǵ�ָ�������ֹͣ��

        delay_us(TIME_INTERVAL_US);
        SlaveID = 5;
        SetMotorAbsPos(SlaveID, (ptrServ[5]->PosPV));

        delay_us(TIME_INTERVAL_US);
        SlaveID = 6;
        SetMotorAbsPos(SlaveID, (ptrServ[5]->PosPV - ptrServ[5]->PosLocked +
                                 ptrServ[6]->PosLocked));

        delay_us(TIME_INTERVAL_US);
        SlaveID = 7;
        SetMotorAbsPos(SlaveID, (ptrServ[5]->PosPV - ptrServ[5]->PosLocked +
                                 ptrServ[7]->PosLocked));

        ptrMotionBlk->ErrCode |= AXIS_SYN_ERR;  //��ͬ������

        delay_ms(5);
      }

      SlaveID = 5;
      if ((finish_flag & (0x0001 << SlaveID)) == 0) {
        if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= (0x0001 << SlaveID);
        }
      } else
        ;

      SlaveID = 6;
      if ((finish_flag & (0x0001 << SlaveID)) == 0) {
        if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= (0x0001 << SlaveID);
        }
      } else
        ;

      SlaveID = 7;
      if ((finish_flag & (0x0001 << SlaveID)) == 0) {
        if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= (0x0001 << SlaveID);
        }
      } else
        ;

      if (finish_flag == ((0x0001 << 5) | (0x0001 << 6) | (0x0001 << 7))) {
        *state |= BIT_STEP_FINISH;  // finish
      }

      break;
    case 2:
      break;
    case 3:
      break;
    default:
      break;
  }
  ptrMotionBlk->FuncInnerState = finish_flag;
}

void Motion_Home_M1M2(u16 *state) {
  u8 SlaveID = 1;
  static u16 finish_flag = 0;

  switch (*state) {
    case 0:
      // slaveid = 1
      SlaveID = 1;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->HomingSpd);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq1Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosPV +
           (ptrCfgMotionPar->DeltaPos *
            ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��

      // slaveid = 2
      SlaveID = 2;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->HomingSpd);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq1Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosPV +
           (ptrCfgMotionPar->DeltaPos *
            ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��

      delay_us(TIME_INTERVAL_US);
      SlaveID = 1;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 2;
      SetMotorCtrlword(SlaveID, 0x000F);

      finish_flag = 0;
      *state = 1;  //״̬
      break;
    case 1:

      SlaveID = 1;
      if ((finish_flag & 0x0001) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             100))  // ��������ǰ������λ��û�д��?
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          ptrServ[SlaveID]->PosLocked = ptrServ[SlaveID]->PosPV;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosLocked);
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0001;
        } else {
          SetMotorAbsPos(
              SlaveID,
              (ptrServ[SlaveID]->PosPV +
               (ptrCfgMotionPar->DeltaPos *
                ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
        }
      } else
        ;

      SlaveID = 2;
      if ((finish_flag & 0x0002) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             100))  //ǰ��λ��С��100
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          ptrServ[SlaveID]->PosLocked = ptrServ[SlaveID]->PosPV;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosLocked);
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0002;
        } else {
          SetMotorAbsPos(
              SlaveID,
              (ptrServ[SlaveID]->PosPV +
               (ptrCfgMotionPar->DeltaPos *
                ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
        }
      } else
        ;

      if (finish_flag == 0x0003) {
        *state |= BIT_STEP_FINISH;  // finish
      }

      break;
    case 2:
      break;
    case 3:
      break;
    default:
      break;
  }
  ptrMotionBlk->FuncInnerState = finish_flag;
}

void Motion_Home_M3M4(u16 *state) {
  u8 SlaveID = 3;
  static u16 finish_flag = 0;

  switch (*state) {
    case 0:
      // slaveid = 1
      SlaveID = 3;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->HomingSpd);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq1Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosPV +
           (ptrCfgMotionPar->DeltaPos *
            ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��

      // slaveid = 2
      SlaveID = 4;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->HomingSpd);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq1Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosPV +
           (ptrCfgMotionPar->DeltaPos *
            ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��

      delay_us(TIME_INTERVAL_US);
      SlaveID = 3;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 4;
      SetMotorCtrlword(SlaveID, 0x000F);

      finish_flag = 0;
      *state = 1;  //״̬
      break;
    case 1:

      SlaveID = 3;
      if ((finish_flag & 0x0001) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             100))  // ��������ǰ������λ��û�д��?
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          ptrServ[SlaveID]->PosLocked = ptrServ[SlaveID]->PosPV;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosLocked);
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0001;
        } else {
          SetMotorAbsPos(
              SlaveID,
              (ptrServ[SlaveID]->PosPV +
               (ptrCfgMotionPar->DeltaPos *
                ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
        }
      } else
        ;

      SlaveID = 4;
      if ((finish_flag & 0x0002) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             100))  //ǰ��λ��С��100
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          ptrServ[SlaveID]->PosLocked = ptrServ[SlaveID]->PosPV;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosLocked);
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0002;
        } else {
          SetMotorAbsPos(
              SlaveID,
              (ptrServ[SlaveID]->PosPV +
               (ptrCfgMotionPar->DeltaPos *
                ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
        }
      } else
        ;

      if (finish_flag == 0x0003) {
        *state |= BIT_STEP_FINISH;  // finish
      }

      break;
    case 2:
      break;
    case 3:
      break;
    default:
      break;
  }
  ptrMotionBlk->FuncInnerState = finish_flag;
}

void Motion_Home_M5M6M7(u16 *state) {
  u8 SlaveID = 5;
  static u16 finish_flag = 0;

  switch (*state) {
    case 0:
      // slaveid = 1
#if 0			
				SlaveID = 5;
				SetMotorSpd(SlaveID,ptrCfgMotionPar->HomingSpdM5M6M7); //����ת��	
				delay_us(TIME_INTERVAL_US);
			
				SlaveID = 6;
				SetMotorSpd(SlaveID,ptrCfgMotionPar->HomingSpdM5M6M7); //����ת��
				delay_us(TIME_INTERVAL_US);			

				SlaveID = 7;
				SetMotorSpd(SlaveID,ptrCfgMotionPar->HomingSpdM5M6M7); //����ת��			

				delay_ms(10);
			
				SlaveID = 5;
				SetMotorCurrentLimit(SlaveID,ptrCfgMotionPar->Iq2Limit);//��������ȵ���?
				delay_us(TIME_INTERVAL_US);			
			
				SlaveID = 6;
				SetMotorCurrentLimit(SlaveID,ptrCfgMotionPar->Iq2Limit);//��������ȵ���?
				delay_us(TIME_INTERVAL_US);				
			
				SlaveID = 7;
				SetMotorCurrentLimit(SlaveID,ptrCfgMotionPar->Iq2Limit);//��������ȵ���?
				delay_us(TIME_INTERVAL_US);				
				
				delay_ms(10);
				
				SlaveID = 5;
				SetMotorAbsPos(SlaveID,(ptrServ[SlaveID ]->PosPV - (ptrCfgMotionPar->DeltaPosM5M6M7)));			//�����п����Ǵ��ġ��Ǿ��� ��-��
				delay_us(TIME_INTERVAL_US);				
			
				SlaveID = 6;
				SetMotorAbsPos(SlaveID,(ptrServ[SlaveID ]->PosPV - (ptrCfgMotionPar->DeltaPosM5M6M7)));			//�����п����Ǵ��ġ��Ǿ��� ��-��	
				delay_us(TIME_INTERVAL_US);				
			
				SlaveID = 7;
				SetMotorAbsPos(SlaveID,(ptrServ[SlaveID ]->PosPV - (ptrCfgMotionPar->DeltaPosM5M6M7)));			//�����п����Ǵ��ġ��Ǿ��� ��-��
				
        delay_us(TIME_INTERVAL_US);
				delay_us(TIME_INTERVAL_US);				
				
				SlaveID = 5;
				SetMotorCtrlword(SlaveID,0x000F);	
				SlaveID = 6;
				SetMotorCtrlword(SlaveID,0x000F);	
				SlaveID = 7;
				SetMotorCtrlword(SlaveID,0x000F);					
			
        finish_flag = 0;
				*state = 1; //״̬
#else

      SlaveID = 5;
      delay_us(TIME_INTERVAL_US);
      SetMotorSpd(SlaveID, ptrCfgMotionPar->HomingSpdM5M6M7);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq2Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosPV -
           (ptrCfgMotionPar->DeltaPosM5M6M7)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      delay_us(TIME_INTERVAL_US);

      SlaveID = 6;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->HomingSpdM5M6M7);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq2Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosPV -
           (ptrCfgMotionPar->DeltaPosM5M6M7)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      delay_us(TIME_INTERVAL_US);

      SlaveID = 7;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->HomingSpdM5M6M7);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq2Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosPV -
           (ptrCfgMotionPar->DeltaPosM5M6M7)));  //�����п����Ǵ��ġ��Ǿ��� ��-��

      delay_us(TIME_INTERVAL_US);

      SlaveID = 5;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 6;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 7;
      SetMotorCtrlword(SlaveID, 0x000F);

      finish_flag = 0;
      *state = 1;  //״̬
#endif
      break;
    case 1:

      SlaveID = 5;
      if ((finish_flag & 0x0001) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             20))  // ��������ǰ������λ��û�д��?
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          ptrServ[SlaveID]->PosLocked =
              ptrServ[SlaveID]->PosPV + ORG_POS_OFFSET;
          SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosPV + 1000));
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);

          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0001;
        } else {
          SetMotorAbsPos(
              SlaveID,
              (ptrServ[SlaveID]->PosPV -
               (ptrCfgMotionPar->DeltaPosM5M6M7)));  //�����п����Ǵ��ġ��Ǿ���
                                                     //��-��

          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
        }
      } else
        ;

      SlaveID = 6;
      if ((finish_flag & 0x0002) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             20))  //ǰ��λ��С��100
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          ptrServ[SlaveID]->PosLocked =
              ptrServ[SlaveID]->PosPV + ORG_POS_OFFSET;
          SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosPV + 1000));

          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);

          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0002;
        } else {
          SetMotorAbsPos(
              SlaveID,
              (ptrServ[SlaveID]->PosPV -
               (ptrCfgMotionPar->DeltaPosM5M6M7)));  //�����п����Ǵ��ġ��Ǿ���
                                                     //��-��

          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
        }
      } else
        ;

      SlaveID = 7;
      if ((finish_flag & 0x0004) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             20))  //ǰ��λ��С��100
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          ptrServ[SlaveID]->PosLocked =
              ptrServ[SlaveID]->PosPV + ORG_POS_OFFSET;
          SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosPV + 1000));

          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          delay_ms(INTERVAL_DELAY_TIME_MS);
          finish_flag |= 0x0004;
        } else {
          SetMotorAbsPos(
              SlaveID,
              (ptrServ[SlaveID]->PosPV -
               (ptrCfgMotionPar->DeltaPosM5M6M7)));  //�����п����Ǵ��ġ��Ǿ���
                                                     //��-��

          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
        }
      } else
        ;

      if (finish_flag == 0x0007) {
        *state |= BIT_STEP_FINISH;  // finish
      }

      break;
    case 2:
      // if(((ptrServ[5]->StatusWord & 0x0400)!=0)&&(ptrServ[5]->StatusWord &
      // 0x0400)!=0)&&(ptrServ[5]->StatusWord & 0x0400)!=0))
      break;
    case 3:
      break;
    default:
      break;
  }
  ptrMotionBlk->FuncInnerState = finish_flag;
}

void Motion_Home_All(u16 *state) {
  u8 SlaveID = 1;
  static u16 finish_flag = 0;

  switch (*state) {
    case 0:
      // slaveid = 1
      SlaveID = 1;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->HomingSpd);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq1Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosPV +
           (ptrCfgMotionPar->DeltaPos *
            ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      delay_us(TIME_INTERVAL_US);
      // slaveid = 2
      SlaveID = 2;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->HomingSpd);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq1Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosPV +
           (ptrCfgMotionPar->DeltaPos *
            ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��

      SlaveID = 3;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->HomingSpd);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq1Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosPV +
           (ptrCfgMotionPar->DeltaPos *
            ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      delay_us(TIME_INTERVAL_US);

      // slaveid = 2
      SlaveID = 4;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->HomingSpd);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq1Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosPV +
           (ptrCfgMotionPar->DeltaPos *
            ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��

      delay_us(TIME_INTERVAL_US);
      SlaveID = 1;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 2;
      SetMotorCtrlword(SlaveID, 0x000F);
      // delay_us(TIME_INTERVAL_US);
      SlaveID = 3;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 4;
      SetMotorCtrlword(SlaveID, 0x000F);

      finish_flag = 0;
      *state = 1;  //״̬
      break;
    case 1:

      SlaveID = 1;
      if ((finish_flag & 0x0001) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             100))  // ��������ǰ������λ��û�д��?
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          ptrServ[SlaveID]->PosLocked = ptrServ[SlaveID]->PosPV;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosLocked);
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          finish_flag |= 0x0001;
        } else {
          SetMotorAbsPos(
              SlaveID,
              (ptrServ[SlaveID]->PosPV +
               (ptrCfgMotionPar->DeltaPos *
                ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
        }
      } else
        ;

      SlaveID = 2;
      if ((finish_flag & 0x0002) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             100))  //ǰ��λ��С��100
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          ptrServ[SlaveID]->PosLocked = ptrServ[SlaveID]->PosPV;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosLocked);
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          finish_flag |= 0x0002;
        } else {
          SetMotorAbsPos(
              SlaveID,
              (ptrServ[SlaveID]->PosPV +
               (ptrCfgMotionPar->DeltaPos *
                ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
        }
      } else
        ;

      SlaveID = 3;
      if ((finish_flag & 0x0004) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             100))  // ��������ǰ������λ��û�д��?
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          ptrServ[SlaveID]->PosLocked = ptrServ[SlaveID]->PosPV;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosLocked);
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          finish_flag |= 0x0004;
        } else {
          SetMotorAbsPos(
              SlaveID,
              (ptrServ[SlaveID]->PosPV +
               (ptrCfgMotionPar->DeltaPos *
                ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
        }
      } else
        ;

      SlaveID = 4;
      if ((finish_flag & 0x0008) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             100))  //ǰ��λ��С��100
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          ptrServ[SlaveID]->PosLocked = ptrServ[SlaveID]->PosPV;
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosLocked);
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          finish_flag |= 0x0008;
        } else {
          SetMotorAbsPos(
              SlaveID,
              (ptrServ[SlaveID]->PosPV +
               (ptrCfgMotionPar->DeltaPos *
                ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
        }
      } else
        ;

      if (finish_flag == 0x000f) {
        //*state |= BIT_STEP_FINISH; //finish
        *state = 2;
      }

      break;
    case 2:  // M1M2 Open
      SlaveID = 1;
      SetMotorSpd(SlaveID, M_OPEN_SPD1);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, MAX_IQ1);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosLocked -
           ptrCfgMotionPar->PosOpenLen *
               ptrCfgMotionPar->PosFactor));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?

      SlaveID = 2;
      SetMotorSpd(SlaveID, M_OPEN_SPD1);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, MAX_IQ1);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosLocked -
           ptrCfgMotionPar->PosOpenLen *
               ptrCfgMotionPar->PosFactor));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?

      delay_us(TIME_INTERVAL_US);
      SlaveID = 1;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 2;
      SetMotorCtrlword(SlaveID, 0x000F);

      finish_flag = 0;
      *state = 3;  //״̬
      break;
    case 3:
      SlaveID = 1;
      if ((finish_flag & 0x0001) == 0) {
        if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          finish_flag |= 0x0001;
        }
      } else
        ;

      SlaveID = 2;
      if ((finish_flag & 0x0002) == 0) {
        if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��
          finish_flag |= 0x0002;
        }
      } else
        ;

      if (finish_flag == 0x0003) {
        //*state |= BIT_STEP_FINISH; //finish
        *state = 4;
      }
      break;
    case 4:  // M5M6M7 ����

      SlaveID = 5;
      delay_us(TIME_INTERVAL_US);
      SetMotorSpd(SlaveID, ptrCfgMotionPar->HomingSpdM5M6M7);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq2Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosPV -
           (ptrCfgMotionPar->DeltaPosM5M6M7)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      delay_us(TIME_INTERVAL_US);

      SlaveID = 6;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->HomingSpdM5M6M7);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq2Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosPV -
           (ptrCfgMotionPar->DeltaPosM5M6M7)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      delay_us(TIME_INTERVAL_US);

      SlaveID = 7;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->HomingSpdM5M6M7);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq2Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(
          SlaveID,
          (ptrServ[SlaveID]->PosPV -
           (ptrCfgMotionPar->DeltaPosM5M6M7)));  //�����п����Ǵ��ġ��Ǿ��� ��-��

      delay_us(TIME_INTERVAL_US);

      SlaveID = 5;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 6;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 7;
      SetMotorCtrlword(SlaveID, 0x000F);

      finish_flag = 0;
      *state = 5;  //״̬

      break;
    case 5:

      SlaveID = 5;
      if ((finish_flag & 0x0001) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             20))  // ��������ǰ������λ��û�д��?
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          ptrServ[SlaveID]->PosLocked =
              ptrServ[SlaveID]->PosPV + ORG_POS_OFFSET;
          SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosPV + 1000));
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);

          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��

          finish_flag |= 0x0001;
        } else {
          SetMotorAbsPos(
              SlaveID,
              (ptrServ[SlaveID]->PosPV -
               (ptrCfgMotionPar->DeltaPosM5M6M7)));  //�����п����Ǵ��ġ��Ǿ���
                                                     //��-��

          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
        }
      } else
        ;

      SlaveID = 6;
      if ((finish_flag & 0x0002) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             20))  //ǰ��λ��С��100
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          ptrServ[SlaveID]->PosLocked =
              ptrServ[SlaveID]->PosPV + ORG_POS_OFFSET;
          SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosPV + 1000));

          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);

          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��

          finish_flag |= 0x0002;
        } else {
          SetMotorAbsPos(
              SlaveID,
              (ptrServ[SlaveID]->PosPV -
               (ptrCfgMotionPar->DeltaPosM5M6M7)));  //�����п����Ǵ��ġ��Ǿ���
                                                     //��-��

          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
        }
      } else
        ;

      SlaveID = 7;
      if ((finish_flag & 0x0004) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             20))  //ǰ��λ��С��100
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          ptrServ[SlaveID]->PosLocked =
              ptrServ[SlaveID]->PosPV + ORG_POS_OFFSET;
          SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosPV + 1000));

          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��

          finish_flag |= 0x0004;
        } else {
          SetMotorAbsPos(
              SlaveID,
              (ptrServ[SlaveID]->PosPV -
               (ptrCfgMotionPar->DeltaPosM5M6M7)));  //�����п����Ǵ��ġ��Ǿ���
                                                     //��-��

          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
        }
      } else
        ;

      if (finish_flag == 0x0007) {
        //*state |= BIT_STEP_FINISH; //finish
        *state = 6;  // finish
      }
      break;
    case 6:  // M1M2 CLOSE
      SlaveID = 1;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->CloseSpd);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq1Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked +
                               500));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?

      SlaveID = 2;
      SetMotorSpd(SlaveID, ptrCfgMotionPar->CloseSpd);  //����ת��
      delay_us(TIME_INTERVAL_US);
      SetMotorCurrentLimit(SlaveID, ptrCfgMotionPar->Iq1Limit);  //��������ȵ���?
      delay_us(TIME_INTERVAL_US);
      SetMotorAbsPos(SlaveID, (ptrServ[SlaveID]->PosLocked +
                               500));  //�����п����Ǵ��ġ��Ǿ��� ��-��
      ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?

      delay_us(TIME_INTERVAL_US);
      SlaveID = 1;
      SetMotorCtrlword(SlaveID, 0x000F);
      SlaveID = 2;
      SetMotorCtrlword(SlaveID, 0x000F);
      finish_flag = 0;

      *state = 7;  //״̬
      break;
    case 7:
      SlaveID = 1;
      if ((finish_flag & 0x0001) == 0) {
        if ((abs(ptrServ[SlaveID]->TrqPV) >=
             ptrServ[SlaveID]->MaxcurrentLocked) &&
            (abs(ptrServ[SlaveID]->PosPV - ptrServ[SlaveID]->PosPV_Last) <
             100))  //ǰ��λ��С��100
        // if(abs(ptrServ[SlaveID]->TrqPV) >=
        // ptrServ[SlaveID]->MaxcurrentLocked)
        {
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��

          finish_flag |= 0x0001;
        } else {
          if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
            SetMotorAbsPos(
                SlaveID,
                (ptrServ[SlaveID]->PosPV +
                 (ptrCfgMotionPar->DeltaPos *
                  ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
            delay_us(TIME_INTERVAL_US);
            SetMotorCtrlword(SlaveID, 0x000F);
            ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?
          }
        }
      } else
        ;

      SlaveID = 2;
      if ((finish_flag & 0x0002) == 0) {
        if (abs(ptrServ[SlaveID]->TrqPV) >=
            ptrServ[SlaveID]->MaxcurrentLocked) {
          SetMotorAbsPos(SlaveID, ptrServ[SlaveID]->PosPV);
          delay_us(TIME_INTERVAL_US);
          SetMotorCtrlword(SlaveID, 0x000F);
          // SetMotorCtrlword(SlaveID,(SERV_HALT_BIT|ptrServ[SlaveID]->CtrlWord));
          // //������ָͣ��

          finish_flag |= 0x0002;
        } else {
          if ((ptrServ[SlaveID]->StatusWord & 0x0400) != 0) {
            SetMotorAbsPos(
                SlaveID,
                (ptrServ[SlaveID]->PosPV +
                 (ptrCfgMotionPar->DeltaPos *
                  ptrCfgMotionPar->PosFactor)));  //�����п����Ǵ��ġ��Ǿ��� ��-��
            delay_us(TIME_INTERVAL_US);
            SetMotorCtrlword(SlaveID, 0x000F);
            ptrServ[SlaveID]->StatusWord &= ~0x0400;  //�Ѷ�λ��ɸ��?
          }
        }
      } else
        ;

      if (finish_flag == 0x0003) {
        //�ָ�����������
        SlaveID = 5;
        SetMotorCurrentLimit(SlaveID, MAX_IQ2);  //��������ȵ�������ȵ�����3��
        delay_us(TIME_INTERVAL_US);

        SlaveID = 6;
        SetMotorCurrentLimit(SlaveID, MAX_IQ2);  //��������ȵ�������ȵ�����3��
        delay_us(TIME_INTERVAL_US);

        SlaveID = 7;
        SetMotorCurrentLimit(SlaveID, MAX_IQ2);  //��������ȵ�������ȵ�����3��
        delay_us(TIME_INTERVAL_US);

        ptrMotionBlk->StatusWord |= BIT_HOMING_FINISH;

        *state |= BIT_STEP_FINISH;  // finish
      }
      break;
    default:
      break;
  }
  ptrMotionBlk->FuncInnerState = finish_flag;
}
