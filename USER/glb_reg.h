
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLB_REG_H
#define __GLB_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "os_cpu.h"
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/

#ifndef Para
#define Para dGlbReg
#endif

typedef __packed struct {
  u16 SlaveID;
  u16 CtrlMode;
  u16 ServSTA;
  u16 ServErr;
  u16 CtrlWord;
  s16 StatusWord;
  s32 PosSV;      // set
  s32 PosPV;      // feedback
  s32 PosLocked;  //��Ӧ�������m1/m2/m3/4,��λ�ñ�ʾΪ�������������Ӧ��λ�á�����??
                  //����m4/m5/6 ,
                  //Ϊ����ƽ�������λ�á��˲�����Ϊ�ǲο���
  s32 PosLimit;  // ����һ���������λλ�á�??
  s32 SpdSV;
  s32 SpdPV;
  s16 MaxCurrenLimit;  // TrqSV
  s16 TrqPV;
  s16 MaxcurrentLocked;
  u16 RdUpdate;  // 1:��ʾ����ɣ������?���¡���������?��,��ô��Ѹ��?��0
                 // .bit0:posSV, bit1:TrqPV, bit2:spdPV
  u16 init_ok;
  s32 PosPV_Last;
} SERV_PAR_TYPE;

typedef __packed struct {
  u16 PosFactor;  //����λ�������趨��ʵ��λ��
                  //= posset*DebugPosFactor;
  s16 PosOpenLen;  //��բ�ɿ���λ������ ,actual =
                   // PosOpenLen * PosFactor;
  u16 ClimbLenSameFlag;  // 0:��һ����1��һ��
  u16 PosClimbLen;  //���Ƹ�1�����ĳ���λ��
  u16 PosClimbLen2;  //���Ƹ�2�ĳ���
  u16 PosClimbLen3;  //���Ƹ�3�ĳ���
  u16 HomingSpd;     //�����ٶ�
  u16 CloseSpd;      //����������ٶ�??
  u16 ClimbSpd;      // �����ٶȡ�
  s16 Iq1Limit;  //���������������??
  s16 DeltaPos;  //�����λ������??

  u16 HomingSpdM5M6M7;

  s16 DeltaPosM5M6M7;
  u16 Iq2Limit;

  u16 PosOpenLen2;
  s16 CompCoff;  //�ٶȲ���ϵ��

  u32 acc1;
  u32 acc2;
  u32 RunSteps;    //�����еĲ���
  u16 DirAutoChg;  //�����Զ��޸�����
  u16 Rev1[7];
  u16 pwr_en;

} CFG_MOTION_PAR_TYPE;

typedef __packed struct {
  u16 MotionMode;  // 0���Զ�ģʽ�� 1������ģʽ
  u16 AutoCycleNum;    //
  u16 AutocycleLeft;   //
  u16 StepsPreSet;     //Ԥ�ö��ٲ�
  u16 StepsLeft;       //ʣ����ٲ�??
  u16 MotionDir;       // �ƶ�������
                       //
  u16 MotionCtrlWord;  // ����?������Ч�������1���Ե�
  u16 AutoCycleEn;     //
  u16 MotionPause;  //���ᶯ��������ͣ��ͨ����λʵ��
  u16 MotionCmdCode;  //�ƻ�Ϊ1���Ե�����
                      //
  u16 SysState;       //ϵͳ״̬
  u16 MotionState;    //�˶���״̬��״̬
                      //
  u16 Rev1[8];
  /*
          u16 PosFactor;
     //����λ�������趨��ʵ��λ�� =
     posset*DebugPosFactor; s16 PosOpenLen;
     //��բ�ɿ���λ������ ,actual = PosOpenLen *
     PosFactor; u16 PosClimbLen; //�����ĳ���λ�� u16
     HomingSpd; //�����ٶ� u16 CloseSpd;
     //����������ٶ�?? u16 ClimbSpd; //
     �����ٶȡ� s16 Iq1Limit;
     //���������������?? s16
     DeltaPos;//�����λ������??
  */

  //
  u16 MotionStateLast;  //��1״̬
  u16 MotionStateNext;  //��1״̬
  u16 SmallState;       //��ʼ״̬Ϊ0,���״�?Ϊ128
  u16 StatusWord;       //״̬��
  u16 FuncInnerState;  // �����ڲ���״̬�����ڵ���
  u16 ErrCode;         //������
  u16 CmdRepState;     // �������Ӧ״�?
                    // ����һ��������������Ӧ����
                    // ���??0��NoErr��
                    // ���������Ӧ�������Ӧ�Ĵ�����˵��������Ӧ��ԭ��
  u16 StartNodeFlag;  // 7���ŷ���ÿ��ռ1bit���ֱ�Ϊbit1...bit7

  u16 Rev2[3];
  /*
          u16 HomingSpdM5M6M7;
          s16 DeltaPosM5M6M7;
          u16 Iq2Limit;
  */
  u16 AutoCycleState;
  //
  u16 DebugSlaveId;  //���Ե���������վ��ַ
  u16 DebugMaxIqSet;    //�����������趨
  s16 DebugSpdSet;      //�����ٶ��趨
  s16 DebugPosSet;      //���Ե�λ���趨
  u16 M1orM3OpenState;  // bit0 for m1m2, bit1 for m3m4

  u16 Rev3[2];
  /*
          u16 PosOpenLen2;
    s16 CompCoff; //�ٶȲ���ϵ��
  */
  //
  s32 Pos6Pos5Err;
  s32 Pos7Pos5Err;

  u16 Di;
  u16 Do;
  s16 Ai1;
  s16 Ai2;
  u16 StateOfCharge;
  u16 BatVolt;
  u16 BatCurrent;
  u16 BatTemp;  //

  //	u16 Rev4[4];
  /*
          u32 acc1;
          u32 acc2;
  */

} MOTION_VAR_TYPE;

#define SMALLSTATEFINISH 0x80  // 128
/* Exported constants --------------------------------------------------------*/
#define SLAVE_ADDR 0

#define Judge_SAVE_PAR_ADDR \
  198  //�������ַ�ж��Ƿ��״μ��س���??
#define SAVE_PAR_ADDR 299
#define DEBUG_EN_ADDR SAVE_PAR_ADDR
#define CMD_CODE_ADDR 209

#define SAVE_PAR_CMD 2049
#define SAVE_PAR_NUM 200  // 299

#define DEBUG_EN_CMD 2050
#define DEBUG_DIS_CMD 2051

#define RdPOS_UD_BIT 0x0001
#define RdTRQ_UD_BIT 0x0002
#define RdSPD_UD_BIT 0x0004

#define SERV0_BASE 250
#define SERV2_BASE 350
#define SERV3_BASE 400
#define SERV4_BASE 450
#define SERV5_BASE 500
#define SERV6_BASE 550
#define SERV7_BASE 600

#define CFG_MOTION_PAR_BASE 100
#define MOTION_STATE_BASE 200
#define STEPS_CNT_BASE 120

#define MONITOR_BASE 260

/*
#define DI_BASE 90
#define DO_BASE 91
#define AI0_BASE 93
#define AI1_BASE 94

#define REG_DI Para[DI_BASE]
#define REG_DO Para[DO_BASE]
#define REG_AI0 Para[AI0_BASE]
#define REG_AI1 Para[AI1_BASE]
*/

//#define PWR_EN_BASE 92

//#define ptrCfgMotionPar->pwr_en Para[PWR_EN_BASE]
//#define ptrCfgMotionPar->RunSteps (*((u32 *)(&Para[STEPS_CNT_BASE])))

/*
#define ROBOT_EN_BASE 100 //
#define REG_ROBOT_EN Para[ROBOT_EN_BASE]

#define ROBOT_MODE_BASE 101 //
#define REG_ROBOT_MODE Para[ROBOT_MODE_BASE]

#define ROBOT_CMD_BASE 102 //
#define REG_ROBOT_CMD Para[ROBOT_CMD_BASE]

#define ROBOT_STA_BASE 103 //
#define REG_ROBOT_STA Para[ROBOT_STA_BASE]

#define ROBOT_SMALLSTA_BASE 104 //
#define REG_ROBOT_SMALLSTA Para[ROBOT_SMALLSTA_BASE]

#define DEBUG_ID_BASE 105
#define REG_DEBUG_ID Para[DEBUG_ID_BASE]

#define DEBUG_MaxIq_BASE 106
#define REG_MaxIq Para[DEBUG_MaxIq_BASE]

#define DEBUG_Spd_BASE 107
#define REG_DEBUG_Spd Para[DEBUG_Spd_BASE]

#define DEBUG_PosSet_BASE 108
#define REG_DEBUG_PosSet Para[DEBUG_PosSet_BASE]

#define DEBUG_Pos_Xxx_BASE 109   //���ʡ� ʵ��λ�� =
REG_DEBUG_PosSet*REG_DEBUG_Pos_Xxx, ͨ��REG_DEBUG_Pos_XxxΪ100 ��1000
#define REG_DEBUG_Pos_Xxx Para[DEBUG_Pos_Xxx_BASE]

#define DEBUG_POS_LEN_BASE 110
#define REG_DEBUG_POS_LEN Para[DEBUG_POS_LEN_BASE]

#define DEBUG_POS_LEN2_BASE 111
#define REG_DEBUG_POS_LEN2 Para[DEBUG_POS_LEN2_BASE]
*/

/* Exported macro ------------------------------------------------------------*/
// for ctrlword or statusword
#define BIT_ROBOT_EN Bit[0]  //ʹ��λ
#define BIT_INIT_OK Bit[1]   //��ʼ�����??
#define BIT_DIR Bit[3]  //  ����λ //����statusword ��Ч

#define BIT_SOFT_STOP Bit[8]  //ֹͣλ �Ͳ�����ͣ��һ��
#define BIT_IM_PAUSE \
  Bit[9]  // ������ͣ�ڵ�ǰ״̬�ı�־�����ڵ���

// for statusword
#define BIT_REMOTE_START Bit[9]  //

#define BIT_HOMING_FINISH Bit[13]
#define BIT_HOMING_BUSY Bit[14]
// for smallState
#define BIT_STEP_FINISH Bit[15]

#define POS_ERR_MAX 10000 * 3
// errcode
#define START_FAIL Bit[9]
#define SERVO_ERR Bit[8]
#define AXIS_SYN_ERR Bit[10]

/* Exported functions ------------------------------------------------------- */

// void SysTick_Handler(void);

#define HOLDING_REG_END 4096  // 1024

void StateMonitor(void);

extern u16 DebugFlag;
extern u16 SlaveIdAddr;
extern const u16 Bit[];
extern volatile u16 dGlbReg[];
extern SERV_PAR_TYPE* const ptrServ[8];
extern MOTION_VAR_TYPE* const ptrMotionBlk;
extern CFG_MOTION_PAR_TYPE* const ptrCfgMotionPar;

#ifdef __cplusplus
}
#endif

#endif /* ____GLB_REG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
