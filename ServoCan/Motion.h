#ifndef __MOTION__
#define __MOTION__

#include "includes.h"
#include "stm32f4xx.h"

#ifndef Para
#define Para dGlbReg
#endif

//电机参数

//抱紧电机
#define RATED_IQ1 2760
#define MAX_IQ1 RATED_IQ1 * 3
#define NOMINAL_TORQUE1 57.8
#define REDUCTION_RATIO1 83

//升降电机
#define RATED_IQ2 4260
#define MAX_IQ2 RATED_IQ2 * 3
#define NOMINAL_TORQUE2 121
#define REDUCTION_RATIO2 3.9

#define RATED_SPD1 9690 * 0.6
#define MAX_SPD1 10700
#define M_OPEN_SPD1 RATED_SPD1

#define RATED_SPD2 7160 * 0.6
#define MAX_SPD2 7720

#define LONG_POS 10000  //升降的位置
#define SHORT_POS 0

#define ORG_POS_OFFSET 10000

#define MOTION_EN_BIT 0x0001
#define TARGET_REACH_BIT 0x0200  //状态字的位置到达信号

//#define (ptrMotionBlk->DeltaPos * ptrMotionBlk->PosFactor) 1000
#define MOTOR_UPDOWN_POS 1000

#define MOTOR_MAX_CURRENT \
  500  //这个置为多少，需要根据测试的结构来修改，和载重,摩擦系数，倾斜度有关
#define AUTO_MODE 0

typedef enum enum_SystemState {
  SYS_IDLE = 0,
  SYS_INIT = 1,
  SYS_RDY,
  SYS_WORKING,
  SYS_FAULT
} enum_SystemStateTYPE;

typedef enum enum_motionState {
  S_MOTION_IDLE = 0,  //执行完一状态的暂停状态
  S_MOTION_OPEN_M1M2 = 1,
  S_MOTION_EXTEND_M5M6M7,  //像上方向升长,和
                           // S_MOTION_DN_SHORTEN_M5M6M7，其实是一样的
  S_MOTION_CLOSE_M1M2,
  S_MOTION_OPEN_M3M4,
  S_MOTION_SHORTEN_M5M6M7,  //向上方向缩短，和S_MOTION_DN_EXTEND_M5M6M7
                            //一样，实现
  S_MOTION_CLOSE_M3M4,
  S_MOTION_HOME_M1M2,
  S_MOTION_HOME_M3M4,
  S_MOTION_HOME_M5M6M7,
  S_MOTION_HOME_ALL,
  S_MOTION_PAUSE,
  S_MOTION_IM_PAUSE,
  S_MOTION_FAULT,
  S_SINGLE_OPEN_M1M2,
  S_SINGLE_CLOSE_M1M2,
  S_SINGLE_OPEN_M3M4,
  S_SINGLE_CLOSE_M3M4,
  S_SINGLE_EXTEND_M5M6M7,  //
  S_SINGLE_SHORTEN_M5M6M7

} enum_motionStateTYPE;

extern void MotionInit(void);
extern void VarClear(void);
extern void HandleMotion(void);

#if 0  // for Normal
extern void MOTION_S01_Fun(u16 *state);
extern void MOTION_S02_Fun(u16 *state);
extern void MOTION_S03_Fun(u16 *state);
extern void MOTION_S05_Fun(u16 *state);
extern void MOTION_S06_Fun(u16 *state);
extern void MOTION_S07_Fun(u16 *state);
#endif

#endif
