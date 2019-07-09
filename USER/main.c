#include "24cxx.h"
#include "Motion.h"
#include "RS485.h"
#include "ServoCan.h"
#include "ServoCom.h"
#include "adc.h"
#include "beep.h"
#include "delay.h"
#include "glb_reg.h"
#include "includes.h"
#include "key.h"
#include "led.h"
#include "sys.h"
#include "usart.h"

/////////////////////////UCOSII�����ջ����?///////////////////////////////////
// START ����
//�����������ȼ�
#define START_TASK_PRIO 10  //��ʼ��������ȼ������?���?
//���������ջ���?
#define START_STK_SIZE 64
//���������ջ�ռ�?
OS_STK START_TASK_STK[START_STK_SIZE];
//�������ӿ�
void start_task(void *pdata);

// LED���񼰷�����������ָʾ�����?
//�����������ȼ�
#define LED_TASK_PRIO 50
//���������ջ���?
#define LED_STK_SIZE 64
//���������ջ�ռ�?
OS_STK LED_TASK_STK[LED_STK_SIZE];
//�������ӿ�
void led_task(void *pdata);

//����λ��PC��ͨ������
//�����������ȼ�
#define PcCom_TASK_PRIO 15
//���������ջ���?
#define PcCom_STK_SIZE 256
//���������ջ�ռ�?
OS_STK PcCom_TASK_STK[PcCom_STK_SIZE];
//�������ӿ�
void PcCom_task(void *pdata);

//"�����?"����
#define TaskPlan_TASK_PRIO 8

#define TaskPlan_STK_SIZE 256

OS_STK TaskPlan_TASK_STK[TaskPlan_STK_SIZE];

void TaskPlan_task(void *pdata);

//�˶��������� //Trajectory planning
#define MotionCtrl_TASK_PRIO 5

#define MotionCtrl_STK_SIZE 256

OS_STK MotionCtrl_TASK_STK[MotionCtrl_STK_SIZE];

void MotionCtrl_task(void *pdata);

//ִ�л��������� ͨ������ĿǰΪRS485
#define ServoCom_TASK_PRIO 4

#define ServoCom_STK_SIZE 256

OS_STK ServoCom_TASK_STK[ServoCom_STK_SIZE];

void ServoCom_task(void *pdata);

//��������ȡ����
#define RdSensor_TASK_PRIO 20

#define RdSensor_STK_SIZE 256

OS_STK RdSensor_TASK_STK[RdSensor_STK_SIZE];

void RdSensor_task(void *pdata);

//����ɨ������
//�����������ȼ�
#define KEY_TASK_PRIO 3
//���������ջ���?
#define KEY_STK_SIZE 256  // 64
//���������ջ�ռ�?
OS_STK KEY_TASK_STK[KEY_STK_SIZE];
//�������ӿ�
void key_task(void *pdata);

//�ź��������ڽ���ͨ��

// OS_EVENT * sem_printf;		//��ӡ�ź���ָ��

int main(void) {
  delay_init(168);  //��ʼ����ʱ����
  uart_init(115200);
  // LED_Init();
  // //��ʼ����LED���ӵ�Ӳ���ӿ�
  HardWare_Init();
  AT24CXX_Init();
  Adc_Init();
  BEEP_Init();  //��������ʼ��
  // KEY_Init();				//������ʼ��
  OSInit();  //��ʼ��UCOSII
  OSTaskCreate(start_task, (void *)0,
               (OS_STK *)&START_TASK_STK[START_STK_SIZE - 1],
               START_TASK_PRIO);  //������ʼ����
  OSStart();
}

//��ʼ����
void start_task(void *pdata) {
  OS_CPU_SR cpu_sr = 0;
  pdata = pdata;
  // sem_printf=OSSemCreate(1);		//������ӡ�ź���

  OSStatInit();  //��ʼ��ͳ������.��������?1��������
  OS_ENTER_CRITICAL();  //�����ٽ���(�޷����жϴ��?)
  OSTaskCreate(led_task, (void *)0, (OS_STK *)&LED_TASK_STK[LED_STK_SIZE - 1],
               LED_TASK_PRIO);
  // OSTaskCreate(key_task,(void
  // *)0,(OS_STK*)&KEY_TASK_STK[KEY_STK_SIZE-1],KEY_TASK_PRIO);
  OSTaskCreate(PcCom_task, (void *)0,
               (OS_STK *)&PcCom_TASK_STK[PcCom_STK_SIZE - 1], PcCom_TASK_PRIO);
  OSTaskCreate(TaskPlan_task, (void *)0,
               (OS_STK *)&TaskPlan_TASK_STK[TaskPlan_STK_SIZE - 1],
               TaskPlan_TASK_PRIO);
  OSTaskCreate(MotionCtrl_task, (void *)0,
               (OS_STK *)&MotionCtrl_TASK_STK[MotionCtrl_STK_SIZE - 1],
               MotionCtrl_TASK_PRIO);
  OSTaskCreate(ServoCom_task, (void *)0,
               (OS_STK *)&ServoCom_TASK_STK[ServoCom_STK_SIZE - 1],
               ServoCom_TASK_PRIO);
  OSTaskCreate(RdSensor_task, (void *)0,
               (OS_STK *)&RdSensor_TASK_STK[RdSensor_STK_SIZE - 1],
               RdSensor_TASK_PRIO);
  OSTaskSuspend(START_TASK_PRIO);  //������ʼ����.
  OS_EXIT_CRITICAL();  //�˳��ٽ���(���Ա��жϴ��?)
}
// LED����
void led_task(void *pdata) {
  static u32 Steps_Last = 0;
  //	INT8U err;
  LED1 = 1;  // red led off
  Steps_Last = ptrCfgMotionPar->RunSteps;
  ptrMotionBlk->StateOfCharge = 500;  //ʣ��50%����
  ptrMotionBlk->BatVolt = 240;        //��ص��?24V x0.1
  ptrMotionBlk->BatCurrent = 80;      //����8A x0.1
  ptrMotionBlk->BatTemp = 456;        // �¶� x0.1

  DebugFlag = 0;
  while (1) {
    //���������
    if (DebugFlag == 0) {
      if (Para[DEBUG_EN_ADDR] == DEBUG_EN_CMD) {
        DebugFlag = 1;
        Para[DEBUG_EN_ADDR] = 0;
      }
    } else {
      if (Para[DEBUG_EN_ADDR] == DEBUG_DIS_CMD) {
        ptrMotionBlk->MotionCmdCode = 14;  //ȡ������ģʽʱ�����»�1��0
        delay_ms(10);
        DebugFlag = 0;
        Para[DEBUG_EN_ADDR] = 0;
      }
    }

    //дeeprom����
    if (Para[SAVE_PAR_ADDR] == SAVE_PAR_CMD) {
      AT24CXX_Write(0, (u8 *)(Para),
                    (SAVE_PAR_NUM * 2));  //��ָ����ַ��ʼд��ָ�����ȵ�����
      // void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);
      // //��ָ����ַ��ʼ����ָ�����ȵ�����
      Para[SAVE_PAR_ADDR] = 0;
    }

    {
      if (Steps_Last != ptrCfgMotionPar->RunSteps) {
        Steps_Last = ptrCfgMotionPar->RunSteps;
        AT24CXX_Write(STEPS_CNT_BASE * 2, (u8 *)(&Para[STEPS_CNT_BASE]), 2);
      }
    }

    LED0 = !LED0;  // yellow led
    if (ptrMotionBlk->ErrCode != 0) {
      LED1 = ~LED1;  // red led blink
    } else {
      LED1 = 1;  // off state
    }
    // LED0=!LED0;
    //		 LED1=!LED1;

    delay_ms(1000);
    // OSSemPend(sem_printf,0,&err);
    // printf("led_task \n");
    // OSSemPost(sem_printf);
  }
}

//����ɨ������
void key_task(void *pdata) {
  // u8 err;
  u8 key;
  while (1) {
    key = KEY_Scan(0);

    if (key == KEY0_PRES) {
      printf("key0,dGlbReg[255]:%d \n", dGlbReg[255]);
    } else if (key == KEY2_PRES) {
      printf("key2 \n");
    } else if (key == WKUP_PRES) {
      printf("keyup \n");
    } else if (key == KEY1_PRES) {
      printf("key1 \n");
    }

    // OSSemPend(sem_printf,0,&err);
    // printf("key_task \n");
    // OSSemPost(sem_printf);
    delay_ms(1000);
  }
}

//ͨ������
void PcCom_task(void *pdata) {
  //	static u8 aa = 0;
  RS485Init();
  printf("uart1 init ok \n");
  while (1) {
    Protocol_RS485();
    // printf("In PcCom_task \n");
    // USART1->DR = aa++;

    // delay_ms(1000);
  }
}

//�����? ����
void TaskPlan_task(void *pdata) {
  while (1) {
    // printf("In TaskPlan_task \n");
    delay_ms(1000);
  }
}

void MotionCtrl_task(void *pdata) {
  while (1) {
    // printf("In MotionCtrl_task \n");
    delay_ms(1000);
  }
}

#if 0
void ServoCom_task(void *pdata)
{
  ServoCom_Init();
	printf("In ServoCom_task \n");
	while(1)
	{	
    //USART2->DR = 'a';//(Data & (uint16_t)0x01FF);		
		ServoComRW();
		delay_ms(100); 	
	}	
}
#endif

void ServoCom_task(void *pdata) {
  CanInit();

  //�ָ�eeprom ����
  AT24CXX_Read(0, (u8 *)(Para),
               (SAVE_PAR_NUM * 2));  //��ָ����ַ��ʼ����ָ�����ȵ�����
  if (Para[Judge_SAVE_PAR_ADDR] != 0x55aa) {  //�״γ�ʼ������
    Para[Judge_SAVE_PAR_ADDR] = 0x55aa;
    Para[SLAVE_ADDR] = 1;  //ͨ�ŵ�ַ
    MotionInit();
    AT24CXX_Write(0, (u8 *)(Para),
                  (SAVE_PAR_NUM * 2));  //��ָ����ַ��ʼд��ָ�����ȵ�����
  }

  VarClear();
  SlaveIdAddr = Para[SLAVE_ADDR];
  // MotorInit();
  ptrCfgMotionPar->pwr_en = 3;

  while (1) {
    //	Para[9] = SlaveIdAddr;
    // HandleCan();
    HandleMotion();
    StateMonitor();
    // printf("In ServoCan_task \n");
    delay_ms(2);
  }
}

void RdSensor_task(void *pdata) {
  while (1) {
    // printf("In RdSensor_task \n");
    HandleGPIO();                    //
    ptrMotionBlk->Ai1 = Get_Adc(9);  //��ADC
    delay_ms(100);
  }
}
