#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "includes.h"
#include "key.h"
#include "beep.h"
#include "glb_reg.h"
#include "RS485.h"
#include "ServoCom.h"
#include "ServoCan.h"
#include "adc.h"
#include "Motion.h"
#include "24cxx.h"



/////////////////////////UCOSII任务堆栈设置///////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				64
//创建任务堆栈空间	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数接口
void start_task(void *pdata);	
 			   
//LED任务及蜂蜜器，用于指示或调试
//设置任务优先级
#define LED_TASK_PRIO       			50 
//设置任务堆栈大小
#define LED_STK_SIZE  		    		64
//创建任务堆栈空间	
OS_STK LED_TASK_STK[LED_STK_SIZE];
//任务函数接口
void led_task(void *pdata);


//和上位机PC的通信任务
//设置任务优先级
#define PcCom_TASK_PRIO       			15 
//设置任务堆栈大小
#define PcCom_STK_SIZE  					256
//创建任务堆栈空间	
OS_STK PcCom_TASK_STK[PcCom_STK_SIZE];
//任务函数接口
void PcCom_task(void *pdata);

//"任务规划"任务
#define TaskPlan_TASK_PRIO 8

#define TaskPlan_STK_SIZE 256

OS_STK TaskPlan_TASK_STK[TaskPlan_STK_SIZE];

void TaskPlan_task(void *pdata);


//运动控制任务 //Trajectory planning
#define MotionCtrl_TASK_PRIO 5

#define MotionCtrl_STK_SIZE 256

OS_STK MotionCtrl_TASK_STK [MotionCtrl_STK_SIZE];

void MotionCtrl_task(void *pdata);

//执行机构驱动器 通信任务，目前为RS485
#define ServoCom_TASK_PRIO 4

#define ServoCom_STK_SIZE 256

OS_STK ServoCom_TASK_STK [ServoCom_STK_SIZE];

void ServoCom_task(void *pdata);


//传感器读取任务
#define RdSensor_TASK_PRIO 20

#define RdSensor_STK_SIZE 256

OS_STK RdSensor_TASK_STK [RdSensor_STK_SIZE];

void RdSensor_task(void *pdata);

//按键扫描任务
//设置任务优先级
#define KEY_TASK_PRIO       			3 
//设置任务堆栈大小
#define KEY_STK_SIZE  					256//64
//创建任务堆栈空间	
OS_STK KEY_TASK_STK[KEY_STK_SIZE];
//任务函数接口
void key_task(void *pdata);

//信号量，用于进程通信

//OS_EVENT * sem_printf;		//打印信号量指针	 	 

int main(void)
{ 
 
	delay_init(168);		  //初始化延时函数
	uart_init(115200);
	//LED_Init();		  		//初始化与LED连接的硬件接口
	HardWare_Init();
	AT24CXX_Init();
	Adc_Init();
 	BEEP_Init();			//蜂鸣器初始化	
	//KEY_Init();				//按键初始化
	OSInit();  	 			//初始化UCOSII		 			  
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();	
}

//开始任务
void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;
	pdata = pdata; 		  
	//sem_printf=OSSemCreate(1);		//创建打印信号量			
	
	
	OSStatInit();					//初始化统计任务.这里会延时1秒钟左右	
 	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)    
 	OSTaskCreate(led_task,(void *)0,(OS_STK*)&LED_TASK_STK[LED_STK_SIZE-1],LED_TASK_PRIO);   
 	//OSTaskCreate(key_task,(void *)0,(OS_STK*)&KEY_TASK_STK[KEY_STK_SIZE-1],KEY_TASK_PRIO);	 				   
	OSTaskCreate(PcCom_task,(void *)0,(OS_STK*)&PcCom_TASK_STK[PcCom_STK_SIZE-1],PcCom_TASK_PRIO);	
	OSTaskCreate(TaskPlan_task,(void *)0,(OS_STK*)&TaskPlan_TASK_STK[TaskPlan_STK_SIZE-1],TaskPlan_TASK_PRIO);	
	OSTaskCreate(MotionCtrl_task,(void *)0,(OS_STK*)&MotionCtrl_TASK_STK[MotionCtrl_STK_SIZE-1],MotionCtrl_TASK_PRIO);	
	OSTaskCreate(ServoCom_task,(void *)0,(OS_STK*)&ServoCom_TASK_STK[ServoCom_STK_SIZE-1],ServoCom_TASK_PRIO);
	OSTaskCreate(RdSensor_task,(void *)0,(OS_STK*)&RdSensor_TASK_STK[RdSensor_STK_SIZE-1],RdSensor_TASK_PRIO);		
 	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
}	  
//LED任务
void led_task(void *pdata)
{   
	static u32 Steps_Last = 0;
//	INT8U err;
	LED1 = 1; //red led off
	Steps_Last = ptrCfgMotionPar->RunSteps;
	ptrMotionBlk->StateOfCharge = 500; //剩余50%电量
	ptrMotionBlk->BatVolt = 240;//电池电压24V x0.1
	ptrMotionBlk->BatCurrent = 80;//电流8A x0.1
	ptrMotionBlk->BatTemp = 456;// 温度 x0.1
	
	DebugFlag = 0;
	while(1)
	{
		
		 //调试命令处理
		 if(DebugFlag == 0)
		 {
			 if(Para[DEBUG_EN_ADDR] == DEBUG_EN_CMD)
			 {
				 DebugFlag = 1;
				 Para[DEBUG_EN_ADDR] = 0;
			 }
		 }
		 else
		 {
			 if(Para[DEBUG_EN_ADDR] == DEBUG_DIS_CMD)
			 {
				 ptrMotionBlk->MotionCmdCode = 14; //取消调试模式时，从新回1次0
				 delay_ms(10);
				 DebugFlag = 0;
				 Para[DEBUG_EN_ADDR] = 0;
			 }				 
		 }
		 
		 //写eeprom命令
			if(Para[SAVE_PAR_ADDR] == SAVE_PAR_CMD)
			{				
				AT24CXX_Write(0,(u8 *)(Para),(SAVE_PAR_NUM*2));	//从指定地址开始写入指定长度的数据
				//void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//从指定地址开始读出指定长度的数据
				Para[SAVE_PAR_ADDR] = 0;
			}

			{
				
				if(Steps_Last != ptrCfgMotionPar->RunSteps)
				{
					Steps_Last = ptrCfgMotionPar->RunSteps;
					AT24CXX_Write(STEPS_CNT_BASE*2,(u8 *)(&Para[STEPS_CNT_BASE]),2);
				}
			}
			
			LED0=!LED0;//yellow led
			if(ptrMotionBlk->ErrCode != 0)
			{
				LED1 = ~LED1; // red led blink
			}
			else
			{
				LED1 = 1; //off state
			}
	   //LED0=!LED0;
//		 LED1=!LED1;

	   delay_ms(1000);
     //OSSemPend(sem_printf,0,&err);		 
		 //printf("led_task \n");
		 //OSSemPost(sem_printf);
	}									 
}	  


//按键扫描任务
void key_task(void *pdata)
{	
	//u8 err;
	u8 key;		    						 
	while(1)
	{		
		key=KEY_Scan(0);
		
		if(key==KEY0_PRES)
		{			

			printf("key0,dGlbReg[255]:%d \n",dGlbReg[255]);	
		}
		else if (key==KEY2_PRES)
		{
			printf("key2 \n");		 
		}
		else if (key==WKUP_PRES)
		{
			printf("keyup \n");		  
		}
		else if(key==KEY1_PRES)
		{
		 printf("key1 \n");					   				   
		}
		
		//OSSemPend(sem_printf,0,&err);		
		//printf("key_task \n");
		//OSSemPost(sem_printf);
 		delay_ms(1000);
	}
}


//通信任务
void PcCom_task(void *pdata)
{
//	static u8 aa = 0;
	RS485Init();	
	printf("uart1 init ok \n");
	while(1)
	{ 
		Protocol_RS485();
		// printf("In PcCom_task \n");
		 //USART1->DR = aa++;

	   //delay_ms(1000); 		 
	}									 
}

//任务规划 任务
void TaskPlan_task(void *pdata)
{
	while(1)
	{ 
		 //printf("In TaskPlan_task \n");	
	   delay_ms(1000); 		 
	}									 
}


void MotionCtrl_task(void *pdata)
{
	while(1)
	{
		//printf("In MotionCtrl_task \n");	
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

void ServoCom_task(void *pdata)
{
	CanInit();

	//恢复eeprom 参数
	AT24CXX_Read(0,(u8 *)(Para),(SAVE_PAR_NUM*2));   	//从指定地址开始读出指定长度的数据
	if(Para[Judge_SAVE_PAR_ADDR] != 0x55aa)
	{//首次初始化参数
		Para[Judge_SAVE_PAR_ADDR] = 0x55aa;
		Para[SLAVE_ADDR] = 1 ;//通信地址
		MotionInit();		
		AT24CXX_Write(0,(u8 *)(Para),(SAVE_PAR_NUM*2));	//从指定地址开始写入指定长度的数据		
	}
	
	VarClear();
	SlaveIdAddr = Para[SLAVE_ADDR];
	//MotorInit();	
	ptrCfgMotionPar->pwr_en = 3;
	
	while(1)
	{
	//	Para[9] = SlaveIdAddr;
		//HandleCan();
		HandleMotion();
	  //printf("In ServoCan_task \n");
		delay_ms(2); 	
	}
}

void RdSensor_task(void *pdata)
{
	while(1)
	{
		//printf("In RdSensor_task \n");			
		HandleGPIO();//
		ptrMotionBlk->Ai1 = Get_Adc(9);//读ADC
		delay_ms(100); 			
	}
}



