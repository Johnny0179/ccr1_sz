#include "led.h" 
#include "glb_reg.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

//��ʼ��PF9��PF10Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOFʱ��
  //GPIOF9,F10��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;//LED0��LED1��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIO	
	GPIO_SetBits(GPIOE,GPIO_Pin_2 | GPIO_Pin_3);//GPIOF9,F10���øߣ�����
	
	/*	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//ʹ��GPIOFʱ��
  //GPIOF9,F10��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;//LED0��LED1��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��GPIO	
	GPIO_SetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);//GPIOF9,F10���øߣ�����
	*/

}

void HardWare_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOG, ENABLE);//ʹ��GPIOʱ��
	
	//�������
	//GPIOA PA8
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//LED0��LED1��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO		
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);// �������ͨ	
	
	//PC6,PC7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;//LED0��LED1��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO		
	GPIO_ResetBits(GPIOC,GPIO_Pin_6|GPIO_Pin_7);// �������ͨ	
	
	
	//PG3,PG5,PG8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_8;//LED0��LED1��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��GPIO		
	GPIO_ResetBits(GPIOG,GPIO_Pin_8);// �������ͨ	
	GPIO_SetBits(GPIOG,GPIO_Pin_3 | GPIO_Pin_5); //2·��Դʹ��Ĭ����ʹ�ܹرյ�
	
	
	
	//GPIOE  PE2��PE3//LED��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;//LED0��LED1��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIO	
	//GPIO_SetBits(GPIOE,GPIO_Pin_2 | GPIO_Pin_3);//GPIOF9,F10���øߣ�����		
	

  //��������
	//PA15
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//LED0��LED1��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//GPIO_Mode_OUT;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO		

	//PD0��PD3��PD7
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_3|GPIO_Pin_7;//LED0��LED1��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//GPIO_Mode_OUT;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��GPIO		
	

	//PG6,PG7
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_12;//LED0��LED1��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//GPIO_Mode_OUT;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��GPIO		
	

	
	
}

void HandleGPIO(void)
{
	u16 tmp = 0;
/////////////////////////////////////////////////////////////
// DI ��������ʱ�����˲�	
	if(DI0!=0)
	{
		tmp |= 0x0001;
	}

	if(DI1 !=0)
	{
		tmp |= 0x0002;
	}
	
	if(DI2 !=0)
	{
		tmp |= 0x0004;
	}	

	if(DI3 !=0)
	{
		tmp |= 0x0008;
	}	
	
	if(DI4 !=0)
	{
		tmp |= 0x0010;
	}	
	
	if(DI5 !=0)
	{
		tmp |= 0x0020;
	}	

	if(DI6 !=0)
	{
		tmp |= 0x0040;
	}		
	
	if(DI7 !=0)
	{
		tmp |= 0x0080;
	}		
	
	//REG_DI = tmp;
	ptrMotionBlk->Di = (~tmp)&0x00ff; //20171213
	//REG_DI = (~tmp)&0x00ff; //����DI�Ĵ���
/////////////////////////////////////////////////////////////	

/////////////////////////////////////////////////////////////	
//DO ���
	if((ptrMotionBlk->Do & 0x0001) != 0)
	{
		DO0 = 1;
	}
	else
	{
		DO0 = 0;
	}
	
	if((ptrMotionBlk->Do & 0x0002) != 0)
	{
		DO1 = 1;
	}
	else
	{
		DO1 = 0;
	}	
	
	
	if((ptrMotionBlk->Do & 0x0004) != 0)
	{
		DO2 = 1;
	}
	else
	{
		DO2 = 0;
	}	
	
	if((ptrMotionBlk->Do & 0x0008) != 0)
	{
		DO3 = 1;
	}
	else
	{
		DO3 = 0;
	}		
/////////////////////////////////////////////////////////////
	
/////////////////////////////////////////////////////////////
	//��Դʹ��
	if((ptrCfgMotionPar->pwr_en & 0x0001) != 0)
	{
		EN_5V_OUT = 1;
	}
	else
	{
		EN_5V_OUT = 0;
	}	
	
	if((ptrCfgMotionPar->pwr_en & 0x0002) != 0)
	{
		EN_12V_OUT = 1;
	}
	else
	{
		EN_12V_OUT = 0;
	}		
	
	
	
	
	
}








