#include "key.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
// ALIENTEK STM32F407������
//����������������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/3
//�汾��V1.0
//��Ȩ���У�����ؾ���?
// Copyright(C) �������������ӿƼ����޹�˾
// 2014-2024 All rights reserved
//////////////////////////////////////////////////////////////////////////////////

//������ʼ������
void KEY_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE,
                         ENABLE);  //ʹ��GPIOA,GPIOEʱ��

  GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;  // KEY0 KEY1 KEY2��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // 100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
  GPIO_Init(GPIOE, &GPIO_InitStructure);              //��ʼ��GPIOE2,3,4

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  // WK_UP��Ӧ����PA0
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;  //����
  GPIO_Init(GPIOA, &GPIO_InitStructure);          //��ʼ��GPIOA0
}
//������������
//���ذ���ֵ
// mode:0,��֧��������;1,֧��������;
// 0��û���κΰ�������
// 1��KEY0����
// 2��KEY1����
// 3��KEY2����
// 4��WKUP���� WK_UP
//ע��˺�������Ӧ���ȼ�?,KEY0>KEY1>KEY2>WK_UP!!
u8 KEY_Scan(u8 mode) {
  static u8 key_up = 1;  //�������ɿ���־
  if (mode) key_up = 1;  //֧������
  if (key_up && (KEY0 == 0 || KEY1 == 0 || KEY2 == 0 || WK_UP == 1)) {
    delay_ms(10);  //ȥ����
    key_up = 0;
    if (KEY0 == 0)
      return 1;
    else if (KEY1 == 0)
      return 2;
    else if (KEY2 == 0)
      return 3;
    else if (WK_UP == 1)
      return 4;
  } else if (KEY0 == 1 && KEY1 == 1 && KEY2 == 1 && WK_UP == 0)
    key_up = 1;
  return 0;  // �ް�������
}
