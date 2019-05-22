#ifndef __RS485__
#define __RS485__

#include "stm32f4xx.h"

#ifndef Para
#define Para dGlbReg
#endif

#define S_RX 0
#define S_Frame 1
#define S_TX 2

#define COMM_BUFF_LEN 256 //通信缓冲区长度

#define MAX_ACCESS_REG_NUM ((COMM_BUFF_LEN - 6) / 2) - 4 //MODBUS 主设备单次可访问的最多寄存器数量

#define SEND_DATA(X) USART1->DR = X //#define SEND_DATA(X) USART2->DR=X
//#define COMM_ID  1  //从机地址   用modbusAddr替代
#define COMMAND 1
#define CRCH 6
#define CRCL 7
#define ADDREH 2
#define ADDREL ADDREH + 1
#define SEND_DATAH ADDREL + 1
#define SEND_DATAL SEND_DATAH + 1
#define RECE_WORD ADDREL + 2
#define RECE_BYTE RECE_WORD + 1

//理论时间   采纳时间   软件实现时间

#define T35_2400 60   //>1.75 ms   3ms        3~2ms
#define T35_4800 40   //>1.75 ms   3ms        3~2ms
#define T35_9600 20   //>4ms       5ms        5~4ms
#define T35_19200 4   // 20         //>2ms       3ms        3~2ms
#define T35_38400 15  //>1.75 ms   3ms        3~2ms
#define T35_57600 10  //>1.75 ms   3ms        3~2ms
#define T35_115200 10 //>1.75 ms   3ms        3~2ms

volatile typedef struct
{
	u8 Rx2[COMM_BUFF_LEN];
	u8 Tx2[COMM_BUFF_LEN];
	u16 index;
	u16 state;
	u16 us_end; //帧间隔时间
	u16 len;
	u16 send_len; //发送的字节数
	u16 send_end;
	u8 err;
	u16 busy : 1; //发送忙
	u16 send_flag : 1;
	u16 timeout_en : 1;

} RS_STR;

extern volatile RS_STR uart2rx;

extern void uart1_init(u32 bound);
extern void RS485Init(void);
extern void TIM3_Int_Init(u16 arr, u16 psc);
extern void Protocol_RS485(void);
#endif

/*


#define RE() DMA1_Channel5->CNDTR//#define RE() DMA1_Channel6->CNDTR
#define WR(X) DMA1_Channel5->CNDTR = X//#define WR(X) DMA1_Channel6->CNDTR = X
#define USART2_DMA_EN() DMA1_Channel5->CCR |= 1;//#define USART2_DMA_EN() DMA1_Channel6->CCR |= 1;
#define USART2_DMA_DI()  DMA1_Channel5->CCR &= ((u32)0xFFFFFFFE);//#define USART2_DMA_DI()  DMA1_Channel6->CCR &= ((u32)0xFFFFFFFE);

//STM32 GPIO define
//-------------------------------------------
#define  RS485_PORT            GPIOB //GPIOD
#define  GPIO_TxPin             GPIO_Pin_6//GPIO_Pin_5
#define  GPIO_RxPin             GPIO_Pin_7//GPIO_Pin_6

#define  RS485_EN_PORT          GPIOC//GPIOD
#define  RS485_EN_Pin          GPIO_Pin_10//GPIO_Pin_10


#define RECE_EN() RS485_EN_PORT->BRR = RS485_EN_Pin	   //hw v3.0  20150404
#define SEND_EN() RS485_EN_PORT->BSRR = RS485_EN_Pin
//#define RECE_EN() RS485_EN_PORT->BSRR = RS485_EN_Pin
//#define SEND_EN() RS485_EN_PORT->BRR = RS485_EN_Pin
//-------------------------------------------







extern void rs485_dma_init(void);
extern void Frame_Detect(void);
extern void Modbus_Send(void);
extern void Frame_deal(void);
 */
