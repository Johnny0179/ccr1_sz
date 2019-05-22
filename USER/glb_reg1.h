
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLB_REG_H
#define __GLB_REG_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "os_cpu.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define BEEP_CTRL_ADDR 255
/* Exported functions ------------------------------------------------------- */


//void SysTick_Handler(void);
	 
#define HOLDING_REG_END 4096
	 	 
extern volatile u16 dGlbReg [];

#ifdef __cplusplus
}
#endif

#endif /* ____GLB_REG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
