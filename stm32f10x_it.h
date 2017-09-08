/******************** (C) COPYRIGHT 2013 www.armjishu.com  ***********************
 * 文件名  ：stm32f10x_it.h
 * 描述    ：实现STM32F107VC神舟IV号开发板STM32中断处理函数文件
 *           部分通用的一般不需要修改的中断处理函数放在"SZ_STM32F107VC_LIB.c"文件中
 * 实验平台：STM32神舟开发板
 * 标准库  ：STM32F10x_StdPeriph_Driver V3.5.0
 * 作者    ：www.armjishu.com 
**********************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
// 在SZ_STM32F107VC_LIB.c中定义
//void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_IT_H */

/******************* (C) COPYRIGHT 2013 www.armjishu.com *****END OF FILE****/
