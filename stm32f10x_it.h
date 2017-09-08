/******************** (C) COPYRIGHT 2013 www.armjishu.com  ***********************
 * �ļ���  ��stm32f10x_it.h
 * ����    ��ʵ��STM32F107VC����IV�ſ�����STM32�жϴ������ļ�
 *           ����ͨ�õ�һ�㲻��Ҫ�޸ĵ��жϴ���������"SZ_STM32F107VC_LIB.c"�ļ���
 * ʵ��ƽ̨��STM32���ۿ�����
 * ��׼��  ��STM32F10x_StdPeriph_Driver V3.5.0
 * ����    ��www.armjishu.com 
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
// ��SZ_STM32F107VC_LIB.c�ж���
//void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_IT_H */

/******************* (C) COPYRIGHT 2013 www.armjishu.com *****END OF FILE****/
