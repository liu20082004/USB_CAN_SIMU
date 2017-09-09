/******************** (C) COPYRIGHT 2013 www.armjishu.com  ***********************
 * �ļ���  ��stm32f10x_it.c
 * ����    ��ʵ��STM32F107VC����IV�ſ�����STM32�жϴ������ļ�
 *           ����ͨ�õ�һ�㲻��Ҫ�޸ĵ��жϴ���������"SZ_STM32F107VC_LIB.c"�ļ���
 * ʵ��ƽ̨��STM32���ۿ�����
 * ��׼��  ��STM32F10x_StdPeriph_Driver V3.5.0
 * ����    ��www.armjishu.com 
**********************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "SZ_STM32F107VC_LIB.h"

uint8_t  USART_Rx_Buffer[30]; 
//uint32_t USART_Rx_ptr_in = 0;
uint32_t USART_Rx_ptr_out = 0;
uint32_t USART_Rx_length  = 0;

uint8_t  USB_Tx_State = 0;




/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern int USB_Rx_Cnt ;
extern uint8_t USB_Rx_Buffer[];
extern enum USER_statment {
                    NONE,
                    CAN_start,
                    CAN_stop,
                    CAN_set,
                    CAN_rece,
                    
                    }
                    user_statment; 
extern CanRxMsg RxMessage_CAN;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
//extern void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len) ;
void Hex_to_Char (uint32_t value , uint8_t *pbuf , uint8_t len) ;
/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**-------------------------------------------------------
  * @������ NMI_Handler
  * @����   ���������жϵ��жϴ�����
  * @����   ��
  * @����ֵ ��
***------------------------------------------------------*/
void NMI_Handler(void)
{
}

/**-------------------------------------------------------
  * @������ HardFault_Handler
  * @����   Ӳ�������жϵ��жϴ�����
  *         ����Ӻ���������Ϊ��whileѭ�������ڷ�������׽
  * @����   ��
  * @����ֵ ��
***------------------------------------------------------*/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**-------------------------------------------------------
  * @������ MemManage_Handler
  * @����   �洢�����ʴ����жϵ��жϴ�����
  *         ����Ӻ���������Ϊ��whileѭ�������ڷ�������׽
  * @����   ��
  * @����ֵ ��
***------------------------------------------------------*/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**-------------------------------------------------------
  * @������ BusFault_Handler
  * @����   ���߷��ʴ����жϵ��жϴ�����
  *         ����Ӻ���������Ϊ��whileѭ�������ڷ�������׽
  * @����   ��
  * @����ֵ ��
***------------------------------------------------------*/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**-------------------------------------------------------
  * @������ UsageFault_Handler
  * @����   �÷������жϵ��жϴ�����
  *         ����Ӻ���������Ϊ��whileѭ�������ڷ�������׽
  * @����   ��
  * @����ֵ ��
***------------------------------------------------------*/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**-------------------------------------------------------
  * @������ SVC_Handler
  * @����   ϵͳ�����жϵ��жϴ�����
  *         һ��ᱻ����ϵͳʹ��
  * @����   ��
  * @����ֵ ��
***------------------------------------------------------*/
void SVC_Handler(void)
{
}

/**-------------------------------------------------------
  * @������ DebugMon_Handler
  * @����   ���Լ���жϵ��жϴ�����
  * @����   ��
  * @����ֵ ��
***------------------------------------------------------*/
void DebugMon_Handler(void)
{
}

/**-------------------------------------------------------
  * @������ PendSV_Handler
  * @����   �ɹ����ϵͳ������������
  * @����   ��
  * @����ֵ ��
***------------------------------------------------------*/
void PendSV_Handler(void)
{
}

/**-------------------------------------------------------
  * @������ SysTick_Handler
  * @����   ϵͳ���Ķ�ʱ��������������
  * @����   ��
  * @����ֵ ��
***------------------------------------------------------*/
// ��SZ_STM32F107VC_LIB.c�ж���
//void SysTick_Handler(void)
//{
//}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/******************* (C) COPYRIGHT 2013 www.armjishu.com *****END OF FILE****/
#ifdef STM32F10X_CL
/*******************************************************************************
* Function Name  : OTG_FS_IRQHandler
* Description    : This function handles USB-On-The-Go 
                   FS global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OTG_FS_IRQHandler(void)
{
  STM32_PCD_OTG_ISR_Handler(); 
}
#endif /* STM32F10X_CL */


/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles CAN1 Handler.
  * @param  None
  * @retval None
  */
#ifndef STM32F10X_CL
void USB_LP_CAN1_RX0_IRQHandler(void)
#else
void CAN1_RX0_IRQHandler(void)
#endif
{
  uint8_t i;
  CanRxMsg RxMessage;
  
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);                                     ///CAN1��������
  for (i=0;i<40;i++)
  {
    USB_Rx_Buffer[i]=' ';                                                       ///���USB_Rx_Buffer
  };
  
  if (RxMessage.IDE==CAN_ID_STD)
  {
    Hex_to_Char(RxMessage.StdId, &USB_Rx_Buffer[0], 8) ;                        ///��׼֡ID
  }
  else
  {
    Hex_to_Char(RxMessage.ExtId, &USB_Rx_Buffer[0], 8) ;                        ///��չ֡ID
  }

  for (i=0;i<RxMessage.DLC;i++)                                                 ///�����򳤶�
  {
    Hex_to_Char(((uint32_t)RxMessage.Data[i]<<24), &USB_Rx_Buffer[15+i*3], 2) ; ///������(ע:��������DataΪuint8_t,��ǿ��ת��Ϊuint32_t������28λ,����Hex_to_Char����)
    //Hex_to_Char(RxMessage.Data[i], &USB_Rx_Buffer[15+i*3], 2);    ����ͨ��,�������������Ƿ����
    USB_Rx_Buffer[15+i*3+2]=' ';
  }
  
  USB_Rx_Buffer[15+i*3+2]='\n';
  USB_Rx_Cnt =15+i*3+2+1;

//  USB_Rx_Buffer[38]='\n';
//  USB_Rx_Cnt =39;
  user_statment=CAN_rece ;

}

void Hex_to_Char (uint32_t value , uint8_t *pbuf , uint8_t len)       		   ///��: 28 6B BB 29
{                                                                              ///
  uint8_t idx = 0;                                                             ///
                                                                               ///
  for( idx = 0 ; idx < len ; idx ++)                                           ///
  {                                                                            ///
    if( ((value >> 28)) < 0xA )                                                /// 28 6B BB 29 >> 28 --> 00 00 00 02
    {                                                                          ///
      pbuf[idx] = (value >> 28) + '0';                                         /// pbuf[ 2* idx] = 2+'0' = 0x32 = '2'    ///yu hexת��Ϊ�ַ�
    }                                                                          ///
    else                                                                       ///
    {                                                                          ///
      pbuf[ idx] = (value >> 28) + 'A' - 10;                                   ///
    }                                                                          ///
                                                                               ///
    value = value << 4;                                                        /// 28 6B BB 29 << 4 --> 86 BB B2 90
  }                                                                            ///
}                                                                              ///




/**
  * @brief  This function handles CAN2 RX0 Handler.
  * @param  None
  * @retval None
  */

void CAN2_RX0_IRQHandler(void)
{
  	int i;
  	CanRxMsg RxMessage ;
  	CanTxMsg TxMessage;
  	CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);

	
	TIM_Cmd(TIM5, DISABLE);  //��ͣ��ʱ��
	// ����LED3
	LED3OBB = 0;



	RxMessage_CAN = RxMessage ;
	simu_CAN20_V2();

	// Ϩ��LED3
	LED3OBB = 1;
	
	TIM5->CNT = 0 ;
	TIM_Cmd(TIM5, ENABLE);  //������ʱ��

	//���USB_Rx_Buffer,USB_Rx_Cnt,��ʹuser_statment=CAN_rece,usbת���ھͻ�������ϴ�PC.������ƻ����Ǻ����

//	Hex_to_Char(RxMessage.StdId, &USB_Rx_Buffer[0], 8) ;                        ///��׼֡ID
//  	for (i=0;i<RxMessage.DLC;i++)                                                 ///�����򳤶�
//  	{
//    	Hex_to_Char(((uint32_t)RxMessage.Data[i]<<24), &USB_Rx_Buffer[15+i*3], 2) ; ///������(ע:��������DataΪuint8_t,��ǿ��ת��Ϊuint32_t������28λ,����Hex_to_Char����)
//    	//Hex_to_Char(RxMessage.Data[i], &USB_Rx_Buffer[15+i*3], 2);    ����ͨ��,�������������Ƿ����
//    	USB_Rx_Buffer[15+i*3+2]=' ';
//  	}
//  
//  	USB_Rx_Buffer[15+i*3+2]='\n';
//  	USB_Rx_Cnt =15+i*3+2+1;
//	user_statment = CAN_rece ;
	
  
//  
//  USB_Tx_State =0;
//  USART_Rx_length=0x0d ;
//  USB_Rx_Buffer[3]++ ;
//  user_statment=2;
}

/******************* (C) COPYRIGHT 2013 www.armjishu.com *****END OF FILE****/
