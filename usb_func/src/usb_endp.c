/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V3.2.1
* Date               : 07/05/2010
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"

///֮ǰ�õı���

extern  uint8_t USART_Rx_Buffer[];
extern uint32_t USART_Rx_ptr_out;
extern uint32_t USART_Rx_length;
extern uint8_t  USB_Tx_State;
extern uint8_t RX_data[8]  ;
#define VCOMPORT_IN_FRAME_INTERVAL             5
extern enum {CAN_disable = 0, CAN_enable = !CAN_disable} CAN_STATUS;
extern CAN1_RX ;
extern CAN_doing ;
//extern u8 CAN_statment	   ;


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USB_Rx_Buffer_Size  80
/* Interval between sending IN packets in frame number (1 frame = 1ms) */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern enum USER_statment {
                    NONE,
                    CAN_start,
                    CAN_stop,
                    CAN_set,
                    CAN_rece,
                    }
                    user_statment;
uint8_t USB_Rx_Buffer[USB_Rx_Buffer_Size];
uint16_t USB_Rx_Cnt;
uint8_t USB_Tx_Statement;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
/// �ε�1 IN callback������������Ҫ�ڷ������һ�����ݺ��к�������������������Ӵ��롣
void EP1_IN_Callback (void)
{
  uint16_t USB_Tx_ptr;
  uint16_t USB_Tx_length;
  if ( 1 )
	 {
	   USART_Rx_length = 0;
	   return ;
	 }
  USB_Tx_length = 2 ;
  CAN1_RX = CAN_disable ;
#ifdef STM32F10X_CL
      USB_SIL_Write(EP1_IN, &RX_data[0], USB_Tx_length);
	    
#else
      UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
      SetEPTxCount(ENDP1, USB_Tx_length);
      SetEPTxValid(ENDP1); 
#endif  
  
  USART_Rx_length = 0;

}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
///�ε�3 OUT callback�������ڽ��յ�һ�����ݺ�����������������ȡUSB���ݡ�
void EP3_OUT_Callback(void)
{
  
  /* Get the received data buffer and update the counter */
  USB_Rx_Cnt = USB_SIL_Read(EP3_OUT, USB_Rx_Buffer+1);
  if ((!strncmp(USB_Rx_Buffer+1,"STARTCAN",USB_Rx_Cnt))&&(USB_Rx_Cnt==8))       ///����֡��
  {
	 user_statment = CAN_start ;
  }
  else if ((!strncmp(USB_Rx_Buffer+1,"STOPCAN",USB_Rx_Cnt))&&(USB_Rx_Cnt==7))
  {
	 user_statment = CAN_stop ;
  }
  else if (!strncmp(USB_Rx_Buffer+1,"SETCAN",6))
  {
     user_statment = CAN_set ;
  }  
  else  
  {
     user_statment = NONE ;
  };

  /* USB data will be immediately processed, this allow next USB traffic beeing 
  NAKed till the end of the USART Xfet */
  
  USB_Rx_Cnt++;
  *(USB_Rx_Buffer)='\n'	 ;
  USB_Tx_Statement = 1;

#ifndef STM32F10X_CL
  /* Enable the receive of data on EP3 */
  SetEPRxValid(ENDP3);
#endif /* STM32F10X_CL */
}


/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
#ifdef STM32F10X_CL
void INTR_SOFINTR_Callback(void)
#else
void SOF_Callback(void)
#endif /* STM32F10X_CL */
{
  static uint32_t FrameCount = 0;
  
  if(bDeviceState == CONFIGURED)
  {
    
    if ((FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)||(user_statment==CAN_rece))
    {
      /* Reset the frame counter */
      FrameCount = 0;
      
      /* Check the data to be sent through IN pipe */
      Handle_USBAsynchXfer();
    }
  }  
}

///SOF��֡�ף�����ȫ���豸��˵��ÿ1������1��֡���źţ���ˣ�ÿ1��������1�����֡���жϻص�������
///���������ж�������ʱ��ÿ1�����VCOMPORT_IN_FRAME_INTERVAL�ļ��ʱ�䣬�ж����Ƿ��д�USART�յ�������Ҫͨ��USB����������                             
///
///
///
///
///
///
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

