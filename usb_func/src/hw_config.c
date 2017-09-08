/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : hw_config.c
* Author             : MCD Application Team
* Version            : V3.2.1
* Date               : 07/05/2010
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "platform_config.h"
#include "usb_pwr.h"

///之前定义的变量
USART_InitTypeDef USART_InitStructure;
extern uint8_t  USART_Rx_Buffer [USART_RX_DATA_SIZE]; 
uint32_t USART_Rx_ptr_in ;
extern uint32_t USART_Rx_ptr_out ;
extern uint32_t USART_Rx_length ;
extern uint8_t  USB_Tx_State ;
void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
extern LINE_CODING linecoding;
extern uint8_t USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
extern uint8_t USB_Tx_Statement;
extern uint16_t USB_Rx_Cnt ;
extern enum USER_statment {
                    NONE,
                    CAN_start,
                    CAN_stop,
                    CAN_set,
                    CAN_rece,
                    }
                    user_statment; 

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
}


/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
#ifdef STM32F10X_CL  
  if (NewState != DISABLE)
  {
    USB_DevConnect();
  }
  else
  {
    USB_DevDisconnect();
  }
#else /* USE_STM3210B_EVAL or USE_STM3210E_EVAL */
  if (NewState != DISABLE)
  {
    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
  else
  {
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
#endif /* USE_STM3210C_EVAL */
}

/*******************************************************************************
* Function Name  :  USART_Config_Default.
* Description    :  configure the EVAL_COM1 with default values.
* Input          :  None.
* Return         :  None.
*******************************************************************************/
void USART_Config_Default(void)
{
  /* EVAL_COM1 default configuration */
  /* EVAL_COM1 configured as follow:
        - BaudRate = 9600 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - Parity Odd
        - Hardware flow control desabled
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_Odd;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure and enable the USART */
//  STM_EVAL_COMInit(COM1, &USART_InitStructure);

  /* Enable the USART Receive interrupt */
///  USART_ITConfig(EVAL_COM1, USART_IT_RXNE, ENABLE);
}

/*******************************************************************************
* Function Name  :  USART_Config.
* Description    :  Configure the EVAL_COM1 according to the linecoding structure.
* Input          :  None.
* Return         :  Configuration status
                    TRUE : configuration done with success
                    FALSE : configuration aborted.
*******************************************************************************/
bool USART_Config(void)
{

  /* set the Stop bit*/
  switch (linecoding.format)
  {
    case 0:
      USART_InitStructure.USART_StopBits = USART_StopBits_1;
      break;
    case 1:
      USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
      break;
    case 2:
      USART_InitStructure.USART_StopBits = USART_StopBits_2;
      break;
    default :
    {
      USART_Config_Default();
      return (FALSE);
    }
  }

  /* set the parity bit*/
  switch (linecoding.paritytype)
  {
    case 0:
      USART_InitStructure.USART_Parity = USART_Parity_No;
      break;
    case 1:
      USART_InitStructure.USART_Parity = USART_Parity_Even;
      break;
    case 2:
      USART_InitStructure.USART_Parity = USART_Parity_Odd;
      break;
    default :
    {
      USART_Config_Default();
      return (FALSE);
    }
  }

  /*set the data type : only 8bits and 9bits is supported */
  switch (linecoding.datatype)
  {
    case 0x07:
      /* With this configuration a parity (Even or Odd) should be set */
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      break;
    case 0x08:
      if (USART_InitStructure.USART_Parity == USART_Parity_No)
      {
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      }
      else 
      {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
      }
      
      break;
    default :
    {
      USART_Config_Default();
      return (FALSE);
    }
  }

  USART_InitStructure.USART_BaudRate = linecoding.bitrate;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
  /* Configure and enable the USART */
///  STM_EVAL_COMInit(COM1, &USART_InitStructure);

  return (TRUE);
}

/*******************************************************************************
* Function Name  : USB_To_USART_Send_Data.
* Description    : send the received data from USB to the UART 0.
* Input          : data_buffer: data address.
                   Nb_bytes: number of bytes to send.
* Return         : none.
*******************************************************************************/
void USB_To_USART_Send_Data(uint8_t* data_buffer, uint8_t Nb_bytes)
{
  
  uint32_t i;
  
  for (i = 0; i < Nb_bytes; i++)
  {
///    USART_SendData(EVAL_COM1, *(data_buffer + i));
///    while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TXE) == RESET); 
  }  
}

/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void Handle_USBAsynchXfer (void)
{
  
//  uint16_t USB_Tx_ptr;
//  uint16_t USB_Tx_length;
	
	if (user_statment==CAN_rece)
	{
	   USB_Tx_Statement = 1;
	};
	
	if (!USB_Tx_Statement)                  ///当USB_Tx_Statement=0时,不向PC发送数据
	{
	   return ;
	}	
	USB_Tx_Statement = 0; 
    
#ifdef STM32F10X_CL
    USB_SIL_Write(EP1_IN, USB_Rx_Buffer, USB_Rx_Cnt);  
#else
    UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
    SetEPTxCount(ENDP1, USB_Tx_length);
    SetEPTxValid(ENDP1); 
#endif
  	USB_Rx_Cnt=0 ;
}
/*******************************************************************************
* Function Name  : UART_To_USB_Send_Data.
* Description    : send the received data from UART 0 to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void USART_To_USB_Send_Data(void)
{
  
  if (linecoding.datatype == 7)
  {
///    USART_Rx_Buffer[USART_Rx_ptr_in] = USART_ReceiveData(EVAL_COM1) & 0x7F;
  }
  else if (linecoding.datatype == 8)
  {
///    USART_Rx_Buffer[USART_Rx_ptr_in] = USART_ReceiveData(EVAL_COM1);
  }
  
  USART_Rx_ptr_in++;
  
  /* To avoid buffer overflow */
  if(USART_Rx_ptr_in == USART_RX_DATA_SIZE)
  {
    USART_Rx_ptr_in = 0;
  }
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
  Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
  Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)        ///例: 28 6B BB 29
{                                                                              ///
  uint8_t idx = 0;                                                             ///
                                                                               ///
  for( idx = 0 ; idx < len ; idx ++)                                           ///
  {                                                                            ///
    if( ((value >> 28)) < 0xA )                                                /// 28 6B BB 29 >> 28 --> 00 00 00 02
    {                                                                          ///
      pbuf[ 2* idx] = (value >> 28) + '0';                                     /// pbuf[ 2* idx] = 2+'0' = 0x32 = '2'    ///yu hex转换为字符
    }                                                                          ///
    else                                                                       ///
    {                                                                          ///
      pbuf[2* idx] = (value >> 28) + 'A' - 10;                                 ///
    }                                                                          ///
                                                                               ///
    value = value << 4;                                                        /// 28 6B BB 29 << 4 --> 86 BB B2 90
                                                                               ///
    pbuf[ 2* idx + 1] = 0;                                                     /// ///yu 给0?
  }                                                                            ///
}                                                                              ///
                                                                               ///
#ifdef STM32F10X_CL                                                            ///
/*******************************************************************************
* Function Name  : USB_OTG_BSP_uDelay.
* Description    : provide delay (usec).
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_OTG_BSP_uDelay (const uint32_t usec)
{
  RCC_ClocksTypeDef  RCC_Clocks;  

  /* Configure HCLK clock as SysTick clock source */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  
  RCC_GetClocksFreq(&RCC_Clocks);
  
  SysTick_Config(usec * (RCC_Clocks.HCLK_Frequency / 1000000));  
  
  SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk ;
  
  while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
}
#endif
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

