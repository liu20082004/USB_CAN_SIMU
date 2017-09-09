/******************** (C) COPYRIGHT 2013 www.armjishu.com  ***********************
 * 文件名  ：stm32f10x_it.c
 * 描述    ：实现STM32F107VC神舟IV号开发板STM32中断处理函数文件
 *           部分通用的一般不需要修改的中断处理函数放在"SZ_STM32F107VC_LIB.c"文件中
 * 实验平台：STM32神舟开发板
 * 标准库  ：STM32F10x_StdPeriph_Driver V3.5.0
 * 作者    ：www.armjishu.com 
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
  * @函数名 NMI_Handler
  * @功能   不可屏蔽中断的中断处理函数
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void NMI_Handler(void)
{
}

/**-------------------------------------------------------
  * @函数名 HardFault_Handler
  * @功能   硬件错误中断的中断处理函数
  *         可添加函数，现在为空while循环，便于仿真器捕捉
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**-------------------------------------------------------
  * @函数名 MemManage_Handler
  * @功能   存储器访问错误中断的中断处理函数
  *         可添加函数，现在为空while循环，便于仿真器捕捉
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**-------------------------------------------------------
  * @函数名 BusFault_Handler
  * @功能   总线访问错误中断的中断处理函数
  *         可添加函数，现在为空while循环，便于仿真器捕捉
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**-------------------------------------------------------
  * @函数名 UsageFault_Handler
  * @功能   用法错误中断的中断处理函数
  *         可添加函数，现在为空while循环，便于仿真器捕捉
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**-------------------------------------------------------
  * @函数名 SVC_Handler
  * @功能   系统服务中断的中断处理函数
  *         一般会被操作系统使用
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void SVC_Handler(void)
{
}

/**-------------------------------------------------------
  * @函数名 DebugMon_Handler
  * @功能   调试监控中断的中断处理函数
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void DebugMon_Handler(void)
{
}

/**-------------------------------------------------------
  * @函数名 PendSV_Handler
  * @功能   可挂起的系统服务请求处理函数
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void PendSV_Handler(void)
{
}

/**-------------------------------------------------------
  * @函数名 SysTick_Handler
  * @功能   系统节拍定时器服务请求处理函数
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
// 在SZ_STM32F107VC_LIB.c中定义
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
  
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);                                     ///CAN1接收数据
  for (i=0;i<40;i++)
  {
    USB_Rx_Buffer[i]=' ';                                                       ///填空USB_Rx_Buffer
  };
  
  if (RxMessage.IDE==CAN_ID_STD)
  {
    Hex_to_Char(RxMessage.StdId, &USB_Rx_Buffer[0], 8) ;                        ///标准帧ID
  }
  else
  {
    Hex_to_Char(RxMessage.ExtId, &USB_Rx_Buffer[0], 8) ;                        ///扩展帧ID
  }

  for (i=0;i<RxMessage.DLC;i++)                                                 ///数据域长度
  {
    Hex_to_Char(((uint32_t)RxMessage.Data[i]<<24), &USB_Rx_Buffer[15+i*3], 2) ; ///数据域(注:数据域中Data为uint8_t,在强行转换为uint32_t后左移28位,便于Hex_to_Char处理)
    //Hex_to_Char(RxMessage.Data[i], &USB_Rx_Buffer[15+i*3], 2);    编译通过,试试这样处理是否可行
    USB_Rx_Buffer[15+i*3+2]=' ';
  }
  
  USB_Rx_Buffer[15+i*3+2]='\n';
  USB_Rx_Cnt =15+i*3+2+1;

//  USB_Rx_Buffer[38]='\n';
//  USB_Rx_Cnt =39;
  user_statment=CAN_rece ;

}

void Hex_to_Char (uint32_t value , uint8_t *pbuf , uint8_t len)       		   ///例: 28 6B BB 29
{                                                                              ///
  uint8_t idx = 0;                                                             ///
                                                                               ///
  for( idx = 0 ; idx < len ; idx ++)                                           ///
  {                                                                            ///
    if( ((value >> 28)) < 0xA )                                                /// 28 6B BB 29 >> 28 --> 00 00 00 02
    {                                                                          ///
      pbuf[idx] = (value >> 28) + '0';                                         /// pbuf[ 2* idx] = 2+'0' = 0x32 = '2'    ///yu hex转换为字符
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

	
	TIM_Cmd(TIM5, DISABLE);  //暂停定时器
	// 点亮LED3
	LED3OBB = 0;



	RxMessage_CAN = RxMessage ;
	simu_CAN20_V2();

	// 熄灭LED3
	LED3OBB = 1;
	
	TIM5->CNT = 0 ;
	TIM_Cmd(TIM5, ENABLE);  //启动定时器

	//填充USB_Rx_Buffer,USB_Rx_Cnt,并使user_statment=CAN_rece,usb转串口就会把数据上传PC.具体机制还不是很清除

//	Hex_to_Char(RxMessage.StdId, &USB_Rx_Buffer[0], 8) ;                        ///标准帧ID
//  	for (i=0;i<RxMessage.DLC;i++)                                                 ///数据域长度
//  	{
//    	Hex_to_Char(((uint32_t)RxMessage.Data[i]<<24), &USB_Rx_Buffer[15+i*3], 2) ; ///数据域(注:数据域中Data为uint8_t,在强行转换为uint32_t后左移28位,便于Hex_to_Char处理)
//    	//Hex_to_Char(RxMessage.Data[i], &USB_Rx_Buffer[15+i*3], 2);    编译通过,试试这样处理是否可行
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
