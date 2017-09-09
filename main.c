/******************** (C) COPYRIGHT 2013 www.armjishu.com  ********************
 * �ļ���  ��main.c
 * ����    ��ʵ��STM32F107VC����IV�ſ������ϵ�UART����1�ʹ���2ͬʱ��ʽ��������빦��
 * ʵ��ƽ̨��STM32���ۿ�����
 * ��׼��  ��STM32F10x_StdPeriph_Driver V3.5.0
 * ����    ��www.armjishu.com
 * ��ֲ�޸�˵��: 
 * ��Ҫ�Ż��ĵط�: 1).�Ż�����ṹ,ʹ֮ģ�黯; 2).ģ�����ݿɶ�̬�޸�,�ɽ���PC��ʵʱ�޸�; 3).CAN�ж���Ҫ�Ż�,�ж���ֻ�����������,�����Ĺ��ܷŵ��ж��ⲿ
**********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "SZ_STM32F107VC_LIB.h"
#include "xprintf.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
//#include "CAN20.h"
//#include "tim_clock.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CanRxMsg RxMessage_CAN;
extern uint8_t USB_Rx_Buffer[];
ErrorStatus HSEStartUpStatus;
u8 CAN_statment ;
enum USER_statment {
                    NONE,
                    CAN_start,
                    CAN_stop,
                    CAN_set,
					CAN_rece,
                    
                    }
                    user_statment;

/* Private function prototypes -----------------------------------------------*/
void SysTick_Handler_User(void);
void Set_System(void);
void Set_USBClock(void);
void USB_Interrupts_Config(void);
void GPIOcan_Configuration(void);
void NVIC_Configuration(void);
void SysTick_close() ;
void CAN_Config();
void key_canconfig(void) ;	
uint8_t CAN_Uesr_Config(uint8_t *pbuf);


/**-------------------------------------------------------
  * @������ main
  * @����   ������
  * @����   ��
  * @����ֵ ��
***------------------------------------------------------*/
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_stm32f10x_xx.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_stm32f10x.c file
       */
    /*!< ��ϵͳ�����ļ�(startup_stm32f10x_xx.s)���Ѿ�����SystemInit()��ʼ����ʱ��,
         ����main��������Ҫ�ٴ��ظ���ʼ��ʱ�ӡ�Ĭ�ϳ�ʼ��ϵͳ��ʱ��Ϊ72MHz��
         SystemInit()������ʵ��λ��system_stm32f10x.c�ļ��С�
       */
    
	u8 i,j ;
	//CanTxMsg TxMessage;
	//TxMessage.StdId = 0x7F0 ;
	user_statment=NONE ;	

	/* ��ʼ������LEDָʾ�� */
    SZ_STM32_LEDInit(LED1);
    SZ_STM32_LEDInit(LED2);
    SZ_STM32_LEDInit(LED3);
    SZ_STM32_LEDInit(LED4);

	Set_System(); /* ϵͳ��ʼ�� */                                  ///ϵͳ����,��Ҫ������ϵͳʱ��
    Set_USBClock();                                                 ///����USBʱ��
    USB_Interrupts_Config();                                        ///������Ժ������NVIC_Configuration����һ��
    USB_Init();                                                     ///USB�������ĳ�ʼ��(��usb_init.h��)

  	/* GPIOB, GPIOD and AFIO clocks enable */
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	GPIOcan_Configuration();                                        ///��������,ʹ��CANʱ��
	NVIC_Configuration();                                           ///�жϹ���

	SZ_STM32_LEDOff(LED1);
	SZ_STM32_LEDOff(LED2);
	SZ_STM32_LEDOff(LED3);
	SZ_STM32_LEDOff(LED4);
	TIM5_Init();
	
	while (1)
	{
	 	if (CONFIGURED != bDeviceState)
		{
			LED1OBB = 1;  										///�豸δ����PC�ɹ�,LED1��
		}
		else
		{
		 	LED1OBB = 0;
		}


		if ( (user_statment==CAN_start) )
		{
			SZ_STM32_SysTickInit(1000);
	 		//key_canconfig();
	 		CAN_Config();	
	 		/* IT Configuration for CAN1 */  
  	 		CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);///CAN_IT_FMP0: FIFO0��Ϣ�Һ��ж�����
 	 		/* IT Configuration for CAN2 */  
  	 		CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
			user_statment=NONE ;
		}
		else if ( (user_statment==CAN_stop) )
		{
			CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
  	 		CAN_ITConfig(CAN2, CAN_IT_FMP0, DISABLE);
	 		SysTick_close();
			user_statment=NONE ;
		}
		else if ( (user_statment==CAN_set) )
		{
            CAN_Uesr_Config(&USB_Rx_Buffer[7]);
            user_statment=NONE ;
		}
//		else if ( (user_statment==CAN_rece) )
//		{
//			//ģ����յ���,���бȽ�
//			simu_CAN20();
//			user_statment=NONE ;
//		}


    
//	if (TxMessage.StdId == 0x7F0)
//	{
//	TxMessage.StdId = 0x7E0;
//    }
//	else
//	{
//	TxMessage.StdId ++;
//	}
//	TxMessage.ExtId = 0x1234;
//    TxMessage.RTR = CAN_RTR_DATA;
//    TxMessage.IDE = CAN_ID_STD;
//    TxMessage.DLC = 4;
// 	
//	for(i=0,j=0x30;i<8;i++,j++)
//	{
//	 TxMessage.Data[i] = j;
//	}
//	CAN_Transmit(CAN2, &TxMessage);
//	delay(1500000);


	}

}

/**-------------------------------------------------------
  * @������ SysTick_Handler_User
  * @����   ϵͳ���Ķ�ʱ�����������û�������
  * @����   ��
  * @����ֵ ��
***------------------------------------------------------*/
void SysTick_Handler_User(void)
{

	static uint32_t TimeIncrease = 0;

	if( !(TimeIncrease%100) )
	{
		if(!(TimeIncrease%2000))
		{
			// ÿ������100ms
			LED2OBB = 0;
		}
		else
		{
			LED2OBB = 1;
		}
    }
	TimeIncrease++;
}

/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
///#ifndef USE_STM3210C_EVAL
///  GPIO_InitTypeDef GPIO_InitStructure;
///#endif /* USE_STM3210C_EVAL */

  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();                                                             ///������RCC�Ĵ�������Ϊȱʡֵ

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);                                                ///�����ⲿ���پ���HSE��

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();                               ///�ȴ�HSE����

  if (HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);                   ///[ʹ�ܻ���ʧ��FLASH�����ڷ���]��[����FLASHԤ�����幦�ܣ�����FLASH�Ķ�ȡ]

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);                                      ///��������FLASH�洢����ʱʱ��������
                                                                            ///
    /* HCLK = SYSCLK */                                                     ///
    RCC_HCLKConfig(RCC_SYSCLK_Div1);                                        ///���� AHB ʱ�ӣ�HCLK��: RCC_SYSCLK_Div1 --> AHB ʱ�� =  ϵͳʱ��
                                                                            ///
    /* PCLK2 = HCLK */                                                      ///
    RCC_PCLK2Config(RCC_HCLK_Div1);                                         ///���ø��� AHB ʱ�ӣ�PCLK2��
                                                                            ///
    /* PCLK1 = HCLK/2 */                                                    ///
    RCC_PCLK1Config(RCC_HCLK_Div2);                                         ///

#ifdef STM32F10X_CL
    /* Configure PLLs *********************************************************/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */              ///
    RCC_PREDIV2Config(RCC_PREDIV2_Div5);                                    ///
    RCC_PLL2Config(RCC_PLL2Mul_8);                                          ///
                                                                            ///
    /* Enable PLL2 */                                                       ///
    RCC_PLL2Cmd(ENABLE);                                                    ///
                                                                            ///
    /* Wait till PLL2 is ready */                                           ///
    while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)                    ///
    {}                                                                      ///
                                                                            ///
    /* PLL configuration: PLLCLK = (PLL2 / 5) * 9 = 72 MHz */               ///
    RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);           ///
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);                     ///
#else                                                                       ///
    /* PLLCLK = 8MHz * 9 = 72 MHz */                                        ///
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);                    ///
#endif                                                                      ///
                                                                            ///
    /* Enable PLL */                                                        ///
    RCC_PLLCmd(ENABLE);                                                     ///
                                                                            ///
    /* Wait till PLL is ready */                                            ///
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)                     ///
    {                                                                       ///
    }                                                                       ///
                                                                            ///
    /* Select PLL as system clock source */                                 ///
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);                              ///
                                                                            ///
    /* Wait till PLL is used as system clock source */                      ///
    while(RCC_GetSYSCLKSource() != 0x08)                                    ///��������ϵͳʱ�ӵ�ʱ��Դ
    {                                                                       ///
    }                                                                       ///
  }                                                                         ///
  else
  { /* If HSE fails to start-up, the application will have wrong clock configuration.
       User can add here some code to deal with this error */

    /* Go to infinite loop */
    while (1)
    {
    }
  }

///#ifndef USE_STM3210C_EVAL
///  /* Enable USB_DISCONNECT GPIO clock */
///  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);
///
///  /* Configure USB pull-up pin */
///  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
///  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
///  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
///  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
///#endif /* USE_STM3210C_EVAL */
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
#ifdef STM32F10X_CL
  /* Select USBCLK source */
  RCC_OTGFSCLKConfig(RCC_OTGFSCLKSource_PLLVCO_Div3);

  /* Enable the USB clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_OTG_FS, ENABLE) ;
#else
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#endif /* STM32F10X_CL */
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Set the Vector Table base address at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00000);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

#ifdef STM32F10X_CL
  /* Enable the USB Interrupts */
  NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#else
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif /* STM32F10X_CL */

  /* Enable USART Interrupt */
///  NVIC_InitStructure.NVIC_IRQChannel = EVAL_COM1_IRQn;
///  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
}

void GPIOcan_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Configure CAN1 RX pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Configure CAN2 RX pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure CAN1 TX pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Configure CAN2 TX pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Remap CAN1 and CAN2 GPIOs */
  GPIO_PinRemapConfig(GPIO_Remap2_CAN1 , ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_CAN2, ENABLE);
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /* Enable CAN1 RX0 interrupt IRQ channel */
#ifndef STM32F10X_CL
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
#else
 NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;                        ///CAN�ж����ȼ�����
#endif /* STM32F10X_CL*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
  NVIC_Init(&NVIC_InitStructure);

}

void SysTick_close()
{
  SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_TICKINT_Pos   | 
                   SysTick_CTRL_ENABLE_Pos;                    /* disable SysTick IRQ and SysTick Timer */
};

void CAN_Config(void)
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  //uint16_t ih=0xffff,il=0xffff,jh=0xffff,jl=0xffff;

  /* CAN register init */
  CAN_DeInit(CAN1);
  CAN_DeInit(CAN2);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN1 cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;							///CAN_TTCM����ʹ�ܻ���ʧ��ʱ�䴥��ͨѶģʽ�������������������ֵΪENABLE����DISABLE��
  CAN_InitStructure.CAN_ABOM = DISABLE;							///CAN_ABOM����ʹ�ܻ���ʧ���Զ����߹��������������������ֵΪENABLE����DISABLE��
  CAN_InitStructure.CAN_AWUM = DISABLE;							///CAN_AWUM����ʹ�ܻ���ʧ���Զ�����ģʽ�������������������ֵΪENABLE����DISABLE��
  CAN_InitStructure.CAN_NART = DISABLE;							///CAN_NARM����ʹ�ܻ���ʧ�ܷ��Զ��ش���ģʽ�������������������ֵΪENABLE����DISABLE��
  CAN_InitStructure.CAN_RFLM = DISABLE;							///CAN_RFLM����ʹ�ܻ���ʧ�ܽ���FIFO����ģʽ�������������������ֵΪENABLE����DISABLE��
  CAN_InitStructure.CAN_TXFP = DISABLE;							///CAN_TXFP����ʹ�ܻ���ʧ�ܷ���FIFO���ȼ��������������������ֵΪENABLE����DISABLE��
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;					///CAN_Mode������CAN�Ĺ���ģʽ��
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;						///CAN_SJW����������ͬ����Ծ���(SJW)������ÿλ�п����ӳ������̶��ٸ�ʱ�䵥λ������
  CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;						///�����������ڼ��㲨���� bitrate=Fpclk/(BRP+1)*((Tseg1+1)+(Tseg2+2)+1)
  CAN_InitStructure.CAN_Prescaler = 4;							///CAN_Prescaler�趨��һ��ʱ�䵥λ�ĳ��ȣ����ķ�Χ��1��1024��	(��Ƶϵ��)
                                                                ///������USB������ʱ,�ѽ�PLL��Ϊϵͳʱ��,������ϵͳʱ��Ƶ��Ϊ72MHz,CAN������APB1��ʱƵ��Ϊ36MHz
                                                                ///500K=36000/[(8+9+1)*4]
  CAN_Init(CAN1, &CAN_InitStructure);
  CAN_Init(CAN2, &CAN_InitStructure);

  /* CAN1 filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 0;							 ///CAN_FilterNumberָ���˴���ʼ���Ĺ����������ķ�Χ��1��13��
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;		 ///CAN_FilterModeָ���˹�����������ʼ������ģʽ ��ʶ������λģʽ���ʶ���б�ģʽ
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;		 ///CAN_FilterScale�����˹�����λ��	2��16λ��������1��32λ������
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0 ;					 ///CAN_FilterIdHigh�����趨��������ʶ����32λλ��ʱΪ��߶�λ��16λλ��ʱΪ��һ����
  CAN_FilterInitStructure.CAN_FilterIdLow = 0 ;						 ///CAN_FilterIdHigh�����趨��������ʶ����32λλ��ʱΪ��Ͷ�λ��16λλ��ʱΪ�ڶ�������
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0;				 ///CAN_FilterMaskIdHigh�����趨���������α�ʶ�����߹�������ʶ����32λλ��ʱΪ��߶�λ��16λλ��ʱΪ��һ������
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0;					 ///CAN_FilterMaskIdLow�����趨���������α�ʶ�����߹�������ʶ����32λλ��ʱΪ��Ͷ�λ��16λλ��ʱΪ�ڶ�����
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;					 ///CAN_FilterFIFO�趨��ָ���������FIFO��0��1��
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;				 ///CAN_FilterActivationʹ�ܻ���ʧ�ܹ��������ò�����ȡ��ֵΪENABLE����DISABLE��
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  /* CAN2 filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 14;
  CAN_FilterInit(&CAN_FilterInitStructure);
}

void key_canconfig(void)
{
   Button_TypeDef key1=KEY1;
   Button_TypeDef key2=KEY2;
   ButtonMode_TypeDef keymode=BUTTON_MODE_GPIO;
   SZ_STM32_KEYInit(key1,keymode);
   SZ_STM32_KEYInit(key2,keymode);

};

uint8_t CAN_Uesr_Config(uint8_t *pbuf)							   ///SETCAN1
{

    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	CAN_TypeDef  *CanType ;

    ///��ʼ���ṹ��
    CAN_StructInit(&CAN_InitStructure);
    CAN_InitStructure.CAN_TTCM = DISABLE;							///CAN_TTCM����ʹ�ܻ���ʧ��ʱ�䴥��ͨѶģʽ�������������������ֵΪENABLE����DISABLE��
    CAN_InitStructure.CAN_ABOM = DISABLE;							///CAN_ABOM����ʹ�ܻ���ʧ���Զ����߹��������������������ֵΪENABLE����DISABLE��
    CAN_InitStructure.CAN_AWUM = DISABLE;							///CAN_AWUM����ʹ�ܻ���ʧ���Զ�����ģʽ�������������������ֵΪENABLE����DISABLE��
    CAN_InitStructure.CAN_NART = DISABLE;							///CAN_NARM����ʹ�ܻ���ʧ�ܷ��Զ��ش���ģʽ�������������������ֵΪENABLE����DISABLE��
    CAN_InitStructure.CAN_RFLM = DISABLE;							///CAN_RFLM����ʹ�ܻ���ʧ�ܽ���FIFO����ģʽ�������������������ֵΪENABLE����DISABLE��
    CAN_InitStructure.CAN_TXFP = DISABLE;							///CAN_TXFP����ʹ�ܻ���ʧ�ܷ���FIFO���ȼ��������������������ֵΪENABLE����DISABLE��
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;					///CAN_Mode������CAN�Ĺ���ģʽ��
    ///�ݶ�������Ϊ500k,֮���ټӲ����޸�
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;						///CAN_SJW����������ͬ����Ծ���(SJW)������ÿλ�п����ӳ������̶��ٸ�ʱ�䵥λ������
    CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;						///�����������ڼ��㲨���� bitrate=Fpclk/(BRP+1)*((Tseg1+1)+(Tseg2+2)+1)
    CAN_InitStructure.CAN_Prescaler = 4;							///CAN_Prescaler�趨��һ��ʱ�䵥λ�ĳ��ȣ����ķ�Χ��1��1024��	(��Ƶϵ��)
    
//    if (*pbuf=='1')
//	{
//		CAN_Init(CAN1, &CAN_InitStructure);
//	}
//	else
//	{
//	 	CAN_Init(CAN2, &CAN_InitStructure);
//	}
		
	CanType= (*pbuf=='1')? CAN1 : CAN2 ;
	CAN_Init(CanType, &CAN_InitStructure);

  CAN_FilterInitStructure.CAN_FilterNumber = 0;							 ///CAN_FilterNumberָ���˴���ʼ���Ĺ����������ķ�Χ��1��13��
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;		 ///CAN_FilterModeָ���˹�����������ʼ������ģʽ ��ʶ������λģʽ���ʶ���б�ģʽ
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;		 ///CAN_FilterScale�����˹�����λ��	2��16λ��������1��32λ������
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000 ;					 ///CAN_FilterIdHigh�����趨��������ʶ����32λλ��ʱΪ��߶�λ��16λλ��ʱΪ��һ����
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000 ;						 ///CAN_FilterIdHigh�����趨��������ʶ����32λλ��ʱΪ��Ͷ�λ��16λλ��ʱΪ�ڶ�������
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;				 ///CAN_FilterMaskIdHigh�����趨���������α�ʶ�����߹�������ʶ����32λλ��ʱΪ��߶�λ��16λλ��ʱΪ��һ������
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;					 ///CAN_FilterMaskIdLow�����趨���������α�ʶ�����߹�������ʶ����32λλ��ʱΪ��Ͷ�λ��16λλ��ʱΪ�ڶ�����
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;					 ///CAN_FilterFIFO�趨��ָ���������FIFO��0��1��
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;				 ///CAN_FilterActivationʹ�ܻ���ʧ�ܹ��������ò�����ȡ��ֵΪENABLE����DISABLE��
  CAN_FilterInit(&CAN_FilterInitStructure);
	
  return 0 ;

};

/******************* (C) COPYRIGHT 2013 www.armjishu.com *****END OF FILE****/
















