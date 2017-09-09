

#include "tim_clock.h"

/**-------------------------------------------------------
  * @函数名 NVIC_TIM5Configuration
  * @功能   配置TIM5中断向量参数函数
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
static void NVIC_TIM5Configuration(void)
{ 
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Set the Vector Table base address at 0x08000000 */
    //NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);

    /* Enable the TIM5 gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}


/**-------------------------------------------------------
  * @函数名 NVIC_TIM5Configuration
  * @功能   配置TIM5参数函数，每秒计数器中断一次 
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void TIM5_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    /* TIM5 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    /* ---------------------------------------------------------------
    TIM4 Configuration: Output Compare Timing Mode:
    TIM2CLK = 36 MHz, Prescaler = 7200, TIM2 counter clock = 7.2 MHz
    --------------------------------------------------------------- */

    /* Time base configuration */
    //这个就是自动装载的计数值，由于计数是从0开始的，计数10000次后为9999
    TIM_TimeBaseStructure.TIM_Period = (10000 - 1);
    // 这个就是预分频系数，当由于为0时表示不分频所以要减1
    TIM_TimeBaseStructure.TIM_Prescaler = (7200 - 1);
    // 高级应用本次不涉及。定义在定时器时钟(CK_INT)频率与数字滤波器(ETR,TIx)
    // 使用的采样频率之间的分频比例
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    //向上计数
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    //初始化定时器5
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    /* Clear TIM5 update pending flag[清除TIM5溢出中断标志] */
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

    /* TIM IT enable */ //打开溢出中断
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    /* TIM5 enable counter */
    TIM_Cmd(TIM5, ENABLE);  //计数器使能，开始工作

    /* 中断参数配置 */
    NVIC_TIM5Configuration();
}


/**-------------------------------------------------------
  * @函数名 TIM5_IRQHandler
  * @功能   TIM5中断处理函数，每秒中断一次 
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void TIM5_IRQHandler(void)
{
    /* www.armjishu.com ARM技术论坛 */
    static u32 counter = 0;

    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        /* LED1指示灯状态取反 */
        //SZ_STM32_LEDToggle(LED3);

		// 熄灭LED3
		LED3OBB = 1;
		
		/* can中断超时,复位计数变量 */
		timeOut_Function();
    }
}
