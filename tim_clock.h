
#include "SZ_STM32F107VC_LIB.h"

extern void timeOut_Function();

static void NVIC_TIM5Configuration(void);
void TIM5_Init(void);
void TIM5_IRQHandler(void);
