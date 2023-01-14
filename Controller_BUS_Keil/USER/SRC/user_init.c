#include "user_init.h"

#define ADC_CH 4
#define ADC1_DR_Address    0x40012440

extern uint16_t ADC_config_word;
extern uint16_t duty_PWM;

uint16_t data_ADC_forDMA[4];



void init_PORTA (void) {
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC -> AHBENR |= PORTA_CLK;
  GPIO_InitStructure.GPIO_Pin = LED_R | EN_DRV;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init (PORTA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = ADC_MC | STATUS_L_PLUS | STATUS_L_MINUS | line_VOL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init (PORTA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = DATA_IN | FLAG_EX_POW;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init (PORTA, &GPIO_InitStructure);
  DATA_OUT_LOW;
}



void init_PORTB (void) {
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC -> AHBENR |= RCC_AHBPeriph_GPIOB;
  GPIO_InitStructure.GPIO_Pin = SEL_RANGE;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init (GPIOB, &GPIO_InitStructure);
}



void led_one_ON (uint16_t led_) {
  PORTA-> BSRR = led_;
}



void led_one_OFF (uint16_t led_) {
  PORTA-> BRR = led_;
}



void init_tmr3 (void) {
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC -> AHBENR |= PORTA_CLK;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(PORTA, &GPIO_InitStructure);
  GPIO_PinAFConfig (GPIOA, GPIO_PinSource6, GPIO_AF_1);

  RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3, ENABLE);
  TIM3->PSC = (48*DIV_base)-1;
  TIM3->ARR = 8;
  TIM3->CCR1 = 9 - 6;
  TIM3->CCER |= TIM_CCER_CC1E;
  TIM3->BDTR |= TIM_BDTR_MOE;
  TIM3->CCMR1|=(TIM_CCMR1_OC1M_0| TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
  TIM3->CR1 &= ~TIM_CR1_DIR;
  TIM3->CR1 |= TIM_CR1_OPM;
  TIM3->CR1 |= TIM_CR1_CEN;
}



void init_tmr16 (void) {
  TIM_TimeBaseInitTypeDef timer;
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_TIM16, ENABLE);
  TIM_TimeBaseStructInit (&timer);
  timer.TIM_Prescaler = 48000;
	timer.TIM_Period = 100;
  TIM_TimeBaseInit (TIM16, &timer);
  TIM_ITConfig (TIM16, TIM_IT_Update, ENABLE);
  NVIC_SetPriority (TIM16_IRQn, 1);
  NVIC_EnableIRQ (TIM16_IRQn);
  TIM_Cmd (TIM16, ENABLE);
}



void init_tmr17 (void) {
  TIM_TimeBaseInitTypeDef timer;
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_TIM17, ENABLE);
  TIM_TimeBaseStructInit (&timer);
  timer.TIM_Prescaler = 47;
  timer.TIM_Period = DMSAMPLE+2;
  TIM_TimeBaseInit (TIM17, &timer);
  TIM_ITConfig (TIM17, TIM_IT_Update, ENABLE);
  NVIC_SetPriority (TIM17_IRQn, 0);
  NVIC_EnableIRQ(TIM17_IRQn);
}



void init_GPIO_ineterrupt (void) {
  RCC->APB2RSTR = 0;
  RCC->APB2ENR |= RCC_APB2Periph_SYSCFG;
  SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;
  SYSCFG->EXTICR[1] = SYSCFG_EXTICR2_EXTI5_PA | SYSCFG_EXTICR2_EXTI7_PA;
  EXTI-> IMR = EXTI_IMR_MR5 | EXTI_IMR_MR7;
  EXTI ->RTSR = EXTI_RTSR_TR7 | EXTI_RTSR_TR5;
  EXTI ->FTSR = EXTI_RTSR_TR7 | EXTI_RTSR_TR5;
  NVIC_SetPriority (EXTI4_15_IRQn, 0);
  NVIC_EnableIRQ (EXTI4_15_IRQn);
}



void TogglePin (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  GPIOx->ODR ^= GPIO_Pin;
}



void init_gpio_ (void) {
  init_PORTA();
  init_PORTB();
  init_GPIO_ineterrupt();
  EN_DRV_OFF;
  SEL_RANGE_OFF;
}



void init_ADC_DMA (void) {
  ADC_InitTypeDef stuct_ADC_STM;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  stuct_ADC_STM.ADC_ContinuousConvMode = ENABLE;
  stuct_ADC_STM.ADC_Resolution = ADC_Resolution_12b;
  stuct_ADC_STM.ADC_DataAlign = ADC_DataAlign_Right;
  stuct_ADC_STM.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  stuct_ADC_STM.ADC_ScanDirection = ADC_ScanDirection_Upward;
  stuct_ADC_STM.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
  ADC_Init(ADC1, &stuct_ADC_STM);

  ADC_ChannelConfig(ADC1, ADC_CHSELR_CHSEL0, ADC_SampleTime_239_5Cycles);
  ADC_ChannelConfig(ADC1, ADC_CHSELR_CHSEL1, ADC_SampleTime_239_5Cycles);
  ADC_ChannelConfig(ADC1, ADC_CHSELR_CHSEL2, ADC_SampleTime_239_5Cycles);
  ADC_ChannelConfig(ADC1, ADC_CHSELR_CHSEL3, ADC_SampleTime_239_5Cycles);
  ADC_GetCalibrationFactor(ADC1);
  ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
  ADC_DMACmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));
  ADC_StartOfConversion(ADC1);
}



void DMA_Config (void) {
  DMA_InitTypeDef   DMA_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)data_ADC_forDMA;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  // DMA1 Channel1 enable //
  DMA_Cmd(DMA1_Channel1, ENABLE);
}
