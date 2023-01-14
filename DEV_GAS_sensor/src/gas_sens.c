#include "gas_sens.h"



static void init_PWM_tmr1 (void) {
  RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA->MODER |= GPIO_AFN(PIN_PWM);
  GPIOA->AFR[0] |= GPIO_AF(PIN_PWM , 2);

  RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;
  TIM1->PSC = 0;
  TIM1->ARR = 480;              // Fsc PWM 100kHz
  TIM1->CCR1 = 240;
  TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;
  TIM1->SMCR &= ~ TIM_SMCR_SMS;
  TIM1->BDTR |= TIM_BDTR_MOE;
  TIM1->CCMR1|=(TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
  TIM1->CR1 &= ~TIM_CR1_DIR;
  TIM1->CR1 |= TIM_CR1_CEN;
}



FlagStatus ADC_GetFlagStatus (ADC_TypeDef* ADCx, uint32_t ADC_FLAG) {
  FlagStatus bitstatus = RESET;
  uint32_t tmpreg = 0;
  if ((uint32_t)(ADC_FLAG & 0x01000000)) {
    tmpreg = ADCx->CR & 0xFEFFFFFF;
  }
  else {
    tmpreg = ADCx->ISR;
  }
  if ((tmpreg & ADC_FLAG) != (uint32_t)RESET) {
    bitstatus = SET;
  }
  else {
    bitstatus = RESET;
  }
  return  bitstatus;
}



uint32_t ADC_GetCalibrationFactor(ADC_TypeDef* ADCx)
{
  #define CALIBRATION_TIMEOUT       ((uint32_t)0x0000F000)
  uint32_t tmpreg = 0, calibrationcounter = 0, calibrationstatus = 0;
  ADCx->CR |= (uint32_t)ADC_CR_ADCAL;

  do {
    calibrationstatus = ADCx->CR & ADC_CR_ADCAL;
    calibrationcounter++;
  } while ((calibrationcounter != CALIBRATION_TIMEOUT) && (calibrationstatus != 0x00));
  if ((uint32_t)(ADCx->CR & ADC_CR_ADCAL) == RESET) {
    tmpreg = ADCx->DR;
  } else {
    tmpreg = 0x00000000;
  }
  return tmpreg;
}



uint16_t run_ADCSTM (void) {
  uint16_t result_ADC;
  ADC1->CR |= (uint32_t)ADC_CR_ADSTART;
  while (ADC_GetFlagStatus (ADC1, ADC_FLAG_EOC) == RESET);
  result_ADC = (uint16_t) ADC1->DR;
  return result_ADC;
}



uint16_t filter_median (uint16_t *buf, uint16_t len) {
  uint16_t count;
  uint16_t tmp_count;
  uint16_t maximum;
  uint16_t rezult = 0;

  for (tmp_count = (len-1); tmp_count>0; tmp_count--) {
    for (count = 0; count<tmp_count; count++) {
      if (buf[count]>buf[count+1]) {
        maximum = buf[count];
        buf[count] = buf[count+1];
        buf[count+1] = maximum;
      };
    }
  }
  rezult = buf[len/2];
  return rezult;
}



uint16_t work_ADC_STM (void) {
  uint16_t buffer_ADC[200];
  uint16_t value = 0;
  for (uint8_t count = 0; count<100; count++) {
    buffer_ADC[count] = run_ADCSTM();
    delay_us (300);
  }
  value = filter_median (buffer_ADC, 100);
  return value;
}



static void init_ADC1 (void) {
  #define CFGR1_CLEAR_MASK           ((uint32_t)0xFFFFD203)
  #define ADC_Resolution_12b         ((uint32_t)0x00000000)
  #define ADC_DataAlign_Right        ((uint32_t)0x00000000)
  #define ADC_DataAlign_Left         ADC_CFGR1_ALIGN
  uint32_t tmpreg = 0;

  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  tmpreg = ADC1->CFGR1;
  tmpreg &= CFGR1_CLEAR_MASK;
  tmpreg  |= (uint32_t)(ADC_Resolution_12b | ADC_DataAlign_Right |  ADC_DataAlign_Right);
  ADC1->CFGR1 = tmpreg;
  ADC1->CHSELR |= (uint32_t)ADC_CHSELR_CHSEL6;
  tmpreg &= ~ADC_SMPR1_SMPR;
  tmpreg |= (uint32_t)ADC_SampleTime_239_5Cycles;
  ADC1->SMPR = tmpreg ;
  ADC_GetCalibrationFactor(ADC1);
  ADC1->CR |= (uint32_t)ADC_CR_ADEN;
}



void init_gas_sens (void) {
  init_i2c();
  write_byte_DEV (adr_MCP47A1, 0, 32);
  init_PWM_tmr1();
  init_ADC1();
}
