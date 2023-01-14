#include "dev_door.h"

static void init_PIN_door (void) {
  // Config PIN
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA->MODER |= GPIO_OUT (pin_CHARGE) | GPIO_OUT (pin_OPEN) | GPIO_OUT (pin_EN);
  GPIOA->MODER |= GPIO_INP (pin_BUTTON) | GPIO_INP (pin_ACC_GOOD);
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



uint32_t ADC_GetCalibrationFactor (ADC_TypeDef* ADCx) {
  #define CALIBRATION_TIMEOUT       ((uint32_t)0x0000F000)
  uint32_t tmpreg = 0, calibrationcounter = 0, calibrationstatus = 0;
  ADCx->CR |= (uint32_t)ADC_CR_ADCAL;
  do {
    calibrationstatus = ADCx->CR & ADC_CR_ADCAL;
    calibrationcounter++;
  } while ((calibrationcounter != CALIBRATION_TIMEOUT) && (calibrationstatus != 0x00));
  if ((uint32_t)(ADCx->CR & ADC_CR_ADCAL) == RESET) {
    tmpreg = ADCx->DR;
  }
  else {
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
  uint16_t rezult;
//располагаем массив данных  по возрастанию
  rezult = 0;
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
  tmpreg |= (uint32_t)(ADC_Resolution_12b | ADC_DataAlign_Right |  ADC_DataAlign_Right);
  ADC1->CFGR1 = tmpreg;
  ADC1->CHSELR |= (uint32_t)ADC_CHSELR_CHSEL0;
  tmpreg &= ~ADC_SMPR1_SMPR;
  tmpreg |= (uint32_t)ADC_SampleTime_239_5Cycles;
  ADC1->SMPR = tmpreg ;
  ADC_GetCalibrationFactor (ADC1);
  ADC1->CR |= (uint32_t)ADC_CR_ADEN;
}



static void init_PWM (void) {
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA->MODER |= GPIO_AFN (pin_BUZ);
  GPIOA->AFR[0] |= GPIO_AF (pin_BUZ, 5);
  RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
  TIM16->PSC = 7;
  TIM16->ARR = 3000;
  TIM16->CCR1 = 1;
  TIM16->CCER |= TIM_CCER_CC1E;
  TIM16->BDTR |= TIM_BDTR_MOE;
  TIM16->CCMR1|=(TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
  TIM16->CR1 &= ~TIM_CR1_DIR;
  TIM16->CR1 |= TIM_CR1_CEN;
  TIM16->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
}



void change_cyclePWM (double freq) {
  double freq_ = freq*2;
  double fr = (2000.0*3000.0)/freq_;
  TIM16->ARR = (uint16_t)fr;
  TIM16->CCR1 = (uint16_t)(fr/2);
}



static void DelaymS (uint16_t time) {
  delay_us (1000*time);
}



void music_play (range_okt okt) {  //// подмосковные вечера, кусок песни, начало
  uint16_t delay_0_125 = 350;
  uint16_t delay_0_25 = 700;
  uint16_t delay_0_5 = 1400;

  TIM16->CR1 |= TIM_CR1_CEN;
  change_cyclePWM (nota_C/okt);
  DelaymS (delay_0_125);
  change_cyclePWM (nota_D_dies/okt);
  DelaymS (delay_0_125);
  change_cyclePWM (nota_G/okt);
  DelaymS (delay_0_125);
  change_cyclePWM (nota_D_dies/okt);
  DelaymS (delay_0_125);
  change_cyclePWM (nota_F/okt);
  DelaymS (delay_0_25);
  change_cyclePWM (nota_D_dies/okt);
  DelaymS (delay_0_125);
  change_cyclePWM (nota_D/okt);
  DelaymS (delay_0_125);
  change_cyclePWM (nota_G/okt);
  DelaymS (delay_0_25);
  change_cyclePWM (nota_F/okt);
  DelaymS (delay_0_25);
  change_cyclePWM (nota_C/okt);
  DelaymS (delay_0_5);

  change_cyclePWM (nota_D_dies/okt);
  DelaymS (delay_0_125);
  change_cyclePWM (nota_G/okt);
  DelaymS (delay_0_125);
  change_cyclePWM (nota_A_dies/okt);
  DelaymS (delay_0_125);
  TIM16->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
  DelaymS (10);
  TIM16->CR1 |= TIM_CR1_CEN;
  change_cyclePWM (nota_A_dies/okt);
  DelaymS (delay_0_125);
  change_cyclePWM ((nota_C/okt)*2);
  DelaymS (delay_0_25);
  change_cyclePWM (nota_A_dies/okt);
  DelaymS (delay_0_125);
  change_cyclePWM (nota_G_dies/okt);
  DelaymS (delay_0_125);
  change_cyclePWM (nota_G/okt);
  DelaymS (delay_0_5);

  TIM16->ARR = 3000;
  TIM16->CCR1 = 1;
  TIM16->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
}



void init_door_sens (void) {
  init_PIN_door();
  init_ADC1();
  pin_OPEN_OFF;
  pin_CHARGE_OFF;
  init_PWM();
  DelaymS (50);
  pin_EN_ON;
}
