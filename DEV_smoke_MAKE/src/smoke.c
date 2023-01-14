#include "smoke.h"
//РУс

uint16_t buffer_ADC[200];

FlagStatus ADC_GetFlagStatus (ADC_TypeDef* ADCx, uint32_t ADC_FLAG) {
  FlagStatus bitstatus = RESET;
  uint32_t tmpreg = 0;
  if ((uint32_t)(ADC_FLAG & 0x01000000)) {
    tmpreg = ADCx->CR & 0xFEFFFFFF;
  }
  else {
    tmpreg = ADCx->ISR;
  }
  /* Check the status of the specified ADC flag */
  if ((tmpreg & ADC_FLAG) != (uint32_t)RESET) {
    /* ADC_FLAG is set */
    bitstatus = SET;
  }
  else {
    /* ADC_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the ADC_FLAG status */
  return  bitstatus;
}

uint32_t ADC_GetCalibrationFactor(ADC_TypeDef* ADCx)
{
	#define CALIBRATION_TIMEOUT       ((uint32_t)0x0000F000)
  uint32_t tmpreg = 0, calibrationcounter = 0, calibrationstatus = 0;
  /* Set the ADC calibartion */
  ADCx->CR |= (uint32_t)ADC_CR_ADCAL;
  /* Wait until no ADC calibration is completed */
  do
  {
    calibrationstatus = ADCx->CR & ADC_CR_ADCAL;
    calibrationcounter++;
  } while((calibrationcounter != CALIBRATION_TIMEOUT) && (calibrationstatus != 0x00));

  if((uint32_t)(ADCx->CR & ADC_CR_ADCAL) == RESET)
  {
    /*Get the calibration factor from the ADC data register */
    tmpreg = ADCx->DR;
  }
  else
  {
    /* Error factor */
    tmpreg = 0x00000000;
  }
  return tmpreg;
}

uint16_t run_ADCSTM(void){
  uint16_t result_ADC;
	ADC1->CR |= (uint32_t)ADC_CR_ADSTART;
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	result_ADC = (uint16_t) ADC1->DR;
	return result_ADC;
}

//фильтр медиана (кол-во элементов массива должно быть четное)
uint16_t filter_median(uint16_t *buf, uint16_t len) {
  uint16_t count;
  uint16_t tmp_count;
  uint16_t maximum;
  uint16_t rezult;
//располагаем массив данных  по возрастанию
  rezult = 0;
	for(tmp_count = (len-1); tmp_count>0; tmp_count--) {
		for(count = 0; count<tmp_count; count++){
			if(buf[count]>buf[count+1]){
				maximum = buf[count];
				buf[count] = buf[count+1];
				buf[count+1] = maximum;
			};

		}

	}
	rezult = buf[len/2];
	return rezult;
}

uint16_t work_ADC_STM(void){
  uint16_t value = 0;
  for(uint8_t count = 0; count<100; count++){
    buffer_ADC[count] = run_ADCSTM();
    delay_us(300);
  }
  value = filter_median(buffer_ADC, 100);
  memset(buffer_ADC,0, sizeof(buffer_ADC));
  return value;
}

static void init_PIN(void) {
		#define GPIO_OUT(pin)   (0x01U << ((pin) << 1))
    _BST(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
    GPIOA->MODER |= GPIO_OUT(PIN_ADR0) | GPIO_OUT(PIN_ADR1) |
		                GPIO_OUT(PIN_ADR2) | GPIO_OUT(PIN_ONOFF);
}

static void init_ADC1(void) {
	uint32_t tmpreg = 0;
	#define CFGR1_CLEAR_MASK           ((uint32_t)0xFFFFD203)
	#define ADC_Resolution_12b         ((uint32_t)0x00000000)
	#define ADC_DataAlign_Right        ((uint32_t)0x00000000)
	#define ADC_DataAlign_Left         ADC_CFGR1_ALIGN
	RCC->APB2ENR |= RCC_APB2Periph_ADC1;
//////////////
  tmpreg = ADC1->CFGR1;
  /* Clear SCANDIR, RES[1:0], ALIGN, EXTSEL[2:0], EXTEN[1:0] and CONT bits */
  tmpreg &= CFGR1_CLEAR_MASK;
	tmpreg  |= (uint32_t)(ADC_Resolution_12b | ADC_DataAlign_Right |  ADC_DataAlign_Right);
  /* Write to ADCx CFGR */
  ADC1->CFGR1 = tmpreg;
/////////////
	  /* Configure the ADC Channel */
  ADC1->CHSELR |= (uint32_t)ADC_CHSELR_CHSEL3;
  /* Clear the Sampling time Selection bits */
  tmpreg &= ~ADC_SMPR1_SMPR;
  /* Set the ADC Sampling Time register */
  tmpreg |= (uint32_t)ADC_SampleTime_239_5Cycles;
  /* Configure the ADC Sample time register */
  ADC1->SMPR = tmpreg ;
	ADC_GetCalibrationFactor(ADC1);
	ADC1->CR |= (uint32_t)ADC_CR_ADEN;
}

void init_smoke(void) {
		init_PIN();
		init_ADC1();
}

uint8_t status_sens(void) {
	uint8_t b = 0;
	uint16_t alarm_sens = 1000;
	uint16_t delay_msec = 150;
	set_ADR0(); delay_ms(5);
	if(work_ADC_STM()> alarm_sens){ b = 1; for(uint16_t i = 0; i <2; i++ ){ tooglePIN_G(); delay_ms(delay_msec); } delay_ms(1000); };
	set_ADR1(); delay_ms(5);
	if(work_ADC_STM()> alarm_sens){ b |= 2; for(uint16_t i = 0; i <4; i++ ){ tooglePIN_G(); delay_ms(delay_msec); } delay_ms(1000); };
	set_ADR2(); delay_ms(5);
	if(work_ADC_STM()> alarm_sens){ b |= 4; for(uint16_t i = 0; i <6; i++ ){ tooglePIN_G(); delay_ms(delay_msec); } delay_ms(1000); };
	set_ADR3(); delay_ms(5);
	if(work_ADC_STM()> alarm_sens){ b |= 8; for(uint16_t i = 0; i <8; i++ ){ tooglePIN_G(); delay_ms(delay_msec); } delay_ms(1000); };
	set_ADR4(); delay_ms(5);
	if(work_ADC_STM()> alarm_sens){ b |= 16; for(uint16_t i = 0; i <10; i++ ){ tooglePIN_G(); delay_ms(delay_msec); } delay_ms(1000); };
	set_ADR5(); delay_ms(5);
	if(work_ADC_STM()> alarm_sens){ b |= 32; for(uint16_t i = 0; i <12; i++ ){ tooglePIN_G(); delay_ms(delay_msec); } delay_ms(1000);};
	set_ADR6(); delay_ms(5);
	if(work_ADC_STM()> alarm_sens){ b |= 64; for(uint16_t i = 0; i <14; i++ ){ tooglePIN_G(); delay_ms(delay_msec); } delay_ms(1000); };
	set_ADR7(); delay_ms(5);
	if(work_ADC_STM()> alarm_sens){ b |= 128; for(uint16_t i = 0; i <16; i++ ){ tooglePIN_G(); delay_ms(delay_msec); } delay_ms(1000); };
	return b;
}

void set_ADR0(void) {
	GPIOA->BRR |= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
}

void set_ADR1(void) {
	GPIOA->BSRR |= GPIO_Pin_0;
	GPIOA->BRR |= GPIO_Pin_1 | GPIO_Pin_2;
}

void set_ADR2(void) {
	GPIOA->BSRR |= GPIO_Pin_1;
	GPIOA->BRR |= GPIO_Pin_0 | GPIO_Pin_2;
}

void set_ADR3(void) {
	GPIOA->BSRR |= GPIO_Pin_0 | GPIO_Pin_1;
	GPIOA->BRR |= GPIO_Pin_2;
}

void set_ADR4(void) {
	GPIOA->BSRR |= GPIO_Pin_2;
	GPIOA->BRR |= GPIO_Pin_0 | GPIO_Pin_1;
}

void set_ADR5(void) {
	GPIOA->BSRR |= GPIO_Pin_2 | GPIO_Pin_0;
	GPIOA->BRR |= GPIO_Pin_1;
}

void set_ADR6(void) {
	GPIOA->BSRR |= GPIO_Pin_2 | GPIO_Pin_1;
	GPIOA->BRR |= GPIO_Pin_0;
}

void set_ADR7(void) {
	GPIOA->BSRR |= GPIO_Pin_2 | GPIO_Pin_1 | GPIO_Pin_0;
}
