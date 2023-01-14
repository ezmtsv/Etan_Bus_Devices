#include "func.h"

uint16_t data_ADC_cur_line = 0;
uint16_t data_ADC_line_HI = 0;
uint16_t data_ADC_line_LOW = 0;
uint16_t data_ADC_line_VOL = 0;
uint16_t buffer_ADC[200];



uint16_t run_ADCSTM (void) {
  uint16_t result_ADC;
  ADC_StartOfConversion (ADC1);
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  result_ADC = ADC_GetConversionValue (ADC1);
  return result_ADC;
}



uint16_t filter_median (uint16_t *buf, uint16_t len) {
	uint16_t count;
	uint16_t tmp_count;
	uint16_t maximum;
	uint16_t rezult;

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



void work_ADC_STM (void) {
  int count;

  for (count = 0; count < 100; count++) {
    buffer_ADC[count] = run_ADCSTM();
//    Delay(300);
  }
  data_ADC_cur_line = filter_median (buffer_ADC, 100);
  ADC1->CHSELR = ADC_CHSELR_CHSEL1;
  for (count = 0; count < 100; count++) {
    buffer_ADC[count] = run_ADCSTM();
//    Delay(300);
  }
  data_ADC_line_HI = filter_median (buffer_ADC, 100);

  ADC1->CHSELR = ADC_CHSELR_CHSEL2;
  for (count = 0; count < 100; count++) {
    buffer_ADC[count] = run_ADCSTM();
//    Delay(300);
  }
  data_ADC_line_LOW = filter_median (buffer_ADC, 100);

  ADC1->CHSELR = ADC_CHSELR_CHSEL3;
  for (count = 0; count < 100; count++) {
    buffer_ADC[count] = run_ADCSTM();
//    Delay(300);
  }
  data_ADC_line_VOL = filter_median(buffer_ADC, 100);

  ADC1->CHSELR = ADC_CHSELR_CHSEL0;
  memset(buffer_ADC,0, sizeof(buffer_ADC));
}
