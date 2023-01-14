#ifndef _GAS_SENS_H_
#define _GAS_SENS_H_

#include "system.h"
#include "platform.h"
#include "stm32.h"
#include "prog_I2C.h"

#define PIN_PWM                                      7
#define PIN_ADC                                      6
#define ADC_FLAG_EOC                                 ((uint32_t)0x00000004)        /*!< End of Conversion */
#define RCC_APB2Periph_ADC1                          ((uint32_t)0x00000200)        /*!< ADC1 clock enable */
#define ADC_SampleTime_1_5Cycles                     ((uint32_t)0x00000000)
#define ADC_SampleTime_7_5Cycles                     ((uint32_t)0x00000001)
#define ADC_SampleTime_13_5Cycles                    ((uint32_t)0x00000002)
#define ADC_SampleTime_28_5Cycles                    ((uint32_t)0x00000003)
#define ADC_SampleTime_41_5Cycles                    ((uint32_t)0x00000004)
#define ADC_SampleTime_55_5Cycles                    ((uint32_t)0x00000005)
#define ADC_SampleTime_71_5Cycles                    ((uint32_t)0x00000006)
#define ADC_SampleTime_239_5Cycles                   ((uint32_t)0x00000007)


void init_gas_sens (void);
uint16_t work_ADC_STM (void);

#endif
