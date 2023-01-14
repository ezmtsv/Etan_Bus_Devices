#ifndef _SMOKE_H_
#define _SMOKE_H_

#include "system.h"
#include "platform.h"
#include "stm32.h"
#include <string.h>

//РУс
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

#define PIN_ADR0                                     0
#define PIN_ADR1                                     1
#define PIN_ADR2                                     2
#define PIN_ONOFF                                    7

#define GPIO_Pin_0                                   ((uint16_t)0x0001U)  /* Pin 0 selected    */
#define GPIO_Pin_1                                   ((uint16_t)0x0002U)  /* Pin 1 selected    */
#define GPIO_Pin_2                                   ((uint16_t)0x0004U)  /* Pin 2 selected    */
#define GPIO_Pin_7                                   ((uint16_t)0x0080U)  /* Pin 7 selected    */

#define Sens_ON	                	                   (GPIOA->BSRR |= GPIO_Pin_7)
#define Sens_OFF	                                   (GPIOA->BRR |= GPIO_Pin_7)

uint16_t work_ADC_STM(void);
void init_smoke(void);
uint8_t status_sens(void);
void set_ADR0(void);
void set_ADR1(void);
void set_ADR2(void);
void set_ADR3(void);
void set_ADR4(void);
void set_ADR5(void);
void set_ADR6(void);
void set_ADR7(void);
void delay_ms(uint32_t us);

#endif

