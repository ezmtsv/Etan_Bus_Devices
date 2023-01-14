#ifndef _DEV_DOOR_H_
#define _DEV_DOOR_H_

#include "system.h"
#include "platform.h"
#include "stm32.h"
#include <string.h>
#include <stdbool.h>

#define pin_VACC                                     0
#define pin_EN                                       1
#define pin_CHARGE                                   2
#define pin_OPEN                                     3
#define pin_ACC_GOOD                                 4
#define pin_BUTTON                                   5
#define pin_BUZ                                      6

#define pin_EN_ON                                    (GPIOA->BSRR|= (1<<pin_EN))
#define pin_EN_OFF                                   (GPIOA->BRR|= (1<<pin_EN))
#define pin_OPEN_ON                                  (GPIOA->BSRR|= (1<<pin_OPEN))
#define pin_OPEN_OFF                                 (GPIOA->BRR|= (1<<pin_OPEN))
#define pin_CHARGE_ON                                (GPIOA->BRR|= (1<<pin_CHARGE))
#define pin_CHARGE_OFF                               (GPIOA->BSRR|= (1<<pin_CHARGE))

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

#define nota_C 1046.5*2
#define nota_C_dies 1108.7*2
#define nota_D 1174.6*2
#define nota_D_dies 1244.5*2
#define nota_E 1318.5*2
#define nota_F 1396.9*2
#define nota_F_dies 1480.0*2
#define nota_G 1568.0*2
#define nota_G_dies 1661.2*2
#define nota_A 1720.0*2
#define nota_A_dies 1864.6*2
#define nota_B 1975.5*2

typedef enum {
  okt_0 = 16,
  okt_1 = 8,
  okt_2 = 4,
  okt_3 = 2,
  okt_4 = 1,
} range_okt;

uint16_t work_ADC_STM (void);
void music_play (range_okt okt);
void init_door_sens (void);

#endif
