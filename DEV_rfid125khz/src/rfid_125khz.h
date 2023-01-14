#ifndef _RFID_125KHZ_
#define _RFID_125KHZ_

#include "system.h"
#include "platform.h"
#include "stm32.h"
#include <string.h>
#include <stdbool.h>
//РУс
#define GPIO_Pin_5              ((uint16_t)0x0020) 
#define GPIO_Pin_6              ((uint16_t)0x0040)  

#define IN_RFID                 5
#define CLK_125K                6
#define countTIM14              (TIM14->CNT)
#define stat_IN_data            ((GPIOA->IDR & GPIO_Pin_5)>>5)
#define CLK_125kHz_toggle       (GPIOA->ODR ^= GPIO_Pin_6)  

#define bitrate_RF_8            0       // 15.6 kbt/sec, time bit period 64 usec  
#define bitrate_RF_16           4       // 7.8 kbt/sec, time bit period 128 usec
#define bitrate_RF_32           8       // 3.9 kbt/sec, time bit period 256 usec
#define bitrate_RF_40           12      // 3.1 kbt/sec, time bit period 320 usec
#define bitrate_RF_50           16      // 2.5 kbt/sec, time bit period 400 usec
#define bitrate_RF_64           20      // 1.9 kbt/sec, time bit period 512 usec
#define bitrate_RF_100          24      // 1.25 kbt/sec, time bit period 800 usec
#define bitrate_RF_128          28      // 0.976 kbt/sec, time bit period 1024 usec

#define modulationRF_DIR        0
#define modulationRF_PSK1       8
#define modulationRF_PSK2       16
#define modulationRF_PSK3       24
#define modulationRF_FSK1       32
#define modulationRF_FSK2       40
#define modulationRF_FSK1a      48
#define modulationRF_FSK2a      56
#define modulationRF_MANCH      64
#define modulationRF_BIPHASE    128

#define pskCF_RF_2              0
#define pskCF_RF_4              4
#define pskCF_RF_8              8
#define pskCF_RF_RES            12

#define manch                   1
#define biPhaseD                2
#define biPhaseR                3

#define testbit(var,_bit)       ((var >> _bit) & 1)

#define TIM_IT_Update           ((uint16_t)0x0001)
#define EXTI_Line5              ((uint32_t)0x00000020)  /*!< External interrupt line 5  */

void init_RFID125khz(void);
void init_configRFID(uint8_t bitrate, uint8_t modul, uint8_t pskCF);
void read_tag(uint8_t* buf);
void writeRFID(uint8_t* buf);
void read_tag(uint8_t* buf);

#endif
