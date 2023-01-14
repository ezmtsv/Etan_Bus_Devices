
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"
#include <string.h>

extern bool CUR_over;

//extern __IO uint32_t Gv_EOA;
extern USB_CORE_HANDLE USB_Device_dev;
extern uint8_t Send_Buffer[64];//65
extern uint8_t Report_buf[64];
extern uint8_t PrevXferDone;
extern uint8_t count_debug;
extern uint8_t stop_dog;
extern uint8_t dev1_online;
extern uint8_t dev2_online;
extern uint8_t dev3_online;
extern uint8_t cnt_dev1_ex;
extern uint8_t cnt_dev2_ex;
extern uint8_t cnt_dev3_ex;
extern uint8_t stat_dev1;
extern uint8_t stat_dev2;
extern uint8_t stat_dev3;
extern uint16_t duty_PWM;
extern uint16_t last_duty_PWM;
extern uint16_t data_ADC_forDMA[8];
extern uint16_t data_ADC_cur_line;
extern uint16_t data_ADC_line_HI;
extern uint16_t data_ADC_line_LOW;
extern uint16_t data_ADC_line_VOL;
extern int all_packet;
extern int current_packet;
extern uint32_t prg_data;

bool int_pinIN		= false;
bool data_inBUF		= false;
bool ERR_for_DEBUG		= false;
bool flag_but = false;
bool start_IN = false;
bool event_transmit = false;
bool event_receive = false;
uint8_t bufINDATA[32];
uint8_t ERR_transmit = 0;  // 0 - передача успешна, 1 - линия занята, 2 - потеря приоритета
uint8_t dim_debug[64];
uint8_t previousState = 0;
uint8_t previousStatetmr17 = 0;
uint8_t count_adcLine = 0;
uint8_t first_bit = 0;
uint8_t transmit_dim(uint8_t* data, uint16_t sz, bool prioritet);
uint8_t transmit_str(char* data);
uint16_t count_receive_byte = 0;
int16_t count_crc_ERR = 0;
uint16_t count_receive_bit = 0;
uint16_t date_tmp = 0;
uint16_t count_receive_bit_debug = 0;
uint16_t timeSP_real;
uint16_t timeMR_real;
//uint16_t count_debug = 0;
//uint16_t count_debug2 = 0;

uint32_t countState = 0;
uint32_t countStatetmr17 = 0;
uint32_t countStateusb = 0;
uint32_t flash_read_date;
uint32_t  prog_date_tmp;
uint32_t currentAddress;						//= 0x08005000;
uint32_t count_dataIN = 0;
uint32_t count_dataINUSB;

//int sss;
//int rrr = 0;

void DelaymS (__IO uint32_t nCount);
void send_dataUSB (void);
uint8_t status_EXPOW (void);
bool status_INDATE (void);
void end_receive (void);

 uint8_t crctable[256] = {
  0x00, 0x63, 0xC6, 0xA5, 0xEF, 0x8C, 0x29, 0x4A, 0xBD, 0xDE, 0x7B, 0x18, 0x52, 0x31, 0x94, 0xF7,
  0x19, 0x7A, 0xDF, 0xBC, 0xF6, 0x95, 0x30, 0x53, 0xA4, 0xC7, 0x62, 0x01, 0x4B, 0x28, 0x8D, 0xEE,
  0x32, 0x51, 0xF4, 0x97, 0xDD, 0xBE, 0x1B, 0x78, 0x8F, 0xEC, 0x49, 0x2A, 0x60, 0x03, 0xA6, 0xC5,
  0x2B, 0x48, 0xED, 0x8E, 0xC4, 0xA7, 0x02, 0x61, 0x96, 0xF5, 0x50, 0x33, 0x79, 0x1A, 0xBF, 0xDC,
  0x64, 0x07, 0xA2, 0xC1, 0x8B, 0xE8, 0x4D, 0x2E, 0xD9, 0xBA, 0x1F, 0x7C, 0x36, 0x55, 0xF0, 0x93,
  0x7D, 0x1E, 0xBB, 0xD8, 0x92, 0xF1, 0x54, 0x37, 0xC0, 0xA3, 0x06, 0x65, 0x2F, 0x4C, 0xE9, 0x8A,
  0x56, 0x35, 0x90, 0xF3, 0xB9, 0xDA, 0x7F, 0x1C, 0xEB, 0x88, 0x2D, 0x4E, 0x04, 0x67, 0xC2, 0xA1,
  0x4F, 0x2C, 0x89, 0xEA, 0xA0, 0xC3, 0x66, 0x05, 0xF2, 0x91, 0x34, 0x57, 0x1D, 0x7E, 0xDB, 0xB8,
  0xC8, 0xAB, 0x0E, 0x6D, 0x27, 0x44, 0xE1, 0x82, 0x75, 0x16, 0xB3, 0xD0, 0x9A, 0xF9, 0x5C, 0x3F,
  0xD1, 0xB2, 0x17, 0x74, 0x3E, 0x5D, 0xF8, 0x9B, 0x6C, 0x0F, 0xAA, 0xC9, 0x83, 0xE0, 0x45, 0x26,
  0xFA, 0x99, 0x3C, 0x5F, 0x15, 0x76, 0xD3, 0xB0, 0x47, 0x24, 0x81, 0xE2, 0xA8, 0xCB, 0x6E, 0x0D,
  0xE3, 0x80, 0x25, 0x46, 0x0C, 0x6F, 0xCA, 0xA9, 0x5E, 0x3D, 0x98, 0xFB, 0xB1, 0xD2, 0x77, 0x14,
  0xAC, 0xCF, 0x6A, 0x09, 0x43, 0x20, 0x85, 0xE6, 0x11, 0x72, 0xD7, 0xB4, 0xFE, 0x9D, 0x38, 0x5B,
  0xB5, 0xD6, 0x73, 0x10, 0x5A, 0x39, 0x9C, 0xFF, 0x08, 0x6B, 0xCE, 0xAD, 0xE7, 0x84, 0x21, 0x42,
  0x9E, 0xFD, 0x58, 0x3B, 0x71, 0x12, 0xB7, 0xD4, 0x23, 0x40, 0xE5, 0x86, 0xCC, 0xAF, 0x0A, 0x69,
  0x87, 0xE4, 0x41, 0x22, 0x68, 0x0B, 0xAE, 0xCD, 0x3A, 0x59, 0xFC, 0x9F, 0xD5, 0xB6, 0x13, 0x70
};



uint8_t val_crc (uint8_t* dim, uint8_t count) {
  uint8_t crc = 0xFF;
  uint8_t val;
  while (count-->0) {
    val = *dim++;
    crc = crctable[crc^val];
  }
  return crc;
}



void NMI_Handler(void) { }



void HardFault_Handler (void) {
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1) {
    led_one_OFF(LED_R);
    DelaymS(400);
    led_one_ON(LED_R);
    DelaymS(400);
  }
}



void SVC_Handler (void) { }

void PendSV_Handler (void) { }

void SysTick_Handler (void) { }



void USB_IRQHandler (void) {
  USB_Istr();
}



uint32_t flash_read (uint32_t address) {
  return (*(__IO uint32_t*) address);
}



void erase_page_() {
  FLASH_Unlock();
  FLASH_ErasePage (0x08005000);
  FLASH_Lock();
}



void EXTI0_1_IRQHandler(void) {
  /* Clear the EXTI line pending bit */
  EXTI_ClearITPendingBit(EXTI_Line1);
}




// прерывание от линии
void EXTI4_15_IRQHandler (void) {
  if (!event_transmit) {
    if (EXTI->PR & EXTI_Line5) {
      __disable_irq();
      led_one_ON(LED_R);
      count_receive_bit = 0;
      count_receive_byte = 0;
      date_tmp = 0;
      count_receive_bit_debug = 0;
      TIM_Cmd(TIM17, ENABLE);
      countTIM17 = 0;
      while (status_DATA_IN) {
        if (countTIM17>DMSAMPLE) {
          goto end_int;
        }
      }
      timeMR_real = countTIM17;
      if (timeMR_real<SMSAMPLE) {
        first_bit = 1;
      } else {
        first_bit = 0;
      }
      countTIM17 = 0;
      while (countTIM17<DMSAMPLE) {
        while (!status_DATA_IN) {
          if (countTIM17>TSPULSE) {
            goto end_int;
          }
        }
        timeSP_real = countTIM17;
        if (timeSP_real>=SSSAMPLE) {
          date_tmp |= 1;
        }			//// rec_bit = true;
        if (!ERR_for_DEBUG) {
          dim_debug[count_receive_bit_debug] = timeSP_real;
        }
        countTIM17 = 0;
        count_receive_bit++;
        count_receive_bit_debug++;
        date_tmp <<= 1;
        while (status_DATA_IN) {
          if (countTIM17>DMSAMPLE) {
            goto end_int;
          }
        }
        timeMR_real = countTIM17;
        if (timeMR_real<SMSAMPLE) {
          date_tmp |= 1;
        } 		//// rec_bit = true;
        date_tmp <<= 1;
        if (!ERR_for_DEBUG) {
          dim_debug[count_receive_bit_debug] = timeMR_real;
        }
        countTIM17 = 0;
        count_receive_bit++;
        count_receive_bit_debug++;
        if (count_receive_bit == 8) {
          count_receive_bit = 0;
          date_tmp >>= 1;
          bufINDATA[count_receive_byte] = (uint8_t)(date_tmp);
          date_tmp = 0;
          count_receive_byte++;
        }
      }
    }
end_int:
      if (count_receive_bit_debug >1) {
        data_inBUF = true; end_receive();
      }
      __enable_irq();
    } else {
      event_transmit = false;
    }
    if (EXTI->PR & EXTI_Line7) {
      if (status_EXPOW() == 1) {			// если внешнее питание подключено
        SEL_RANGE_OFF;
      } else {
        SEL_RANGE_ON;
      }
    }
    EXTI_ClearITPendingBit(EXTI_Line7);
    EXTI_ClearITPendingBit(EXTI_Line5);
}



void handle_BUS (void) {
  switch (bufINDATA[0]) {
    case 1:
      cnt_dev1_ex = 0;
      dev1_online = 1;
      stat_dev1 = bufINDATA[1];
      break;
    case 2:
      cnt_dev2_ex = 0;
      dev2_online = 1;
      stat_dev2 = bufINDATA[1];
      break;
    case 3:
      cnt_dev3_ex = 0;
      dev3_online = 1;
      stat_dev3 = bufINDATA[1];
      break;
  }
}



void end_receive (void) {
  uint8_t buf_tmp = 0;

	led_one_OFF (LED_R);
  if (count_receive_bit > 0) {
    buf_tmp = (uint8_t)date_tmp;
    buf_tmp = buf_tmp << (7 - count_receive_bit);
    bufINDATA[count_receive_byte] = (uint8_t)(buf_tmp);
    count_receive_bit = count_receive_bit + 1 + count_receive_byte * 8;
    count_receive_byte++;
  } else {
    count_receive_bit = count_receive_bit + 1 + count_receive_byte * 8;
  }
  if (data_inBUF) {
    if (count_receive_bit > 16 && (count_receive_bit - 1) % 8 == 0) {
      if (val_crc(bufINDATA, count_receive_byte) != 0) {
        count_crc_ERR++;
        ERR_for_DEBUG = true;
      } else {
        handle_BUS();
      }
    }
    ERR_for_DEBUG = true;
  }
}



void TIM17_IRQHandler (void) {
  TIM_Cmd (TIM17, DISABLE);
  event_receive = false;
  event_transmit = false;
  TIM_ClearITPendingBit (TIM17, TIM_IT_Update);
}



void TIM3_IRQHandler (void) {
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}



void TIM16_IRQHandler (void) {
  uint8_t i = 0;
  Send_Buffer[1] = (uint8_t)(data_ADC_forDMA[ADC_cur_line] >> 8);
  Send_Buffer[2] = (uint8_t)(data_ADC_forDMA[ADC_cur_line]);
  Send_Buffer[3] = (uint8_t)(data_ADC_forDMA[ADC_line_HI] >> 8);
  Send_Buffer[4] = (uint8_t)(data_ADC_forDMA[ADC_line_HI]);
  Send_Buffer[5] = (uint8_t)(data_ADC_forDMA[ADC_line_LOW] >> 8);
  Send_Buffer[6] = (uint8_t)(data_ADC_forDMA[ADC_line_LOW]);
  Send_Buffer[7] = ERR_transmit;
  Send_Buffer[10] = (uint8_t)(data_ADC_forDMA[ADC_line_VOL] >> 8);
  Send_Buffer[11] = (uint8_t)(data_ADC_forDMA[ADC_line_VOL]);

  if (data_inBUF && !event_receive ) {
    for (i = 0; i < count_receive_byte; i++) {
      Send_Buffer[i+13] = bufINDATA[i];
    }
    Send_Buffer[62] = count_receive_byte;
    Send_Buffer[63] = count_receive_bit;
    Send_Buffer[60] = (uint8_t)(count_crc_ERR>>8);
    Send_Buffer[61] = (uint8_t)count_crc_ERR;
    data_inBUF = false;
  }

  if (ERR_for_DEBUG) {
    for (i = 0; i < 18; i++) {
      Send_Buffer[32+i] = dim_debug[i];
    }
    Send_Buffer[58] = count_receive_bit_debug;
    ERR_for_DEBUG = false;
  }
  Send_Buffer[22] = dev1_online;
  Send_Buffer[24] = dev2_online;
  Send_Buffer[26] = dev3_online;
  Send_Buffer[23] = stat_dev1;
  Send_Buffer[25] = stat_dev2;
  Send_Buffer[27] = stat_dev3;
  cnt_dev1_ex++;
  cnt_dev2_ex++;
  cnt_dev3_ex++;
  if (cnt_dev1_ex > 5) {
    cnt_dev1_ex = 0;
    dev1_online = 0;
  }
  if (cnt_dev2_ex > 5) {
    cnt_dev2_ex = 0;
    dev2_online = 0;
  }
  if (cnt_dev3_ex > 5) {
    cnt_dev3_ex = 0;
    dev3_online = 0;
  }
  send_dataUSB();
  TIM_ClearITPendingBit (TIM16, TIM_IT_Update);
}



void DMA1_Channel1_IRQHandler (void) {
  DMA_ClearFlag(DMA1_FLAG_TC1);
}



void ADC1_COMP_IRQHandler (void) {
  ADC_ClearITPendingBit (ADC1, ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_OVR);
}



void TIM14_IRQHandler (void) {
  TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
}



void eixit_CONFL_SP (void) {
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init (PORTA, &GPIO_InitStructure);
  DATA_OUT_LOW;
  TIM_DeInit (TIM3);

  //config DATAOUT as PWM
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init (PORTA, &GPIO_InitStructure);
  GPIO_PinAFConfig (GPIOA, GPIO_PinSource6, GPIO_AF_1);

  TIM3->PSC = (48*DIV_base)-1;			//  при DIV_base = 1 - 1мксек
  *((uint32_t*)reg_tim3_ARR) = 2;
  *((uint32_t*)reg_tim3_CCR1) = 2;
  // разрешаем работу 1 канала(пин А4)
  TIM3->CCER |= TIM_CCER_CC1E;
  //разрешим использовать выводы таймера как выходы
  TIM3->BDTR |= TIM_BDTR_MOE;
  // для режима одиночный импульс
  // (TIMx_ARR - TIMx_CCR1 + 1) - время импульса, значение в TIMx_CCR1 - время задержки перед импульсом
  // формирует импульс "1"
  TIM3->CCMR1 |= (TIM_CCMR1_OC1M_0| TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
  // формирует импульс "0"
  // TIM3->CCMR1|=(TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
  //считаем вверх
		TIM3->CR1 &= ~TIM_CR1_DIR;
		TIM3->CR1 |= TIM_CR1_OPM;							// режим одиночного импульса
		TIM3->CR1 |= TIM_CR1_CEN;
}



void read_data_line (uint16_t data, bool tr_marker) {
  if (!tr_marker) {
    date_tmp <<= 1;
    countTIM17 = 24;
    count_receive_bit++;
    count_receive_bit_debug++;
    if (count_receive_bit == 8) {
      count_receive_bit = 0;
      date_tmp >>= 1;
      bufINDATA[count_receive_byte] = (uint8_t)(date_tmp);
      date_tmp = 0;
      count_receive_byte++;
    }
    Send_Buffer[59] = count_receive_bit;
    Send_Buffer[60] = count_receive_bit_debug;
    Send_Buffer[61] = 99;
  } else {
    Send_Buffer[61]= 77;
  }
  while (countTIM17 < DMSAMPLE) {
    while (status_DATA_IN) {
      if (countTIM17 > DMSAMPLE) {
        goto end_read;
      }
    }
    timeMR_real = countTIM17;
    if (timeMR_real < SMSAMPLE) {
      date_tmp |= 1;
    } 		// rec_bit = true;
    date_tmp <<= 1;
    if (!ERR_for_DEBUG) {
      dim_debug[count_receive_bit_debug] = timeMR_real;
    }
    countTIM17 = 0;
    count_receive_bit++;
    count_receive_bit_debug++;
    if (count_receive_bit == 8) {
      count_receive_bit = 0;
      date_tmp >>= 1;
      bufINDATA[count_receive_byte] = (uint8_t)(date_tmp);
      date_tmp = 0;
      count_receive_byte++;
    }
    while (!status_DATA_IN) {
      if (countTIM17>TSPULSE) {
        goto end_read;
      }
    }
    timeSP_real = countTIM17;
    if (timeSP_real >= SSSAMPLE) {
      date_tmp |= 1;
    }			// rec_bit = true;
    if (!ERR_for_DEBUG) {
      dim_debug[count_receive_bit_debug] = timeSP_real;
    }
    countTIM17 = 0;
    count_receive_bit++;
    count_receive_bit_debug++;
    date_tmp <<= 1;
    if (count_receive_bit == 8) {
      count_receive_bit = 0;
      date_tmp >>= 1;
      bufINDATA[count_receive_byte] = (uint8_t)(date_tmp);
      date_tmp = 0;
      count_receive_byte++;
    }
  }
end_read:
  if (count_receive_bit_debug > 1) {
    data_inBUF = true;
    end_receive();
  }
}



uint8_t transmit_dim (uint8_t* data, uint16_t sz, bool prioritet) {
  #define tmr_ARR (wide+1)
  #define tmr_CCR1	(tmr_ARR+1-wide)
  uint8_t busy;
  volatile uint16_t dim_forIMP[130][2];						// в массиве значения для программирования времени импульсов и пауз для передачи
  volatile uint16_t dim_forIMP_real[130][2];			// в массиве длительности импульсов и пауз для передачи
  uint16_t k = 0;
  uint16_t tmp_arr;
  uint16_t i = 0;
  uint16_t time = 0;
  uint16_t num_b = 0;
  bool bit_d;
  bool line_HI = false;
  uint8_t cont_b = 0;
  ERR_transmit = send_OK;

  //crc для передачи целого числа байт
  if (sz >= 8 && sz % 8 == 0){
    data[sz/8] = val_crc(data, sz/8);
    sz += 8;
  }
  //параметры времени для первого бита
	if (prioritet) {
    dim_forIMP[0][1] = DMPULSE+1;
    dim_forIMP_real[0][1] = DMPULSE ;
  } else {
    dim_forIMP[0][1] = timetick+1;
    dim_forIMP_real[0][1] = timetick;
  }
  dim_forIMP[0][0] = 2;
  //формируем массив параметров времени импульсов для данных на входе функции
	if (prioritet) {
    dim_forIMP[0][1] = DMPULSE+1;
    dim_forIMP_real[0][1] = DMPULSE ;
  } else {
    dim_forIMP[0][1] = timetick+1;
    dim_forIMP_real[0][1] = timetick;
  }
  dim_forIMP[0][0] = 2;
  for (i = 0; i < sz; i++) {
    k = (i+2)/2;
    bit_d = (data[num_b]<<cont_b) & 0x80;	    // первым идет старший бит, если 1, то bit_d = true
		if (i%2) {                                // передача  высоким уровнем линии
      if (bit_d) {
        dim_forIMP[k][0] = SMPULSE;
      } else {
        dim_forIMP[k][0] = DMPULSE;
      }
    } else {                                  // передача низким уровнем линии
      if (bit_d) {
        dim_forIMP[k][1] = DSPULSE;
      } else {
        dim_forIMP[k][1] = SSPULSE;
      }
    }
    cont_b++;
    if (cont_b == 8) {
      cont_b = 0;
      num_b++;
    }
  }
  if (sz%2) {                                 // нечетное кол-во бит без старт маркера
    dim_forIMP[k][0] = DMPULSE;               // дописываем ноль, для нечетного кол-ва бит
  }
  k++;
  //сохраняем реальные параметры паузы и импульса в dim_forIMP_real[][], а исходный dim_forIMP перезаписываем данными для записи в рег.
  for (i = 1; i < k; i++) {
    dim_forIMP_real[i][0] = dim_forIMP[i][0];
    dim_forIMP_real[i][1] = dim_forIMP[i][1];
    dim_forIMP[i][1] = dim_forIMP[i][1]- 2;
    tmp_arr = dim_forIMP[i][0]+dim_forIMP[i][1];
    dim_forIMP[i][0] = dim_forIMP[i][1]+1;    // время импульса для программирования в рег. CCR1 ..///// корректировка времени для полного имп.( 0 + 1 )
    dim_forIMP[i][1] = tmp_arr;               // время для паузы перед импульсом для программирования в рег. ARR
  }
  event_transmit = true;		                  // включаем флаг событие передачи
//		1. Передачу первого маркера можно начинать только если:
//   - нет чужих активных передач,
//   - либо прошло не более, чем SMPULSE-1(12*DIV) микросекунд от начала первого маркера чужой посылки.
  if (status_DATA_IN) {
    busy = line_busy;
  } else {
    busy = send_OK;
  }
  if (busy == send_OK) {
    //слушаем линию на занятость
    countTIM17 = 0;
    TIM_Cmd (TIM17, ENABLE);
    while (countTIM17 < (waitST_send)) {      // для начала передачи выжидаем интервал (TSPULSE-DSSAMPLE) в течение которого линии должна быть пассивна
      if (status_DATA_IN) {
        busy = line_busy;
      }
    }
    if (busy == line_busy) {                  // линия оказалась занята, ждем SMPULSE_1, если после этого линия освободилась - начинаем передачу
      countTIM17 = 0;
      while (countTIM17<SMPULSE_1) { }
        if (status_DATA_IN) {
          ;
        } else {
          busy = send_OK;
        }
    }
  }

  // начинаем передачу
  if (busy == send_OK) {
    countTIM17 = 0;
    count_receive_bit = 0;
    count_receive_byte = 0;
    date_tmp = 0;
    count_receive_bit_debug = 0;
    __disable_irq();
    for (i = 0; i < k; i++) {
      while (status_DATA_IN) {                                  //отработка колизий при передаче маркера
        if (i != 0) {
          if (dim_forIMP_real[i-1][0] == SMPULSE) {             // передаем 1 высоким уровнем
            if (countTIM17 > SMSAMPLE) {
              busy = lost_priority;
              ERR_transmit = lost_priority;
              read_data_line(date_tmp, true);
              goto end_tr;
            }
          }
          if (dim_forIMP_real[0][1] == SMPULSE && i == 1) {     // потеря приоритета при на нулевом бите (мастер-девайс)
            if (countTIM17 > SMSAMPLE) {
              busy = lost_priority;
              ERR_transmit = lost_priority;
              first_bit = 1;                                    // first_bit  - бит указывающий на принятие сообщения с высшим приоритетом
              while (status_DATA_IN) {
                if (countTIM17 > DMSAMPLE) {
                  busy = line_busy;
                  goto end_tr;
                }
              }
              countTIM17 = 0;
              while (!status_DATA_IN) {
                if (countTIM17 > TSPULSE) {
                  goto end_tr;
                }
              }
              timeSP_real = countTIM17;
              if (timeSP_real >= SSSAMPLE) {
                date_tmp |= 1;
              }                                                   // rec_bit = true;
              if (!ERR_for_DEBUG) {
                dim_debug[count_receive_bit_debug] = timeSP_real;
              }
              countTIM17 = 0; count_receive_bit++;
              count_receive_bit_debug++;
              date_tmp <<= 1;
              read_data_line (date_tmp, true);
              goto end_tr;
            } else {
              first_bit = 0;
            }
          }
        }
        if (countTIM17 > DMSAMPLE) {
          busy = line_busy;
          goto end_tr;
        }                                                         // при зависании линии
      }
      if (i > 1) {
        time = countTIM17;
        if (time < SMSAMPLE) {
          date_tmp |= 1;
        }                                                         // сохраняем данные переданные единицей
        date_tmp <<= 1;
        if (!ERR_for_DEBUG) {
          dim_debug[count_receive_bit_debug] = time;
        }
        count_receive_bit++;
        count_receive_bit_debug++;
      }
      if (count_receive_bit == 8) {
        count_receive_bit = 0;
        date_tmp >>= 1;
        bufINDATA[count_receive_byte] = (uint8_t)(date_tmp);
        date_tmp = 0;
        count_receive_byte++;
      }
      countTIM17 = 0;
      line_HI = false;
      *((uint32_t*)reg_tim3_ARR) = dim_forIMP[i][1];
      *((uint32_t*)reg_tim3_CCR1) = dim_forIMP[i][0];
      *((uint32_t*)reg_tim3_CR1) |= ((uint16_t)TIM_CR1_CEN);			// 	запуск пауза-импульс
      while ((*((uint32_t*)reg_tim3_CR1)) &  TIM_CR1_CEN) {
        if (!line_HI) {
          //отработка колизий при передаче пробела
          if (status_DATA_IN) {
            line_HI = true;
            time = countTIM17;
            if (dim_forIMP_real[i][1] == DSPULSE) {               // передаем 1 низким уровнем
//              if (i==4) { time = SMPULSE; }                         //тест коллизий пробела
              if (time < SSSAMPLE) {                              //  countTIM17 < SSSAMPLE
                *((uint32_t*)reg_tim3_CR1) &= ~((uint16_t)TIM_CR1_CEN);     // выключаем TIM3
                eixit_CONFL_SP();
                busy = lost_priority;
                ERR_transmit = lost_priority;
                read_data_line (date_tmp, false);
                goto end_tr;
              }
            }
            if (i != 0) {
              if (time >= SSSAMPLE) {
                date_tmp |= 1;
              }                                                     // сохраняем данные переданные нулем
              if (!ERR_for_DEBUG) {
                dim_debug[count_receive_bit_debug] = time;
              }
              count_receive_bit++;
              count_receive_bit_debug++;
              date_tmp <<= 1;
            }
            countTIM17 = 0;
          }
        }
      }
    }
    //сохраняем значение последнего бита
    while (status_DATA_IN) {                                        //отработка колизий при передаче маркера
      if (i != 0) {
        if (dim_forIMP_real[i-1][0] == SMPULSE) {                   // передаем 1 высоким уровнем
          if (countTIM17 > SMSAMPLE) {
            busy = lost_priority;
            ERR_transmit = lost_priority;
            read_data_line (date_tmp, true);
            goto end_tr;
          }
        }
      }
      if (countTIM17 > DMSAMPLE) {
        busy = line_busy;
        goto end_tr;
      }	                                                            // линия зависла
    }
    time = countTIM17;
    if (time < SMSAMPLE) {
      date_tmp |= 1;
    } 		                                                          // rec_bit = true;
    date_tmp <<= 1;
    if (!ERR_for_DEBUG) {
      dim_debug[count_receive_bit_debug] = time;
    }
    count_receive_bit++;
    count_receive_bit_debug++;
    if (count_receive_bit == 8) {
      count_receive_bit = 0;
      date_tmp >>= 1;
      bufINDATA[count_receive_byte] = (uint8_t)(date_tmp);
      date_tmp = 0;
      count_receive_byte++;
    }
  }

  end_tr:
  __enable_irq();
  return busy;
}
