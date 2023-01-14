/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

unsigned int time_save;
bool change_PWM = false;
bool auto_PWM = false;
bool CUR_over = false;
bool flag_POWLINE_on = true;
bool usb_status_count = false;
bool flag_sendcmd = false;
bool flag_dec_duty_PWM = false;
bool flag_inc_duty_PWM = false;
bool flag_fine_tune_PWM = false;

uint8_t dev1_online = 0;
uint8_t dev2_online = 0;
uint8_t dev3_online = 0;
uint8_t cnt_dev1_ex = 0;
uint8_t cnt_dev2_ex = 0;
uint8_t cnt_dev3_ex = 0;
uint8_t stat_dev1 = 1;
uint8_t stat_dev2 = 1;
uint8_t stat_dev3 = 1;
uint8_t stop_dog = 0;
uint8_t bufOUTDATA[32];

uint16_t duty_PWM = 300;
uint16_t last_duty_PWM = 1;
uint16_t step_change_PWM = 1;
uint16_t count_debug = 0;

extern bool event_receive;
extern bool event_transmit;
extern uint8_t bufINDATA[32];
extern uint8_t Report_buf[64];
extern uint8_t Send_Buffer[64];
extern uint8_t ERR_transmit;
extern uint16_t buffer_ADC[1000];
extern uint16_t data_ADC_cur_line;
extern uint16_t data_ADC_line_HI;
extern uint16_t data_ADC_line_LOW;
extern uint16_t	data_ADC_line_VOL;
extern uint32_t count_dataINUSB;
extern uint16_t data_ADC_forDMA[4];

volatile uint32_t *VectorTable = (volatile uint32_t *)0x20000000;
uint32_t	*VectorTBL = (uint32_t *)0x20000000;



uint8_t receivebit (void) {
  uint8_t b;
  if (receive_BIT) {
    b = 1;
  } else {
    b = 0;
  }
  return b;
}



void Delay (int nTime) {
  double dell;
  dell = (double)(nTime)*6.7;
  nTime = (int)dell;
  while (nTime != 0) {
    nTime--;
  }
}



void DelayuS (__IO uint32_t nCount) {
  uint8_t a;
	while (nCount) {
    a = 4;
    while (a) {
      a--;
    }
    nCount--;
  }
}



void DelaymS (__IO uint32_t nCount) {
  while (nCount) {
    DelayuS(1000);
    nCount--;
  }
}



uint8_t status_EXPOW (void) {
  uint8_t b;
  if (status_FLAG_EX_POW) {
    b = 1;
  } else {
    b = 0;
  }
  return b;
}



bool status_INDATE (void) {
  bool b;
  if (status_DATA_IN) {
    b = true;
  } else {
    b = false;
  }
  return b;
}



void send_dataUSB (void);
uint8_t transmit_byte (uint8_t data);
uint8_t transmit_str (char* data);
uint8_t transmit_dim (uint8_t* data, uint16_t sz, bool prioritet);
uint8_t val_crc (uint8_t* dim, uint8_t count);

int main (void) {
  uint8_t count = 0, i;
  uint16_t count_bit_transm;
  for (i = 0; i < 48; i++) {
    *VectorTBL = *(__IO uint32_t*)(MAIN_PROGRAM_START_ADDRESS + (i<<2));	VectorTBL++;
  }
  __HAL_SYSCFG_REMAPMEMORY_SRAM();

  init_gpio_();
  // Initialize USB  //
  USBD_Init (&USB_Device_dev,
              &USR_desc,
              &USBD_HID_cb,
              &USR_cb);

  for (count = 0; count < 10; count++) {
    led_one_ON (LED_R);
    DelaymS (130);
    led_one_OFF (LED_R);
    DelaymS (130);
  }
  init_tmr16();
  init_tmr17();
  init_tmr3();
  DelaymS (500);
  EN_DRV_ON;
  init_ADC_DMA();
  DMA_Config();

  while (1) {
    DelaymS(50);
    if (count_debug>100) count_debug = 80;
		if (flag_sendcmd) {
      count = 0;
      count_bit_transm = 24;
      transmit_dim(bufOUTDATA, count_bit_transm, true);
      flag_sendcmd = false;
    }
    if (status_EXPOW() == 1) {
      SEL_RANGE_OFF;
      Send_Buffer[9] = 1;
    } else {
      SEL_RANGE_ON;
      Send_Buffer[9] = 0;
    }
    switch (USB_Device_dev.dev.device_status) {
      case USB_CONFIGURED:
        if (usb_status_count) {
          usb_status_count = false;
          USBD_Init(&USB_Device_dev, &USR_desc, &USBD_HID_cb, &USR_cb);
        }
        break;
      case USB_SUSPENDED:
        usb_status_count = true;
        break;
      case USB_UNCONNECTED:
        usb_status_count = true;
//        send_str_USB("USB_status UNCONNECTED ", 0);
        break;
      case USB_DEFAULT:
//        send_str_USB("DEFAULT ", 0);
        break;
      case USB_ADDRESSED:
//        send_str_USB("ADDRESSED ", 0);
        break;
    }
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed (uint8_t* file, uint32_t line) {
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1) { }
}
#endif
