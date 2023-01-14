#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "usbd_hid_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_custom_hid_core.h"
#include "user_init.h"
#include "func.h"

#define BOOTLOADER_KEY_START_ADDRESS            (uint32_t)0x08003000
#define BOOTLOADER_KEY_PAGE_NUMBER              13
#define mode_work                               0xAABB5599
#define mode_programm                           0x77700777
#define mode_wait                               0x11221122

#define MAIN_PROGRAM_START_ADDRESS              (uint32_t)0x08003400
#define START_ADDRESS                           (uint32_t)0x08000000
#define MAIN_PROGRAM_PAGE_NUMBER                14
#define NUM_OF_PAGES                            32
#define FLASH_PAGE_SIZE                         1024
#define NVIC_VectTab_FLASH                      ((uint32_t)0x08000000)

#define erase                                   ':'
#define prg_mode                                'P'
#define packet_OK                               '?'
#define CRC_OK                                  '@'

#define __HAL_SYSCFG_REMAPMEMORY_SRAM()         do {SYSCFG->CFGR1 &= ~(SYSCFG_CFGR1_MEM_MODE); \
                                                SYSCFG->CFGR1 |= (SYSCFG_CFGR1_MEM_MODE_0 | SYSCFG_CFGR1_MEM_MODE_1); \
                                                }while(0)



void Delay(int nTime);
void DelaymS(__IO uint32_t nCount);
bool status_INDATE(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
