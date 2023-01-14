/* This is a part of the ETAN BUS project
 * Copyright (c) 2018-2021 ETAN Research & Development Center.
 */

#ifndef _UTILS_H_
#define _UTILS_H_
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stddef.h>
#include "compiler.h"

extern volatile uint32_t heartbeat_count;

void SystemInit(void);
void delay_us(uint32_t us);
void system_reset(void) _NORETURN;

/** @brief Turn line activity LED ON */
void ebus_led_on(void);

/** @brief Turn line activity LED OFF */
void ebus_led_off(void);

/** @brief Get 32-bit UID */
uint32_t ebus_uid(void);

void led_r_on(void);
void led_r_off(void);

void led_b_on(void);
void led_b_off(void);

void led_y_on(void);
void led_y_off(void);

void led_g_on(void);
void led_g_off(void);

uint8_t buttons_state(void);


#ifdef __cplusplus
}
#endif
#endif //_UTILS_H_