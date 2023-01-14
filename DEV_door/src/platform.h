/* This is a part of the ETAN BUS project
 * Copyright (c) 2018-2021 ETAN Research & Development Center.
 */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#ifdef __cplusplus
extern "C"
{
#endif

// Outputs
#define PIN_LED_G           13  /* GPIOA13 */

// Input
//#define PIN_BTN0            0   /** GPIOA0 */

#define RXFIFO_DEPTH        5
#define TXFIFO_DEPTH        3

#define LINE_BUFSIZE        32
#define TX_TMRNO            3   /* TX timer No */
#define TX_TMR_CHANNEL      4
#define TX_PIN              1   /* GPIOB1 */
#define RX_TMRNO            17  /* RX timer No */
#define RX_PIN              9   /* GPIOA9 */
#define RX_PORT             GPIOA
#define LINEOK_PORT         GPIOA
#define LINEOK_PIN          10

#define GPIO_INP(pin)   (0x00U << ((pin) << 1))
#define GPIO_OUT(pin)   (0x01U << ((pin) << 1))
#define GPIO_AFN(pin)   (0x02U << ((pin) << 1))
#define GPIO_ANA(pin)   (0x03U << ((pin) << 1))
#define GPIO_MSK(pin)   (0x03U << ((pin) << 1))

#define GPIO_PPL(pin)   (0x00U << (pin))
#define GPIO_ODN(pin)   (0x01U << (pin))

#define GPIO_LOS(pin)   (0x00U << ((pin) << 1))
#define GPIO_MES(pin)   (0x01U << ((pin) << 1))
#define GPIO_HIS(pin)   (0x03U << ((pin) << 1))

#define GPIO_NPU(pin)   (0x00U << ((pin) << 1))
#define GPIO_PUP(pin)   (0x01U << ((pin) << 1))
#define GPIO_PDN(pin)   (0x02U << ((pin) << 1))

#define GPIO_AF(pin, func) ((0x0FU & (func)) << ((0x07U & (pin)) << 2))

#ifdef __cplusplus
}
#endif
#endif //_PLATFORM_H_
