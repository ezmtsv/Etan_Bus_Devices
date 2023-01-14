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
#define PIN_ALARM           0  /* GPIOA0 */

// Input

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

#ifdef __cplusplus
}
#endif
#endif //_PLATFORM_H_