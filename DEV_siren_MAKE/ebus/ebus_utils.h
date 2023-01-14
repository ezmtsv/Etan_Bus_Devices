/* This is a part of the ETAN BUS project
 * Copyright (c) 2018-2021 ETAN Research & Development Center.
 */

#ifndef _EBUS_UTILS_H_
#define _EBUS_UTILS_H_
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stddef.h>
#include "platform.h"
#include "compiler.h"

typedef struct {
    size_t   bitscount;
    int      priority;
    uint8_t  data[LINE_BUFSIZE];
} ebus_pkt_t;

/** @brief Initialize RX FIFO */
int ebus_rxfifo_init(void);

/** @brief Initialize TX FIFO */
int ebus_txfifo_init(void);

/** @brief push data to RX FIFO
 * @param const void* data packet data to push
 * @param size_t bitcount data size in bits
 * @param priority packet priority
 * @return Error code
 */
int ebus_rxfifo_push(const void* data, size_t bitcount, int priority);

/** @brief push data to TX FIFO
 * @param const void* data packet data to push
 * @param size_t bitcount data size in bits
 * @param priority packet priority
 * @return Error code
 */
int ebus_txfifo_push(const void* data, size_t bitcount, int priority);

/** @brief Push packet to RX FIFO
 * @param ebus_pkt_t packet packet to push
 * @return eError code
 */
inline static _MAYUNUSED int ebus_rxfifo_push_packet(const ebus_pkt_t* packet) {
    return ebus_rxfifo_push(packet->data, packet->bitscount, packet->priority);
}

/** @brief Push packet to TX FIFO
 * @param ebus_pkt_t packet packet to push
 * @return eError code
 */
inline static _MAYUNUSED int ebus_txfifo_push_packet(const ebus_pkt_t* packet) {
    return ebus_txfifo_push(packet->data, packet->bitscount, packet->priority);
}

/** @brief Get first packet from RX FIFO */
ebus_pkt_t* ebus_rxfifo_packet(void);

/** @brief Get first packet from TX FIFO */
ebus_pkt_t* ebus_txfifo_packet(void);

/** @brief Switch RX_FIFO to the next packet
 * @return Error code
 */
int ebus_rxfifo_next(void);

/** @brief Switch TX_FIFO to the next packet
 * @return Error code
 */
int ebus_txfifo_next(void);


#ifdef __cplusplus
}
#endif
#endif //_EBUS_UTILS_H_
