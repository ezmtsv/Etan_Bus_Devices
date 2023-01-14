/* This is a part of the ETAN BUS project
 * Copyright (c) 2018-2021 ETAN Research & Development Center.
 */

#include "compiler.h"
#include "platform.h"
#include "ebus_errors.h"
#include "ebus_utils.h"
#include <string.h>

typedef struct {
    volatile uint8_t    ridx;
    volatile uint8_t    widx;
    volatile uint8_t    isfull;
    uint8_t             depth;
    ebus_pkt_t          buffer[];
} _ALIGNED32 ebus_fifo_t;

static struct {
    uint8_t     ridx;
    uint8_t     widx;
    uint8_t     isfull;
    uint8_t     depth;
    ebus_pkt_t  buffer[RXFIFO_DEPTH];
} _ALIGNED32 rx_fifo;

static struct {
    uint8_t     ridx;
    uint8_t     widx;
    uint8_t     isfull;
    uint8_t     depth;
    ebus_pkt_t  buffer[TXFIFO_DEPTH];
} _ALIGNED32 tx_fifo;

static int ebus_fifo_push(void* buf, const void* data, size_t bitscount, int priority) {
    ebus_fifo_t *fifo = buf;
    ebus_pkt_t* wrpkt = &(fifo->buffer[fifo->widx]);
    size_t bytes = (bitscount + 0x07U) >> 3;
    size_t widx = fifo->widx;
    if (fifo->isfull) {
        return ERROR_FIFO_FULL;
    }
    wrpkt->bitscount = bitscount;
    wrpkt->priority = priority;
    memcpy(wrpkt->data, data, bytes);
    widx++;
    if (widx >= fifo->depth) {
        widx = 0x00;
    }
     if (widx == fifo->ridx) {
        fifo->isfull = 0x01;
    }
    fifo->widx = widx;
    return ERROR_OK;
}

static ebus_pkt_t* ebus_fifo_packet(void* buf) {
    ebus_fifo_t *fifo = buf;
    if (fifo->ridx != fifo->widx || fifo->isfull) {
        return &(fifo->buffer[fifo->ridx]);
    }
    return NULL;
}

static int ebus_fifo_next(void* buf) {
    ebus_fifo_t *fifo = buf;
    size_t ridx = fifo->ridx;
    if (ridx != fifo->widx || fifo->isfull) {
        fifo->isfull = 0;
        ridx++;
        if (ridx >= fifo->depth) {
            ridx = 0x00U;
        }
        fifo->ridx = ridx;
        return ERROR_OK;
    }
    return ERROR_FIFO_EMPTY;
}

static int ebus_fifo_init(void* buf, uint8_t depth) {
    ebus_fifo_t *fifo = buf;
    fifo->depth = depth;
    fifo->ridx = 0x00U;
    fifo->widx = 0x00U;
    fifo->isfull = 0x00U;
    return ERROR_OK;
}


int ebus_rxfifo_init(void) {
    return ebus_fifo_init(&rx_fifo, RXFIFO_DEPTH);
}

int ebus_txfifo_init(void) {
    return ebus_fifo_init(&tx_fifo, TXFIFO_DEPTH);
}

int ebus_rxfifo_push(const void* data, size_t bitscount, int priority) {
    return ebus_fifo_push(&rx_fifo, data, bitscount, priority);
}

int ebus_txfifo_push(const void* data, size_t bitscount, int priority) {
    return ebus_fifo_push(&tx_fifo, data, bitscount, priority);
}

ebus_pkt_t* ebus_rxfifo_packet(void) {
    return ebus_fifo_packet(&rx_fifo);
}

ebus_pkt_t* ebus_txfifo_packet(void) {
    return ebus_fifo_packet(&tx_fifo);
}

int ebus_rxfifo_next(void) {
    return ebus_fifo_next(&rx_fifo);
}

int ebus_txfifo_next(void) {
    return ebus_fifo_next(&tx_fifo);
}
