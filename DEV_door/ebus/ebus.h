/* This is a part of the ETAN BUS project
 * Copyright (c) 2018-2021 ETAN Research & Development Center.
 */

#ifndef _EBUS_H_
#define _EBUS_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
#include <stddef.h>

typedef enum {
    line_free = 0,
    line_recieve,
    line_transmit,
    line_stuck,
    line_collision,
    line_plost,
    line_down,
} ebus_line_status_t;

/** @brief Callback from E-BUS receiver
 *  @param data data buffer
 *  @param count number of bits received (w/o CRC)
 *  @param priority ptiority of the received packet
 */
typedef void (*rx_callback)(void* data, size_t count, int priority);

/** @brief Initialize E-BUS
 *  @param cb_ok receive callback for the good packet
 *  @param cb_bad receive callback for the mailformed packet
 */
void ebus_init(rx_callback cb_ok, rx_callback cb_bad);

/** @brief Transmit RAW data buffer
 * @param buf pointer to the transmit data (w/o CRC)
 * @param size size of data in bits
 * @param priority packet priority (0 highest)
 * @return int error code
 */
int ebus_transmit(const void* buf, size_t size, int priority);

/** @brief Turn BUS on */
void ebus_up(void);

/** @brief Force BUS off */
void ebus_down(void);

/** @brief Read line status */
ebus_line_status_t ebus_get_line_status(void);

/** @brief Read last TX status */
ebus_line_status_t ebus_get_last_tx_status(void);

#ifdef __cplusplus
}
#endif
#endif //_EBUS_H_
