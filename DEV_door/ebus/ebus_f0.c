/* This is a part of the ETAN BUS project
 * Copyright (c) 2018-2021 ETAN Research & Development Center.
 */

#include "ebus.h"
#include "compiler.h"
#include "platform.h"
#include "stm32.h"
#include "ebus_errors.h"
#include "system.h"

#ifndef RX_TMRNO
#pragma message "RX_TMRNO was not defined, using TIM17"
#define RX_TMRNO 17
#endif

#ifndef TX_TMRNO
#pragma message "TX_TMRNO was not defined, using TIM3"
#define TX_TMRNO 3
#endif

#ifndef TX_TMR_CHANNEL
#pragma message "TX_TMR_CHANNEL was not defined, using 1"
#define TX_TMR_CHANNEL 1
#endif

#define TX_TIM CAT(TIM, TX_TMRNO)
#define TX_TIM_IRQN CAT3(TIM, TX_TMRNO, _IRQn)
#define TX_TIM_HANDLER CAT3(TIM, TX_TMRNO, _IRQHandler)

#define RX_TIM CAT(TIM, RX_TMRNO)
#define RX_TIM_IRQN CAT3(TIM, RX_TMRNO, _IRQn)
#define RX_TIM_HANDLER CAT3(TIM, RX_TMRNO, _IRQHandler)

#if (RX_PIN >= 4)
#define RX_PIN_IRQN EXTI4_15_IRQn
#define RX_PIN_HANDLER EXTI4_15_IRQHandler
#elif (RX_PIN >=2)
#define RX_PIN_IRQN EXTI2_3_IRQn
#define RX_PIN_HANDLER EXTI2_3_IRQHandler
#else
#define RX_PIN_IRQN EXTI0_1_IRQn
#define RX_PIN_HANDLER EXTI0_1_IRQHandler
#endif

#ifndef EBUS_USE_CRC4
#define EBUS_USE_CRC4 0
//#pragma message "CRC4 for the short packets disabled by default"
#endif


#define IS_LINE_MARK        (RX_PORT->IDR & _BV(RX_PIN))
#define RX_TIM_START        RX_TIM->CR1 = TIM_CR1_CEN | TIM_CR1_OPM
#define RX_TIM_COUNT        RX_TIM->CNT
#define IS_LINE_TMR_STARTED RX_TIM->CR1 & TIM_CR1_CEN
#define TX_STOP             TX_TIM->CR1 = 0x00U; TX_TIM->CNT = 0x00U
#define IS_LINE_OK          (LINEOK_PORT->IDR & _BV(LINEOK_PIN))


#if (TX_TMR_CHANNEL == 1)
#define TX_TIM_CCR          TX_TIM->CCR1
#define TX_TIM_CCMR_SET     TX_TIM->CCMR1 = TIM_CCMR1_OC1FE | TIM_CCMR1_OC1M
#define TX_TIM_CCER_SET     TX_TIM->CCER = TIM_CCER_CC1E
#elif (TX_TMR_CHANNEL == 2)
#define TX_TIM_CCR          TX_TIM->CCR2
#define TX_TIM_CCMR_SET     TX_TIM->CCMR1 = TIM_CCMR1_OC2FE | TIM_CCMR1_OC2M
#define TX_TIM_CCER_SET     TX_TIM->CCER = TIM_CCER_CC2E
#elif (TX_TMR_CHANNEL == 3)
#define TX_TIM_CCR          TX_TIM->CCR3
#define TX_TIM_CCMR_SET     TX_TIM->CCMR2 = TIM_CCMR2_OC3FE | TIM_CCMR2_OC3M
#define TX_TIM_CCER_SET     TX_TIM->CCER = TIM_CCER_CC3E
#elif (TX_TMR_CHANNEL == 4)
#define TX_TIM_CCR          TX_TIM->CCR4
#define TX_TIM_CCMR_SET     TX_TIM->CCMR2 = TIM_CCMR2_OC4FE | TIM_CCMR2_OC4M
#define TX_TIM_CCER_SET     TX_TIM->CCER = TIM_CCER_CC4E
#else
#error "Incorrect TX_TMR_CHANNEL"
#endif

#define CRC8_POLY           0x63U
#define CRC4_POLY           0xB0U
#define DIV_BASE            1
#define SSPULSE             (6 * DIV_BASE)  // Single space pulse, us
#define SMPULSE             (12 * DIV_BASE) // Single mark pulse, us
#define SSSAMPLE            (23 * DIV_BASE) // Single space sample, us
#define SMSAMPLE            (71 * DIV_BASE) // Single mark sample, us
#define DSPULSE             (25 * DIV_BASE) // Double space pulse, us
#define DMPULSE             (77 * DIV_BASE) // Double mark pulse, us
#define DSSAMPLE            (44 * DIV_BASE) // Double space sample, us
#define DMSAMPLE            (142 * DIV_BASE)// Double mark sample, us DMPULSE
#define TSPULSE             (47 * DIV_BASE) // Triple space pulse, us


static uint8_t rx_crc8;
static uint8_t rx_crc4;
static size_t rx_bitcount;
static size_t tx_bitcount;
static uint8_t tx_priority;
static uint8_t rx_priority;
static size_t tx_pktsize;
static _ALIGNED32 uint8_t rxd_buf[LINE_BUFSIZE];
static _ALIGNED32 uint8_t txd_buf[LINE_BUFSIZE];
static rx_callback rx_goodpkt;
static rx_callback rx_badpkt;
static volatile ebus_line_status_t line_status;
static ebus_line_status_t last_tx_status;


/* Unable to compare port on the preprocessor stage
 * Let's compiler remove unwanted code
 */
_STATIC_INLINE void setup_syscfg(void) {
    _BST(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
    if (RX_PORT == GPIOA) {
        _BMD(SYSCFG->EXTICR[RX_PIN >> 2],
            0x0FU << (RX_PIN & 0x03U),
            0x00U << (RX_PIN & 0x03U));
        return;
    }
    if (RX_PORT == GPIOB) {
        _BMD(SYSCFG->EXTICR[RX_PIN >> 2],
            0x0FU << (RX_PIN & 0x03U),
            0x01U << (RX_PIN & 0x03U));
        return;
    }
    if (RX_PORT == GPIOC) {
        _BMD(SYSCFG->EXTICR[RX_PIN >> 2],
            0x0FU << (RX_PIN & 0x03U),
            0x02U << (RX_PIN & 0x03U));
        return;
    }
}

/**
 * @brief Set the up gpio int object
 *  setup EXTI for the DATA_RC
 */
static void setup_gpio_int() {
    setup_syscfg();
    _BST(EXTI->IMR, _BV(RX_PIN));
    _BST(EXTI->RTSR, _BV(RX_PIN));
    _BST(EXTI->FTSR, _BV(RX_PIN));
    NVIC_SetPriority (RX_PIN_IRQN, 0);
    NVIC_EnableIRQ (RX_PIN_IRQN);
}

/**
 * @brief Initialize TX timer (TIM3)
 *  Ttick = 1uS @ 48MHz,
 *  Upcounter, One pulse
 *  PWM mode 2 (output low when CNT < CCR1),
 *  Interrupt OVF
 */
static void setup_tx_tmr() {
    _BST(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
    TX_TIM->CR1 = TIM_CR1_OPM;
    TX_TIM->CNT = 0x00U;
    TX_TIM->PSC = 47U;
    TX_TIM_CCR = SSPULSE;
    TX_TIM->ARR = DMPULSE + SSPULSE;
    TX_TIM_CCMR_SET;
    TX_TIM_CCER_SET;
    TX_TIM->EGR = TIM_EGR_UG;
    TX_TIM->DIER = TIM_DIER_UIE;
    NVIC_SetPriority (TX_TIM_IRQN, 0);
    NVIC_EnableIRQ(TX_TIM_IRQN);
}

/**
 * @brief Initialize RX sample timer (TIM17)
 *  Ttick = 1uS @ 48MHz,
 *  Upcounter, One pulse
 *  Interrupt OVF over Tdmsample
 */
static void setup_rx_tmr() {
    _BST(RCC->APB2ENR, RCC_APB2ENR_TIM17EN);
    RX_TIM->CR1 = TIM_CR1_OPM;
    RX_TIM->PSC = 47U;
    RX_TIM->ARR = DMSAMPLE + 0x02U;
    RX_TIM->CNT = 0x00U;
    RX_TIM->RCR = 0x00U;
    RX_TIM->EGR = TIM_EGR_UG;
    RX_TIM->DIER = TIM_DIER_UIE;
    NVIC_SetPriority (RX_TIM_IRQN, 0);
    NVIC_EnableIRQ(RX_TIM_IRQN);
}

/** @brief return error code by line status
 *  @param status line status
 *  @return int error code
 */
static int error_by_status(int status) {
    switch (status) {
    case line_free:
        return ERROR_OK;
    case line_recieve:
    case line_transmit:
        return ERROR_LINE_BUSY;
    case line_stuck:
        return ERROR_LINE_STUCK;
    case line_collision:
        return ERROR_LINE_COLLISION;
    case line_plost:
        return ERROR_LINE_PLOST;
    default:
        break;
    }
    return ERROR_UNKNOWN;
}


/* RECEIVER */
void RX_PIN_HANDLER(void) {
    uint16_t tmr = RX_TIM_COUNT;
    EXTI->PR = _BV(RX_PIN);
    RX_TIM_COUNT = 0x00U;
    if (!IS_LINE_OK) {
        line_status = line_down;
        RX_TIM_START;
    } else if (IS_LINE_TMR_STARTED) {
        uint8_t bit = 0x00U;
        if (IS_LINE_MARK) {
            /* space->mark */
            if (tmr > SSSAMPLE) {
                bit = 0x80U;
            }
        } else {
            /* mark->space */
            if (tmr < SMSAMPLE) {
                bit = 0x80U;
            }
        }
        switch (line_status) {
        case line_transmit:
            /* checking recieved bit */
            if (rx_bitcount < 8 * sizeof(rxd_buf)) {
                uint8_t rbit;
                rbit = txd_buf[rx_bitcount >> 3];
                rbit <<= (rx_bitcount & 0x07U);
                rbit &= 0x80;
                if (rbit ^ bit) {
                    line_status = line_collision;
                    TX_STOP;
                }
            } else {
            /* checking priority bit */
                if (tx_priority ^ bit) {
                    line_status = line_plost;
                    TX_STOP;
                }
            }
            break;
        case line_recieve:
            if (rx_bitcount < 8 * sizeof(rxd_buf)) {
                rx_crc8 ^= bit;
                if (rx_crc8 & 0x80U) {
                    rx_crc8 = (rx_crc8 << 1) ^ CRC8_POLY;
                } else {
                    rx_crc8 <<= 1;
                }
#if (EBUS_USE_CRC4 != 0)
                rx_crc4 ^= bit;
                if (rx_crc4 & 0x80U) {
                    rx_crc4 = (rx_crc4 << 1) ^ CRC4_POLY;
                } else {
                    rx_crc4 <<= 1;
                }
#endif
                rxd_buf[rx_bitcount >> 3] |= (bit >> (rx_bitcount & 0x07));
            } else {
                /* if start bit or buffer overflow */
                rx_crc8 = 0xFFU;
                rx_crc4 = 0xF0U;
                rx_priority = bit;
            }
            break;
        default:
            return;
        }
        rx_bitcount++;
    } else {
        RX_TIM_START;
        if (IS_LINE_MARK) {
            /* space->mark => start recieving */
            rx_bitcount = 0xFFFFFFFFUL; //(-1) start bit index.
            /* make sure this buffer is 32-bit aligned */
            for (size_t i = 0; i < sizeof(rxd_buf) / 4; i++) {
                ((uint32_t*)rxd_buf)[i] = 0x00UL;
            }
            ebus_led_on();
            if (line_status == line_free) {
                line_status = line_recieve;
            }
        } else {
            /* mark->space => line stuck. waiting next bus timeout */
            line_status = line_stuck;
        }
    }
}

void RX_TIM_HANDLER(void) {
    RX_TIM->SR = 0x00U;
    if (!IS_LINE_OK) {
        line_status = line_down;
        RX_TIM_START;
    } else if (IS_LINE_MARK) {
        line_status = line_stuck;
        RX_TIM_START;
    } else {
        switch (line_status) {
        case line_recieve:
#if (EBUS_USE_CRC4 != 0)
            if (rx_bitcount >= 4 && rx_bitcount <= 0x10U && rx_crc4 == 0) {
                /* short packet */
                if (rx_goodpkt) {
                    rx_bitcount -= 4;
                    if (rx_bitcount & 0x07U) {
                        rxd_buf[rx_bitcount >> 3] &= ~(0xFFU >> (rx_bitcount & 0x07U));
                    }
                    rx_goodpkt(rxd_buf, rx_bitcount, rx_priority);
                }
            } else if (rx_bitcount > 0x10U && rx_bitcount < 8 * LINE_BUFSIZE && rx_crc8 == 0) {
                /* long packet */
                if (rx_goodpkt) {
                    rx_bitcount -= 8;
                    if (rx_bitcount & 0x07U) {
                        rxd_buf[rx_bitcount >> 3] &= ~(0xFFU >> (rx_bitcount & 0x07U));
                    }
                    rx_goodpkt(rxd_buf, rx_bitcount, rx_priority);
                }
#else
            if (rx_bitcount <= 8) {
                /* short packet */
                if (rx_goodpkt) {
                    rx_goodpkt(rxd_buf, rx_bitcount, rx_priority);
                }
            } else if (rx_bitcount > 0x08U && rx_bitcount < 8 * LINE_BUFSIZE && rx_crc8 == 0) {
                if (rx_goodpkt) {
                    rx_bitcount -= 8;
                    if (rx_bitcount & 0x07U) {
                        rxd_buf[rx_bitcount >> 3] &= ~(0xFFU >> (rx_bitcount & 0x07U));
                    }
                    rx_goodpkt(rxd_buf, rx_bitcount, rx_priority);
                }
#endif
            } else {
                /* mailformed packet */
                if(rx_badpkt) {
                    rx_badpkt(rxd_buf, rx_bitcount, rx_priority);
                }
            }
            break;
        case line_transmit:
            last_tx_status = line_free;
            break;
        case line_plost:
        case line_collision:
            last_tx_status = line_status;
            break;
        default:
            break;
        }
        line_status = line_free;
        ebus_led_off();
    }
}

/* TRANSMITTER */
int ebus_transmit(const void* buf, size_t size, int priority) {
    int counter, tmp, crc, poly;
    const uint8_t *data = buf;
    if (size == 0 || size > 8 * (LINE_BUFSIZE - 1) || (size & 0x01U)) {
        return ERROR_BAD_DATA;
    }
    /* copy data to buffer and calculate CRC */
#if (EBUS_USE_CRC4 != 0)
    if (size > 0x0CU) {
        poly = CRC8_POLY;
        crc = 0xFFU;
    } else {
        poly = CRC4_POLY;
        crc = 0xF0U;
    }
#else
    poly = CRC8_POLY;
    crc = 0xFFU;
#endif
    for (counter = 0; counter < size; counter++) {
        if (data[counter >> 3] & (0x80 >> (counter & 0x07U))) {
            crc ^= 0x80U;
        }
        if (crc & 0x80U) {
            crc = (crc << 1) ^ poly;
        } else {
            crc <<= 1;
        }
        if ((counter & 0x07U) == 0x00U) {
            txd_buf[counter >> 3] = data[counter >> 3];
        }
    }
    /* append CRC */
#if (EBUS_USE_CRC4 != 0)
    size += (size > 0x0C) ? 8 : 4;
#else
    size += (size > 0x08) ? 8 : 0;
#endif
    for ( ; counter < size; counter++) {
        txd_buf[counter >> 3] &= ~(0x80U >> (counter & 0x07U));
        txd_buf[counter >> 3] |= ((crc & 0x80U) >> (counter & 0x07U));
        crc <<= 1;
    }

    counter = 1000;
    while ((tmp = line_status) != line_free) {
        delay_us(10);
        counter--;
        if (counter == 0) {
            return error_by_status(tmp);
        }
    }

    if (priority) {
        tx_priority = 0x80U;
        tmp = SSPULSE + SMPULSE;
    } else {
        tx_priority = 0x00U;
        tmp = SSPULSE + DMPULSE;
    }
    tx_bitcount = 0x00U;
    tx_pktsize = size;

    TX_TIM->CNT = 0x00U;
    TX_TIM->SR = 0x00U;
    TX_TIM_CCR = SSPULSE;
    TX_TIM->ARR = tmp;
    TX_TIM->CR1 = TIM_CR1_CEN | TIM_CR1_OPM;
    line_status = line_transmit;

    /* start transmission*/
    counter = 2000;
    while (line_status == line_transmit) {
        delay_us(5);
        counter--;
        if (counter == 0) {
            TX_STOP;
            line_status = (IS_LINE_OK) ? line_free : line_down;
            return ERROR_TIMEOUT;
        }
    }
    return error_by_status(last_tx_status);
}

void TX_TIM_HANDLER(void) {
    int tmp;
    size_t bitcount = tx_bitcount;
    TX_TIM->SR = 0x00U;
    if (bitcount < tx_pktsize) {
        if (txd_buf[bitcount >> 3] & (0x80U >> (bitcount & 0x07U))) {
            tmp = DSPULSE;
        } else {
            tmp = SSPULSE;
        }
        bitcount++;
        TX_TIM_CCR = tmp;
        if (txd_buf[bitcount >> 3] & (0x80U >> (bitcount & 0x07U))) {
            tmp += SMPULSE;
        } else {
            tmp += DMPULSE;
        }
        TX_TIM->ARR = tmp;
        TX_TIM->CR1 = TIM_CR1_CEN | TIM_CR1_OPM;
        bitcount++;
        tx_bitcount = bitcount;
    }
}

/* other public functions */

void ebus_init(rx_callback cb_ok, rx_callback cb_bad) {
    __disable_irq();
    setup_gpio_int();
    setup_tx_tmr();
    setup_rx_tmr();
    rx_goodpkt = cb_ok;
    rx_badpkt = cb_bad;
    line_status = line_free;
    last_tx_status = line_free;
    __enable_irq();
}

void ebus_up(void) {
    LINEOK_PORT->BSRR = _BV(LINEOK_PIN);
    line_status = line_free;
}

void ebus_down(void) {
    LINEOK_PORT->BRR = _BV(LINEOK_PIN);
    line_status = line_down;
}

ebus_line_status_t ebus_get_line_status(void) {
    if (IS_LINE_OK) {
        return line_status;
    } else {
        return line_down;
    }
}

ebus_line_status_t ebus_get_last_tx_status(void) {
    return last_tx_status;
}
