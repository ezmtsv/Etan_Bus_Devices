/* This is a part of the ETAN BUS project
 * Copyright (c) 2018-2021 ETAN Research & Development Center.
 */
#include "system.h"
#include "platform.h"
#include "stm32.h"
#include <string.h>

volatile uint32_t heartbeat_count;

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

/** @brief Setup clock HSE @8MHz, SYSCLK 48MHz PLL
 */
static void setup_sysclk(void) {
    /* Enable Prefetch Buffer and set Flash Latency */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
    /* HCLK = SYSCLK ; PCLK = SYSCLK*/
    _BST(RCC->CFGR, RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE_DIV1);
    /* Enable HSE */
    _BST(RCC->CR, RCC_CR_HSEON);
    _WBS(RCC->CR, RCC_CR_HSERDY);

    /* PLL configuration = HSE * 6 = 48 MHz */
    _BMD(RCC->CFGR,
         RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL,
         RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL6);
    _BST(RCC->CR, RCC_CR_PLLON);
    _WBS(RCC->CR, RCC_CR_PLLRDY);
    /* Switch SYSCLK to PLL */
    _BMD(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
    _WVL(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL);
}

/**
 * @brief Set the up GPIOA
 * LINE_OK as OD pullup
 * RX_PIN as IN no pullup
 * BTN0-3 as IN pullup
 * LEDS as OUT
 */
static void setup_gpioa(void) {
    _BST(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
    GPIOA->ODR = 0x00U;
    GPIOA->MODER = GPIO_INP(PIN_BTN0)  | GPIO_INP(PIN_BTN1)  |
                   GPIO_INP(PIN_BTN2)  | GPIO_INP(PIN_BTN3)  |
                   GPIO_OUT(PIN_LED_Y) | GPIO_OUT(PIN_LED_B) |
                   GPIO_OUT(PIN_LED_R) | GPIO_OUT(PIN_LED_G) |
                   GPIO_INP(RX_PIN)    | GPIO_OUT(LINEOK_PIN);

    GPIOA->OTYPER = GPIO_ODN(LINEOK_PIN);
    GPIOA->OSPEEDR = 0x00U;
    GPIOA->PUPDR = GPIO_PUP(PIN_BTN0) | GPIO_PUP(PIN_BTN1) |
                   GPIO_PUP(PIN_BTN2) | GPIO_PUP(PIN_BTN3) |
                   GPIO_PUP(LINEOK_PIN);
    GPIOA->AFR[0] = 0x00U;
    GPIOA->AFR[1] = 0x00U;
}

/**
 * @brief Set the up GPIOB
 * This will setup TX_PIN as TIM3CH4 output
 */
static void setup_gpiob(void) {
    _BST(RCC->AHBENR, RCC_AHBENR_GPIOBEN);
    GPIOB->ODR = 0x00U;
    GPIOB->MODER = GPIO_AFN(TX_PIN);
    GPIOB->OTYPER = 0x00U;
    GPIOB->OSPEEDR = 0x00U;
    GPIOB->PUPDR = 0x00U;
    GPIOB->AFR[0] = GPIO_AF(TX_PIN, 1);
}

static void setup_systick(void) {
    SysTick->LOAD = 6000U;
    SysTick->VAL = 0x00U;
    SysTick->CTRL = 0x03U;
//    NVIC_SetPriority(SysTick_IRQn, 5);
//    NVIC_EnableIRQ(SysTick_IRQn);
}


static uint32_t fnv1a32_turn (uint32_t fnv, uint32_t data ) {
    for (int i = 0; i < 4 ; i++) {
        fnv ^= (data & 0xFF);
        fnv *= 16777619;
        data >>= 8;
    }
    return fnv;
}

void HardFault_Handler(void) {
/* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
        ebus_led_on();
        delay_us(300000UL);
        ebus_led_off();
        delay_us(300000UL);
    }
}

void SysTick_Handler(void) {
    if (heartbeat_count != 0) {
        heartbeat_count--;
    }
}

/// @brief  Setup the microcontroller system.
///         Initialize the Embedded Flash Interface, the PLL and update the
///         SystemCoreClock variable.
void SystemInit(void) {
    /* Set HSION bit */
    RCC->CR |= (uint32_t)0x00000001;

    /* Reset SW[1:0], HPRE[3:0], PPRE[2:0], ADCPRE and MCOSEL[3:0] bits
     * MCOPRE[2:0] */
    RCC->CFGR &= (uint32_t)0x80FFB80C;

    /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= (uint32_t)0xFEF6FFFF;

    /* Reset HSEBYP bit */
    RCC->CR &= (uint32_t)0xFFFBFFFF;

    /* Reset PLLSRC, PLLXTPRE and PLLMUL[3:0] bits */
    RCC->CFGR &= (uint32_t)0xFFC07FFF;

    /* Reset PREDIV1[3:0] bits */
    RCC->CFGR2 &= (uint32_t)0xFFFFFFF0;

    /* Reset USARTSW[1:0], I2CSW, CECSW and ADCSW bits */
    RCC->CFGR3 &= (uint32_t)0xFFFFFEAC;

    /* Reset HSI14 & HSI48 bit */
    RCC->CR2 &= (uint32_t)0xFFFEFFFE;

    /* Disable all interrupts */
    RCC->CIR = 0x00000000;
    NVIC->ICER[0] = 0xFFFFFFFFUL;
    NVIC->ICPR[0] = 0xFFFFFFFFUL;
    /* Configure the System clock frequency, AHB/APBx prescalers and Flash
     * settings */
    setup_sysclk();
    setup_gpioa();
    setup_gpiob();
    setup_systick();
}

void delay_us(uint32_t us) {
    while(us--) {
        for (int i = 0; i < 4; i++) {
            _ASM (
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
            );
        }
    }
}

void system_reset(void) {
    __disable_irq();
    NVIC_SystemReset();
    while(1);
}

void ebus_led_on(void) {
    GPIOA->BSRR = _BV(PIN_LED_G);
}

void ebus_led_off(void) {
    GPIOA->BRR = _BV(PIN_LED_G);
}

void led_r_on(void) {
    GPIOA->BSRR = _BV(PIN_LED_R);
}

void led_r_off(void) {
    GPIOA->BRR = _BV(PIN_LED_R);
}

void led_g_on(void) {
    GPIOA->BSRR = _BV(PIN_LED_G);
}

void led_g_off(void) {
    GPIOA->BRR = _BV(PIN_LED_G);
}

void led_b_on(void) {
    GPIOA->BSRR = _BV(PIN_LED_B);
}

void led_b_off(void) {
    GPIOA->BRR = _BV(PIN_LED_B);
}

void led_y_on(void) {
    GPIOA->BSRR = _BV(PIN_LED_Y);
}

void led_y_off(void) {
    GPIOA->BRR = _BV(PIN_LED_Y);
}

uint32_t ebus_uid(void) {
    uint32_t fnv = 2166136261;
    fnv = fnv1a32_turn(fnv, *(uint32_t*)(UID_BASE + 0x00));
    fnv = fnv1a32_turn(fnv, *(uint32_t*)(UID_BASE + 0x04));
    return fnv1a32_turn(fnv, *(uint32_t*)(UID_BASE + 0x14));
}

uint8_t buttons_state(void) {
    uint8_t res;
    res =  (GPIOA->IDR & _BV(PIN_BTN0)) ? 0x00U : 0x01U;
    res |= (GPIOA->IDR & _BV(PIN_BTN1)) ? 0x00U : 0x02U;
    res |= (GPIOA->IDR & _BV(PIN_BTN2)) ? 0x00U : 0x04U;
    res |= (GPIOA->IDR & _BV(PIN_BTN3)) ? 0x00U : 0x08U;
    return res;
}