#include "stm32f0xx.h"

#define PORTA                           GPIOA
#define PORTA_CLK                       RCC_AHBPeriph_GPIOA
#define LED_R                           GPIO_Pin_13
#define ADC_MC                          GPIO_Pin_0
#define STATUS_L_PLUS                   GPIO_Pin_1
#define STATUS_L_MINUS                  GPIO_Pin_2
#define button                          GPIO_Pin_3
#define line_VOL                        GPIO_Pin_3

#define EN_DRV                          GPIO_Pin_4
#define DATA_IN                         GPIO_Pin_5
#define DATA_OUT                        GPIO_Pin_6
#define FLAG_EX_POW                     GPIO_Pin_7

#define EN_DRV_ON                       (GPIOA->BRR|= EN_DRV)
#define EN_DRV_OFF                      (GPIOA->BSRR|= EN_DRV)
#define DATA_OUT_HI                     (GPIOA->BSRR|=DATA_OUT)
#define DATA_OUT_LOW                    (GPIOA->BRR|=DATA_OUT)
#define DATA_toggle                     (GPIOA->ODR ^= DATA_OUT)
#define receive_BIT                     PORTA->IDR & DATA_IN
#define status_FLAG_EX_POW              PORTA->IDR & FLAG_EX_POW
#define status_DATA_IN                  ((PORTA->IDR & DATA_IN)>>5)

#define SEL_RANGE                       GPIO_Pin_1
#define SEL_RANGE_ON                    (GPIOB->BSRR|=SEL_RANGE)
#define SEL_RANGE_OFF                   (GPIOB->BRR|=SEL_RANGE)

#define ID_contr
#ifdef ID_contr
  #define timetick   16
#else
  #define timetick   8
#endif

#define adressDEV                       0
#define long_timetick                   16
#define short_timetick                  8
#define wait_line                       3
#define midle_timetick                  12
#define cmd_ONRED                       70
#define cmd_OFFRED                      71

#define countTIM17                      (TIM17->CNT)

#define send_OK                         0
#define line_busy                       1
#define lost_priority                   2
#define line_broke                      3

#define DIV_base                        1
#define SSPULSE                         6*DIV_base	// Single space pulse, us
#define SMPULSE                         12*DIV_base	// Single mark pulse, us
#define SSSAMPLE                        23*DIV_base	// Single space sample, us
#define SMSAMPLE                        71*DIV_base	// Single mark sample, us
#define DSPULSE                         25*DIV_base	// Double space pulse, us

#define DMPULSE                         77*DIV_base	// Double mark pulse, us
#define DSSAMPLE                        44*DIV_base	// Double space sample, us
#define DMSAMPLE                        142*DIV_base	// Double mark sample, us DMPULSE
#define TSPULSE                         47*DIV_base	// Triple space pulse, us
#define	waitST_send                     (TSPULSE-DSSAMPLE)
#define	SMPULSE_1                       (SMPULSE-1)

#define reg_tim3_CR1                    0x40000400
#define reg_tim3_ARR                    0x4000042C
#define reg_tim3_CCR1                   0x40000434

#define TIMER_PRESCALER                 48000
#define TIMER_PRESCALER_tim2            48000

#define SET_BIT(REG, BIT)               ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)             ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)              ((REG) & (BIT))
#define CLEAR_REG(REG)                  ((REG) = (0x0))
#define WRITE_REG(REG, VAL)             ((REG) = (VAL))
#define READ_REG(REG)                   ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

enum{
	ADC_cur_line,
	ADC_line_HI,
	ADC_line_LOW,
	ADC_line_VOL
};

void led_one_ON(uint16_t led_);
void led_one_OFF(uint16_t led_);
void init_tmr3(void);
void init_tmr16(void);
void init_tmr17(void);
void init_GPIO_ineterrupt(void);
void duty_cyclePWM(uint16_t duty);
void init_gpio_(void);
void init_ADC_STM(void);
void init_ADC_DMA(void);
void DMA_Config(void);
