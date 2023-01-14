#ifndef _OW_H_
#define _OW_H_

#include "system.h"
#include "platform.h"
#include "stm32.h"
#include <string.h>
#include <stdbool.h>
//РУс
#define GPIO_Pin_2								((uint16_t)0x0004U)  /* Pin 2 selected    */
#define GPIO_Pin_3								((uint16_t)0x0008U)  /* Pin 3 selected    */
#define TX_PIN_OW								2
#define RX_PIN_OW								3
#define USART_StopBits_1                     	((uint32_t)0x00000000)
#define USART_WordLength_8b                  	((uint32_t)0x00000000)
#define USART_Parity_No                      	((uint32_t)0x00000000)
#define USART_Mode_Rx                        	((uint32_t)0x00000004)            /*!< Receiver Enable */
#define USART_Mode_Tx                        	((uint32_t)0x00000008)            /*!< Transmitter Enable */
#define USART_HardwareFlowControl_None       	((uint32_t)0x00000000)
#define RCC_USART1EN          					((uint32_t)0x00004000)        //!< USART1 clock enable //
#define RCC_USART2EN          					((uint32_t)0x00020000)        //!< USART2 clock enable //
#define  USART_CR3_DMRX                         ((uint32_t)0x00000040)            //!< DMA Enable Receiver //
#define  USART_CR3_DMTX                         ((uint32_t)0x00000080)            //!< DMA Enable Transmitter //
#define CR1_CLEAR_MASK            				((uint32_t)(USART_CR1_M | USART_CR1_PCE | \
                                                USART_CR1_PS | USART_CR1_TE | \
                                                USART_CR1_RE))
#define CR3_CLEAR_MASK        					((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE))	
#define RCC_AHBPeriph_DMA1                		((uint32_t)0x00000001)        /*!< DMA clock enable */

/* DMA1 Channelx interrupt pending bit masks */
#define DMA1_CHANNEL1_IT_MASK    				((uint32_t)(DMA_ISR_GIF1 | DMA_ISR_TCIF1 | DMA_ISR_HTIF1 | DMA_ISR_TEIF1))
#define DMA1_CHANNEL2_IT_MASK    				((uint32_t)(DMA_ISR_GIF2 | DMA_ISR_TCIF2 | DMA_ISR_HTIF2 | DMA_ISR_TEIF2))
#define DMA1_CHANNEL3_IT_MASK    				((uint32_t)(DMA_ISR_GIF3 | DMA_ISR_TCIF3 | DMA_ISR_HTIF3 | DMA_ISR_TEIF3))
#define DMA1_CHANNEL4_IT_MASK    				((uint32_t)(DMA_ISR_GIF4 | DMA_ISR_TCIF4 | DMA_ISR_HTIF4 | DMA_ISR_TEIF4))
#define DMA1_CHANNEL5_IT_MASK    				((uint32_t)(DMA_ISR_GIF5 | DMA_ISR_TCIF5 | DMA_ISR_HTIF5 | DMA_ISR_TEIF5))
#define CCR_CLEAR_MASK   						((uint32_t)0xFFFF800F) /* DMA Channel config registers Masks */
#define DMA_DIR_PeripheralSRC              		((uint32_t)0x00000000)
#define DMA_DIR_PeripheralDST              		DMA_CCR_DIR
#define DMA_Mode_Normal                    		((uint32_t)0x00000000)			
#define DMA_PeripheralInc_Disable          		((uint32_t)0x00000000)
#define DMA_PeripheralInc_Enable           		DMA_CCR_PINC		
#define DMA_MemoryInc_Disable              		((uint32_t)0x00000000)
#define DMA_MemoryInc_Enable               		DMA_CCR_MINC
#define DMA_PeripheralDataSize_Byte        		((uint32_t)0x00000000)
#define DMA_MemoryDataSize_Byte            		((uint32_t)0x00000000)
#define DMA_Priority_Low                   		((uint32_t)0x00000000)
#define DMA_M2M_Disable                    		((uint32_t)0x00000000)
#define DMA1_FLAG_TC5                      		((uint32_t)0x00020000)        /*!< Channel 5 Transfer Complete flag   */
//#define SYSCFG_CFGR1_USART1RX_DMA_RMP_Msk			((uint32_t)0x00000400)
//#define SYSCFG_CFGR1_USART1TX_DMA_RMP_Msk			((uint32_t)0x00000200)

#define com_SKIP								0xcc
#define com_Read_ROM  							0x33
#define com_Search_ROM 							0xf0
#define com_Match_ROM 							0x55
#define com_write1								0xD1	// for RW-1990.1
#define com_write2								0x1D	// for RW-1990.2
#define com_write3								0xD5
#define com_wrEN_1 								0xB5	// for RW-1990.1 (answer 0xFF en, 0xFE dis.)
#define com_wrEN_2 								0x1E	// for RW-1990.2 (answer 0xFE en, 0xFF dis.)

//command for DS18B20///
#define com_convert								0x44
#define RESOLUTION_9BIT 						0x1F		//31
#define RESOLUTION_10BIT 						0x3F		//63
#define RESOLUTION_11BIT 						0x5F		//95
#define RESOLUTION_12BIT 						0x7F		//127
#define com_write_RAM 							0x4e
#define com_RAMinROM 							0x48
#define com_READ_SCRATCH						0xbe

void init_OW(void);	
uint8_t status_sens(uint8_t* buffer);
uint8_t read_key(void);
uint8_t reset_OW(void) ;
void wr_byteOW(uint8_t bt);

uint8_t iButtonCRC( uint8_t *code, uint8_t numb);
void delay_ms(uint32_t us);
void delay_us(uint32_t us);

#endif

