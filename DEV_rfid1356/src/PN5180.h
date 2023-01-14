#ifndef PN5180_H
#define PN5180_H

#include "stm32.h"
#include <string.h>
#include <stdbool.h>

// PN5180 1-Byte Direct Commands
// see 11.4.3.3 Host Interface Command List
#define PN5180_WRITE_REGISTER               (0x00)
#define PN5180_WRITE_REGISTER_OR_MASK       (0x01)
#define PN5180_WRITE_REGISTER_AND_MASK      (0x02)
#define PN5180_READ_REGISTER                (0x04)
#define PN5180_READ_EEPROM                  (0x07)
#define PN5180_SEND_DATA                    (0x09)
#define PN5180_READ_DATA                    (0x0A)
#define PN5180_SWITCH_MODE                  (0x0B)
#define PN5180_LOAD_RF_CONFIG               (0x11)
#define PN5180_RF_ON                        (0x16)
#define PN5180_RF_OFF                       (0x17)
#define PN5180_MIFARE_AUTHENTICATE          (0X0C)

// PN5180 Registers
#define SYSTEM_CONFIG                       (0x00)
#define IRQ_ENABLE                          (0x01)
#define IRQ_STATUS                          (0x02)
#define IRQ_CLEAR                           (0x03)
#define TRANSCEIVE_CONTROL                  (0x04)
#define TIMER1_RELOAD                       (0x0c)
#define TIMER1_CONFIG                       (0x0f)
#define RX_WAIT_CONFIG                      (0x11)
#define CRC_RX_CONFIG                       (0x12)
#define RX_STATUS                           (0x13)
#define TX_WAIT_CONFIG                      (0x17)
#define TX_CONFIG                           (0x18)
#define CRC_TX_CONFIG                       (0x19)
#define RF_STATUS                           (0x1d)
#define SYSTEM_STATUS                       (0x24)
#define TEMP_CONTROL                        (0x25)
#define AGC_REF_CONFIG                      (0x26)

// PN5180 EEPROM Addresses
#define DIE_IDENTIFIER                      (0x00)
#define PRODUCT_VERSION                     (0x10)
#define FIRMWARE_VERSION                    (0x12)
#define EEPROM_VERSION                      (0x14)
#define IRQ_PIN_CONFIG                      (0x1A)

#define LOW                                 0
#define HIGH                                1

#define GPIO_Pin_1                          ((uint16_t)0x0002) 
#define GPIO_Pin_2                          ((uint16_t)0x0004) 
#define GPIO_Pin_3                          ((uint16_t)0x0008)

#define PN5180_BUSY                         GPIO_Pin_1			// линия занято SPI PN5180  
#define status_PN5180_BUSY                  ((GPIOA->IDR & PN5180_BUSY)>>1)
#define PN5180_RST                          GPIO_Pin_2			// линия сброса PN5180  
#define PN5180_RSTON                        (GPIOA->BSRR|= PN5180_RST)
#define PN5180_RSTOFF                       (GPIOA->BRR|= PN5180_RST)
#define PN5180_NSS                          GPIO_Pin_3			// линия NSS SPI PN5180
#define PN5180_NSSON                        (GPIOA->BSRR|= PN5180_NSS)
#define PN5180_NSSOFF                       (GPIOA->BRR|= PN5180_NSS)

#define chip_BUSY                           1
#define chip_RST                            2
#define chip_NSS                            3
#define chip_SCK                            5
#define chip_MISO                           6
#define chip_MOSI                           7

#define pause                               5
#define  NFCA                               1
#define  NFCB                               2
#define  NFCF                               3

/*
 * ATQ codes
 */
#define atqa_ul                             0x4400
#define atqa_ulc                            0x4400
#define atqa_mfc                            0x0200
#define atqa_mfp_s                          0x0400
#define atqa_mfp_x                          0x4200
#define atqa_desfire                        0x4403
#define atqa_jcop                           0x0400
#define atqa_mini                           0x0400
#define atqa_nPA                            0x0800

typedef struct {
    uint16_t mem_size;
    uint8_t typeNFC;
    uint8_t standart;
    uint8_t UID[7];
    uint8_t len_UID;
    uint8_t SAK;
    uint16_t ATQA;
    uint16_t SC;
    uint8_t PMm[8];
}struct_NFCteg;

enum NFC_standart{
    MIFARE_Ultralight,
    MIFARE_Ultralight_C,
    MIFARE_Mini,
    MIFARE_Mini_CL2,
    MIFARE_Classic,
    MIFARE_Classic_CL2,
    MIFARE_Plus_CL2,
    MIFARE_Plus,
    MIFARE_Plus_S,
    MIFARE_Plus_X,
    MIFARE_DESFire,
    MIFARE_14443_4,
    Sony_Felica_LiteS,
    Sony_Felica_plug,
    Sony_Felica_Host,
    Sony_Felica_CommonArea
};

enum PN5180TransceiveStat{
    PN5180_TS_Idle = 0,
    PN5180_TS_WaitTransmit = 1,
    PN5180_TS_Transmitting = 2,
    PN5180_TS_WaitReceive = 3,
    PN5180_TS_WaitForData = 4,
    PN5180_TS_Receiving = 5,
    PN5180_TS_LoopBack = 6,
    PN5180_TS_RESERVED = 7
};

// PN5180 IRQ_STATUS
#define RX_IRQ_STAT                         (1<<0)  // End of RF rececption IRQ
#define TX_IRQ_STAT                         (1<<1)  // End of RF transmission IRQ
#define IDLE_IRQ_STAT                       (1<<2)  // IDLE IRQ
#define RFOFF_DET_IRQ_STAT                  (1<<6)  // RF Field OFF detection IRQ
#define RFON_DET_IRQ_STAT                   (1<<7)  // RF Field ON detection IRQ
#define TX_RFOFF_IRQ_STAT                   (1<<8)  // RF Field OFF in PCD IRQ
#define TX_RFON_IRQ_STAT                    (1<<9)  // RF Field ON in PCD IRQ
#define RX_SOF_DET_IRQ_STAT                 (1<<14) // RF SOF Detection IRQ
#define GENERAL_ERROR_IRQ_STAT              (1<<17) // General error IRQ
#define LPCD_IRQ_STAT                       (1<<19) // LPCD Detection IRQ

//#define DebugMSG
//  SPISettings PN5180_SPI_SETTINGS;

void showIRQStatus(uint32_t st);
void showMSG_withDATE(char* txt, uint8_t* dim, uint8_t len);

//  PN5180(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin);

void PN5180_begin(void);
void PN5180_end(void);
bool PN5180transceiveCommand(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen);
  /*
   * PN5180 direct commands with host interface
   */

  /* cmd 0x00 */
bool PN5180writeRegister(uint8_t reg, uint32_t value);
  /* cmd 0x01 */
bool PN5180writeRegisterWithOrMask(uint8_t addr, uint32_t mask);
  /* cmd 0x02 */
bool PN5180writeRegisterWithAndMask(uint8_t addr, uint32_t mask);

  /* cmd 0x04 */
//  bool PN5180readRegister(uint8_t reg, uint32_t *value);
bool PN5180read__Register(uint8_t reg, uint8_t *value);

  /* cmd 0x07 */
bool PN5180readEEprom(uint8_t addr, uint8_t *buffer, uint32_t len);

  /* cmd 0x09 */
bool PN5180sendData(uint8_t *data, uint32_t len, uint8_t validBits);
  /* cmd 0x0a */
uint8_t * PN5180readData(uint32_t len);
bool PN5180_readData(uint16_t len, uint8_t *buffer);
	  /* cmd 0x0B */
bool PN5180switchToLPCD(uint16_t wakeupCounterInMs);  
  /* cmd 0x11 */
bool PN5180loadRFConfig(uint8_t txConf, uint8_t rxConf);
  /* cmd 0x16 */
bool PN5180setRF_on(void);
  /* cmd 0x17 */
bool PN5180setRF_off(void);
uint32_t PN5180getIRQStatus(void);
//  bool setRF_on();

uint16_t rxBytesReceived(void);
void PN5180reset(void);
bool clearIRQStatus(uint32_t irqMask);
bool clearIRQStatus(uint32_t irqMask);
enum PN5180TransceiveStat PN5180getTransceiveState(void);
bool PN5180transceiveCommand(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen);
void init_structNFC(struct_NFCteg strNFC); 
void init_1356(void);
bool felica_setupRF(void);
bool mifare_setupRF(void);
uint8_t felica_readCardSerial(uint8_t *buffer);
uint8_t mifare_readCardSerial(uint8_t *buffer);
void read_tag(uint8_t* buf);

#endif /* PN5180_H */
