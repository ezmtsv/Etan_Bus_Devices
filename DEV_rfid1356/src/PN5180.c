#include "PN5180.h"
#include "platform.h"

uint8_t buf_cmdNFC[10];
uint16_t buffer_SPI[64];
uint16_t N_bait_spi;
uint8_t Send_Buffer[64];
struct_NFCteg cur_NFCteg;
uint8_t in_buf_NFC[64];
uint8_t outbuf_NFC[64];
static uint8_t readBuffer[508];

void delay_us(uint32_t us);
void send_dim_SPI(uint8_t* dim, uint16_t len);
//? void send_str_USB(char *s, uint8_t count);

void DelaymS(uint32_t us) {
    delay_us(us*800);
}

void send_dataSPI(uint8_t Data) {
    while((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE){}
    *(uint8_t *)&(SPI1->DR) = Data;
}

void send_dim_SPI(uint8_t* dim, uint16_t len){
    uint16_t i;
    for( i = 0; i<len; i++){
        send_dataSPI(dim[i]);
    }
}

uint8_t byte_asc2(uint8_t symb, char* dim_asc) {			    // преобразование 8 битного числа в код asc2 (от 1 до 3 байт в dim_asc), возвращает кол-во символов числа + после каждого числа добавляется ':' 	
        uint8_t t; 
        uint8_t cnt = 0;
        if(symb/100 != 0){                              /// трехзначное число
            dim_asc[0] = symb/100+'0'; 
                t = symb%100; dim_asc[1] = t/10+'0';
                dim_asc[2] = t%10+'0';
                dim_asc[3] = ':';
                cnt = 4;
        }
        else {
            if(symb/10 != 0) {                          /// двухзначное число
                dim_asc[0] = symb/10+'0';
                dim_asc[1] = symb%10+'0';
                dim_asc[2] = ':';
                cnt = 3;
            }
            else {                                      /// однозначное число
                dim_asc[0] = symb+'0';
                dim_asc[1] = ':';
                cnt = 2;
            }
        }
        return cnt;
}

void PN5180_begin(void) {
    _BST(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
    GPIOA->MODER |= GPIO_OUT(chip_RST) | GPIO_INP(chip_BUSY) | GPIO_OUT(chip_NSS);
    GPIOA->OSPEEDR |= 0xF0U;
    
    PN5180_NSSON; PN5180_RSTON;
}

void PN5180_end(void) {
/*  digitalWrite(PN5180_NSS, HIGH); // disable
  SPI.end();*/
	PN5180_NSSON; 
}

/*
 * WRITE_REGISTER - 0x00
 * This command is used to write a 32-bit value (little endian) to a configuration register.
 * The address of the register must exist. If the condition is not fulfilled, an exception is
 * raised.
 */
//////////////////////////////
void showIRQStatus(uint32_t st){
    uint16_t count_symb;
    char simb_dim[64];
    char simb_asc[14];
    uint8_t count_char;
    uint16_t k = 0;
    char* txt = "\nСтатус IRQ: ";

    memset(simb_dim,0x00,63);
    count_symb = strlen(txt);
    for (uint8_t i = 0; i<count_symb; i++){ simb_dim[i] = txt[i]; }
    for (uint8_t i = 0; i<4; i++) { 
        count_char = byte_asc2((uint8_t)(st>>(24-i*8)), simb_asc);
        for(uint8_t j = 0; j<count_char; j++) { 
            simb_dim[count_symb+k] = simb_asc[j]; k++;
        } 
        if((count_symb+k)>62)break;	
    }
    simb_dim[count_symb+k] = '\0';

//?	send_str_USB(simb_dim,0); DelaymS(pause); memset(Send_Buffer,0x00,63);
}
//// выводит в отладчик USB сообщение txt и массив dim длиной len. Всего должно быть не более 256 символов
///// //// больше 16 элементов dim не передавать, если планируется дальнейший просмотр в HEX
void showMSG_withDATE(char* txt, uint8_t* dim, uint8_t len){
    uint16_t count_symb;
    char simb_dim[256];
    char simb_asc[14];
    uint8_t count_char;
    uint16_t k = 0;

    memset(simb_dim,0x00,63);
    count_symb = strlen(txt);
    for (uint8_t i = 0; i<count_symb; i++){ simb_dim[i] = txt[i]; }
    for (uint8_t i = 0; i<len; i++) { 
        count_char = byte_asc2(dim[i], simb_asc);
        for(uint8_t j = 0; j<count_char; j++) { 
            simb_dim[count_symb+k] = simb_asc[j]; k++;
        } 
        if((count_symb+k)>254)break;						/// формируем строку до 256 байт
    }
    simb_dim[count_symb+k] = '\0';
//	DelaymS(pause);
//?	send_str_USB(simb_dim,0); DelaymS(pause); memset(Send_Buffer,0x00,63);
}

uint16_t rxBytesReceived(void) {
    uint32_t rxStatus;
    uint8_t dimStat[4];
    uint16_t len = 0;
    PN5180read__Register(RX_STATUS, dimStat);
    rxStatus = (((uint32_t)dimStat[1])<<8) | ((uint32_t)dimStat[0]);
#ifdef debug_READcmd
    showMSG_withDATE("\nrxBytesReceived  ", (uint8_t*)&rxStatus, 2);
#endif	
	// Lower 9 bits has length
    len = rxStatus & 0x000001ff;
    return len;
}
////////////////////////////////
bool PN5180transceiveCommand(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen) {
    uint16_t pause_while = 200;
    uint16_t count_while = 0;
    uint16_t waitcount_while = 30;
    uint16_t koef_delay = 4;
    uint8_t BUF_cmdREC[64];
	
#ifdef DebugMSG
//	Send_Buffer[62] = 0; Send_Buffer[63] = 0;
	showMSG_withDATE("\nОтправка команды SPI: ", sendBuffer, sendBufferLen);
//	Send_Buffer[62] = 5; Send_Buffer[63] = 7;
#endif
  // 0.
    while (LOW != status_PN5180_BUSY){ count_while++; delay_us(pause_while); if(count_while>waitcount_while){ return false; }}; // wait until busy is low
  // 1.
    PN5180_NSSOFF; delay_us(koef_delay*300); // DelaymS(2); /// 
  // 2.
    send_dim_SPI(sendBuffer, sendBufferLen); delay_us(koef_delay*sendBufferLen); //delay_us(2*sendBufferLen);
  // 3.
    count_while = 0;
    while(HIGH != status_PN5180_BUSY){ count_while++; delay_us(pause_while); if(count_while>waitcount_while){ return false; } };  // wait until BUSY is high
  // 4.
    PN5180_NSSON; DelaymS(1); /// delay_us(koef_delay*10); // 
  // 5.
    count_while = 0;
    while (LOW != status_PN5180_BUSY){ count_while++; delay_us(pause_while); if(count_while>waitcount_while){ return false; }}; // wait unitl BUSY is low
  // check, if write-only
    if ((0 == recvBuffer) || (0 == recvBufferLen)) return true;
  // 1.
    PN5180_NSSOFF; delay_us(koef_delay*20); // DelaymS(2); /// 
  // 2.
    N_bait_spi = 0;
    for (uint8_t i=0; i<recvBufferLen; i++) {
        BUF_cmdREC[i] = 0xff;
    }
    send_dim_SPI(BUF_cmdREC, recvBufferLen); delay_us(koef_delay*recvBufferLen);
    for (uint8_t i=0; i<recvBufferLen; i++) {
        recvBuffer[i] = buffer_SPI[i];
    }	
  // 3.
    count_while = 0;
    while(HIGH != status_PN5180_BUSY){ count_while++; delay_us(pause_while); if(count_while>waitcount_while){ return false; }};  // wait until BUSY is high
  // 4.
    PN5180_NSSON; DelaymS(1); /// delay_us(koef_delay*10); // 
  // 5.
    count_while = 0;
    while(LOW != status_PN5180_BUSY){ count_while++; delay_us(pause_while); if(count_while>waitcount_while){ return false; } };  // wait until BUSY is low
#ifdef DebugMSG
    showMSG_withDATE("\nПринято по SPI: ", recvBuffer, recvBufferLen);
#endif
    PN5180_NSSON;
    return true;
}

enum PN5180TransceiveStat PN5180getTransceiveState(void) {
//  PN5180DEBUG(F("Get Transceive state...\n"));
    enum PN5180TransceiveStat tmpstat;
    uint32_t rfStatus;
    uint8_t dim_ST[4];
	if (!PN5180read__Register(RF_STATUS, dim_ST)) {	
#ifdef DebugMSG
    showIRQStatus(PN5180getIRQStatus());
#endif
//   PN5180DEBUG(F("ERROR reading RF_STATUS register.\n"));
//   return PN5180TransceiveStat(0);
    tmpstat = PN5180_TS_Idle;
    return tmpstat;
  }
  /*
   * TRANSCEIVE_STATEs:
   *  0 - idle
   *  1 - wait transmit
   *  2 - transmitting
   *  3 - wait receive
   *  4 - wait for data
   *  5 - receiving
   *  6 - loopback
   *  7 - reserved
   */
    rfStatus = ((uint32_t)dim_ST[0]) | (((uint32_t)dim_ST[1])<<8) | (((uint32_t)dim_ST[2])<<16) | (((uint32_t)dim_ST[3])<<24);
    uint8_t state = ((rfStatus >> 24) & 0x07);
/*  
  PN5180DEBUG(F("TRANSCEIVE_STATE=0x"));
  PN5180DEBUG(formatHex(state));
  PN5180DEBUG("\n");
*/
    switch(state) {
        case 0:
            tmpstat = PN5180_TS_Idle;
        break;
        case 1:
            tmpstat = PN5180_TS_WaitTransmit;
        break;
        case 2:
            tmpstat = PN5180_TS_Transmitting;
        break;
        case 3:
            tmpstat = PN5180_TS_WaitReceive;
        break;
        case 4:
            tmpstat = PN5180_TS_WaitForData;
        break;
        case 5:
            tmpstat = PN5180_TS_Receiving;
        break;
        case 6:
            tmpstat = PN5180_TS_LoopBack;
        break;
        case 7:
            tmpstat = PN5180_TS_RESERVED;
        break;		
	}
    return tmpstat;
}

bool PN5180writeRegister(uint8_t reg, uint32_t value) {
    uint8_t *p = (uint8_t*)&value;
#ifdef DebugMSG
    showMSG_withDATE("\nWrite Register 0x", p, 4);
#endif
  /*
  For all 4 byte command parameter transfers (e.g. register values), the payload
  parameters passed follow the little endian approach (Least Significant Byte first).
   */
    uint8_t buf[6] = { PN5180_WRITE_REGISTER, reg, p[0], p[1], p[2], p[3] };
//  SPI.beginTransaction(PN5180_SPI_SETTINGS);
//  transceiveCommand(buf, 6);
//		send_dim_SPI(buf, 6);
    PN5180transceiveCommand(buf, 6,0,0);
//  SPI.endTransaction();
    return true;
}

/*
 * WRITE_REGISTER_OR_MASK - 0x01
 * This command modifies the content of a register using a logical OR operation. The
 * content of the register is read and a logical OR operation is performed with the provided
 * mask. The modified content is written back to the register.
 * The address of the register must exist. If the condition is not fulfilled, an exception is
 * raised.
 */
bool PN5180writeRegisterWithOrMask(uint8_t reg, uint32_t mask) {
    uint8_t *p = (uint8_t*)&mask;
#ifdef DebugMSG
    showMSG_withDATE("\nWrite OR_MASK: ", p, 4);	
#endif
    uint8_t buf[6] = { PN5180_WRITE_REGISTER_OR_MASK, reg, p[0], p[1], p[2], p[3] };
//  SPI.beginTransaction(PN5180_SPI_SETTINGS);
//  transceiveCommand(buf, 6);
//  SPI.endTransaction();
//	send_dim_SPI(buf, 6);
    PN5180transceiveCommand(buf, 6,0,0);
    return true;
}

/*
 * WRITE _REGISTER_AND_MASK - 0x02
 * This command modifies the content of a register using a logical AND operation. The
 * content of the register is read and a logical AND operation is performed with the provided
 * mask. The modified content is written back to the register.
 * The address of the register must exist. If the condition is not fulfilled, an exception is
 * raised.
 */
bool PN5180writeRegisterWithAndMask(uint8_t reg, uint32_t mask) {
    uint8_t *p = (uint8_t*)&mask;
#ifdef DebugMSG
    showMSG_withDATE("\nWrite AND_MASK: ", p, 4);		
#endif
    uint8_t buf[6] = { PN5180_WRITE_REGISTER_AND_MASK, reg, p[0], p[1], p[2], p[3] };
//  SPI.beginTransaction(PN5180_SPI_SETTINGS);
//  transceiveCommand(buf, 6);
//  SPI.endTransaction();
//	send_dim_SPI(buf, 6);
    PN5180transceiveCommand(buf, 6,0,0);
    return true;
}

/*
 * READ_REGISTER - 0x04
 * This command is used to read the content of a configuration register. The content of the
 * register is returned in the 4 byte response.
 * The address of the register must exist. If the condition is not fulfilled, an exception is
 * raised.
 */
//// возвращает значение считанное из регистра reg в массив *value, при включенном DebugMSG выдает значения в порт USB
/*
bool PN5180readRegister(uint8_t reg, uint32_t *value) {  
	#ifdef DebugMSG

	#endif
//	uint8_t cmd[2] = { PN5180_READ_REGISTER, reg };
//	PN5180transceiveCommand(cmd, 2, (uint8_t*)value, 4);
	
	buf_cmdNFC[0] = PN5180_READ_REGISTER; buf_cmdNFC[1] = reg;
	PN5180transceiveCommand(buf_cmdNFC, 2, (uint8_t*)value, 4);
	
  return true;
}
*/
bool PN5180read__Register(uint8_t reg, uint8_t *value) {  
#ifdef DebugMSG
//?		send_str_USB("\nЧтение регистра", reg); DelaymS(pause);
#endif
//	uint8_t cmd[2] = { PN5180_READ_REGISTER, reg };
//	PN5180transceiveCommand(cmd, 2, (uint8_t*)value, 4);
    buf_cmdNFC[0] = PN5180_READ_REGISTER; buf_cmdNFC[1] = reg;
    PN5180transceiveCommand(buf_cmdNFC, 2, value, 4);
    return true;
}
////////////////////////////////////
uint32_t PN5180getIRQStatus(void) {
    uint32_t irqStatus;
/*  PN5180DEBUG(F("Read IRQ-Status register...\n"));
  
  PN5180DEBUG(F("IRQ-Status=0x"));
  PN5180DEBUG(formatHex(irqStatus));
  PN5180DEBUG("\n");
*/
    uint8_t dim_STat[4];
//	PN5180readRegister(IRQ_STATUS, &irqStatus);
    PN5180read__Register(IRQ_STATUS, dim_STat);
    irqStatus = ((uint32_t)dim_STat[0]) | (((uint32_t)dim_STat[1])<<8) | (((uint32_t)dim_STat[2])<<16) | (((uint32_t)dim_STat[3])<<24);
    return irqStatus;
}

bool clearIRQStatus(uint32_t irqMask) {
/*  PN5180DEBUG(F("Clear IRQ-Status with mask=x"));
  PN5180DEBUG(formatHex(irqMask));
  PN5180DEBUG("\n");
*/
    return PN5180writeRegister(IRQ_CLEAR, irqMask);
}
////////////////////////////////////////
/*
 * READ_EEPROM - 0x07
 * This command is used to read data from EEPROM memory area. The field 'Address'
 * indicates the start address of the read operation. The field Length indicates the number
 * of bytes to read. The response contains the data read from EEPROM (content of the
 * EEPROM); The data is read in sequentially increasing order starting with the given
 * address.
 * EEPROM Address must be in the range from 0 to 254, inclusive. Read operation must
 * not go beyond EEPROM address 254. If the condition is not fulfilled, an exception is
 * raised.
 */
bool PN5180readEEprom(uint8_t addr, uint8_t *buffer, uint32_t len) {
    if ((addr > 254) || ((addr+len) > 254)) {
//    PN5180DEBUG(F("ERROR: Reading beyond addr 254!\n"));
        return false;
    }
/*
  PN5180DEBUG(F("Reading EEPROM at 0x"));
  PN5180DEBUG(formatHex(addr));
  PN5180DEBUG(F(", size="));
  PN5180DEBUG(len);
  PN5180DEBUG(F("...\n"));
  uint8_t cmd[3] = { PN5180_READ_EEPROM, addr, len };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(cmd, 3, buffer, len);
  SPI.endTransaction();
*/
#ifdef DEBUG
  PN5180DEBUG(F("EEPROM values: "));
  for (int i=0; i<len; i++) {
    PN5180DEBUG(formatHex(buffer[i]));
    PN5180DEBUG(" ");
  }
  PN5180DEBUG("\n");
#endif

//	uint8_t cmd[3] = { PN5180_READ_EEPROM, addr, len };
//	PN5180transceiveCommand(cmd, 3, buffer, len);
    buf_cmdNFC[0] = PN5180_READ_EEPROM; buf_cmdNFC[1] =  addr; buf_cmdNFC[2] =  len;
    PN5180transceiveCommand(buf_cmdNFC, 3, buffer, len);	
    return true;
}

/*
 * SEND_DATA - 0x09
 * This command writes data to the RF transmission buffer and starts the RF transmission.
 * The parameter ‘Number of valid bits in last Byte’ indicates the exact number of bits to be
 * transmitted for the last byte (for non-byte aligned frames).
 * Precondition: Host shall configure the Transceiver by setting the register
 * SYSTEM_CONFIG.COMMAND to 0x3 before using the SEND_DATA command, as
 * the command SEND_DATA is only writing data to the transmission buffer and starts the
 * transmission but does not perform any configuration.
 * The size of ‘Tx Data’ field must be in the range from 0 to 260, inclusive (the 0 byte length
 * allows a symbol only transmission when the TX_DATA_ENABLE is cleared).‘Number of
 * valid bits in last Byte’ field must be in the range from 0 to 7. The command must not be
 * called during an ongoing RF transmission. Transceiver must be in ‘WaitTransmit’ state
 * with ‘Transceive’ command set. If the condition is not fulfilled, an exception is raised.
 */
bool PN5180sendData(uint8_t *data, uint32_t len, uint8_t validBits) {
    if (len > 260) {
//    PN5180DEBUG(F("ERROR: sendData with more than 260 bytes is not supported!\n"));
        return false;
    }
#ifdef DEBUG
    PN5180DEBUG(F("Send data (len="));
    PN5180DEBUG(len);
    PN5180DEBUG(F("):"));
    for (int i=0; i<len; i++) {
        PN5180DEBUG(" ");
        PN5180DEBUG(formatHex(data[i]));
    }
    PN5180DEBUG("\n");
#endif
//  uint8_t buffer[len+2];
    uint8_t buffer[508];
    buffer[0] = PN5180_SEND_DATA;
    buffer[1] = validBits; // number of valid bits of last byte are transmitted (0 = all bits are transmitted)
    for (int i=0; i<len; i++) {
        buffer[2+i] = data[i];
    }
    PN5180writeRegisterWithAndMask(SYSTEM_CONFIG, 0xfffffff8);  // Idle/StopCom Command
    PN5180writeRegisterWithOrMask(SYSTEM_CONFIG, 0x00000003);   // Transceive Command
  /*
   * Transceive command; initiates a transceive cycle.
   * Note: Depending on the value of the Initiator bit, a
   * transmission is started or the receiver is enabled
   * Note: The transceive command does not finish
   * automatically. It stays in the transceive cycle until
   * stopped via the IDLE/StopCom command
   */
/*
  PN5180TransceiveStat transceiveState = getTransceiveState();
  if (PN5180_TS_WaitTransmit != transceiveState) {
    PN5180DEBUG(F("*** ERROR: Transceiver not in state WaitTransmit!?\n"));
    return false;
  }

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(buffer, len+2);
  SPI.endTransaction();
*/
    enum PN5180TransceiveStat transceiveState = PN5180getTransceiveState();
    if(PN5180_TS_WaitTransmit != transceiveState) {
        return false;
    }	
    PN5180transceiveCommand(buffer, (len+2), 0,0);
    return true;
}

/*
 * READ_DATA - 0x0A
 * This command reads data from the RF reception buffer, after a successful reception.
 * The RX_STATUS register contains the information to verify if the reception had been
 * successful. The data is available within the response of the command. The host controls
 * the number of bytes to be read via the SPI interface.
 * The RF data had been successfully received. In case the instruction is executed without
 * preceding an RF data reception, no exception is raised but the data read back from the
 * reception buffer is invalid. If the condition is not fulfilled, an exception is raised.
 */
uint8_t * PN5180readData(uint32_t len) {
    
    if (len > 508) {
        return 0L;
    }
    buf_cmdNFC[0] = PN5180_READ_DATA; buf_cmdNFC[1] = 0; 
    PN5180transceiveCommand(buf_cmdNFC, 2, readBuffer, len);
    return readBuffer;
}
bool PN5180_readData(uint16_t len, uint8_t *buffer) {
	if (len > 508) {
		return false;
	}
	uint8_t cmd[2] = { PN5180_READ_DATA, 0x00 };
	bool success = PN5180transceiveCommand(cmd, 2, buffer, len);
	return success;
}
/*
 * LOAD_RF_CONFIG - 0x11
 * Parameter 'Transmitter Configuration' must be in the range from 0x0 - 0x1C, inclusive. If
 * the transmitter parameter is 0xFF, transmitter configuration is not changed.
 * Field 'Receiver Configuration' must be in the range from 0x80 - 0x9C, inclusive. If the
 * receiver parameter is 0xFF, the receiver configuration is not changed. If the condition is
 * not fulfilled, an exception is raised.
 * The transmitter and receiver configuration shall always be configured for the same
 * transmission/reception speed. No error is returned in case this condition is not taken into
 * account.
 *
 * Transmitter: RF   Protocol          Speed     Receiver: RF    Protocol    Speed
 * configuration                       (kbit/s)  configuration               (kbit/s)
 * byte (hex)                                    byte (hex)
 * ----------------------------------------------------------------------------------------------
 * ->0D              ISO 15693 ASK100  26        8D              ISO 15693   26
 *   0E              ISO 15693 ASK10   26        8E              ISO 15693   53
 */
bool PN5180loadRFConfig(uint8_t txConf, uint8_t rxConf) {
    uint8_t cmd[3] = { PN5180_LOAD_RF_CONFIG, txConf, rxConf };
    PN5180transceiveCommand(cmd, 3, 0,0);
    return true;
}

/*
 * RF_ON - 0x16
 * This command is used to switch on the internal RF field. If enabled the TX_RFON_IRQ is
 * set after the field is switched on.
 */

bool PN5180setRF_on(void) {
    uint8_t cmd[2] = { PN5180_RF_ON, 0x00 };
    PN5180transceiveCommand(cmd, 2, 0,0);
    return true;
}

/*
 * RF_OFF - 0x17
 * This command is used to switch off the internal RF field. If enabled, the TX_RFOFF_IRQ
 * is set after the field is switched off.
 */

bool PN5180setRF_off(void) {
    uint16_t count_while = 0;
    uint16_t pause_while = 200;
    uint8_t cmd[2] = { PN5180_RF_OFF, 0x00 };
    PN5180transceiveCommand(cmd, 2, 0,0);
    while (0 == (TX_RFOFF_IRQ_STAT & PN5180getIRQStatus())){ count_while++; delay_us(pause_while); if(count_while>10)break; }; // wait for RF field to shut down
    return true;
}

//---------------------------------------------------------------------------------------------

/*
11.4.3.1 A Host Interface Command consists of either 1 or 2 SPI frames depending whether the
host wants to write or read data from the PN5180. An SPI Frame consists of multiple
bytes.

All commands are packed into one SPI Frame. An SPI Frame consists of multiple bytes.
No NSS toggles allowed during sending of an SPI frame.

For all 4 byte command parameter transfers (e.g. register values), the payload
parameters passed follow the little endian approach (Least Significant Byte first).

Direct Instructions are built of a command code (1 Byte) and the instruction parameters
(max. 260 bytes). The actual payload size depends on the instruction used.
Responses to direct instructions contain only a payload field (no header).
All instructions are bound to conditions. If at least one of the conditions is not fulfilled, an exception is
raised. In case of an exception, the IRQ line of PN5180 is asserted and corresponding interrupt
status register contain information on the exception.
*/

/*
 * A Host Interface Command consists of either 1 or 2 SPI frames depending whether the
 * host wants to write or read data from the PN5180. An SPI Frame consists of multiple
 * bytes.
 * All commands are packed into one SPI Frame. An SPI Frame consists of multiple bytes.
 * No NSS toggles allowed during sending of an SPI frame.
 * For all 4 byte command parameter transfers (e.g. register values), the payload
 * parameters passed follow the little endian approach (Least Significant Byte first).
 * The BUSY line is used to indicate that the system is BUSY and cannot receive any data
 * from a host. Recommendation for the BUSY line handling by the host:
 * 1. Assert NSS to Low
 * 2. Perform Data Exchange
 * 3. Wait until BUSY is high
 * 4. Deassert NSS
 * 5. Wait until BUSY is low
 * If there is a parameter error, the IRQ is set to ACTIVE and a GENERAL_ERROR_IRQ is set.
 */


/*
 * Reset NFC device DelaymS
 */
void PN5180reset(void) {
    uint16_t count_while = 0;
    uint16_t pause_while = 200;
/*  digitalWrite(PN5180_RST, LOW);  // at least 10us required
  delay(10);
  digitalWrite(PN5180_RST, HIGH); // 2ms to ramp up required
  delay(10);

  while (0 == (IDLE_IRQ_STAT & getIRQStatus())); // wait for system to start up

  clearIRQStatus(0xffffffff); // clear all flags
	*/
#ifdef DebugMSG
//?	send_str_USB("\nСброс PN5180", 0); DelaymS(pause);
#endif	
    PN5180_RSTOFF;  // at least 10us required
    DelaymS(10);
    PN5180_RSTON; // 2ms to ramp up required
    DelaymS(10);
    while (0 == (IDLE_IRQ_STAT & PN5180getIRQStatus())){ count_while++; delay_us(pause_while); if(count_while>10)break; }; // wait for system to start up
    clearIRQStatus(0xffffffff); // clear all flags	
}

bool PN5180switchToLPCD(uint16_t wakeupCounterInMs) {
  // clear all IRQ flags
    clearIRQStatus(0xffffffff); 
  // enable only LPCD and general error IRQ
    PN5180writeRegister(IRQ_ENABLE, LPCD_IRQ_STAT | GENERAL_ERROR_IRQ_STAT);  
  // switch mode to LPCD 
    uint8_t cmd[4] = { PN5180_SWITCH_MODE, 0x01, (uint8_t)(wakeupCounterInMs & 0xFF), (uint8_t)((wakeupCounterInMs >> 8U) & 0xFF) };
    bool success = PN5180transceiveCommand(cmd, sizeof(cmd), 0, 0);
    return success;
}

void Configure_SPI1(void) {
    _BST(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
    GPIOA->MODER |= GPIO_AFN(chip_SCK) | GPIO_AFN(chip_MISO) | GPIO_AFN(chip_MOSI);
    GPIOA->AFR[0] |= GPIO_AF(chip_SCK, 0) | GPIO_AF(chip_MISO, 0) | GPIO_AF(chip_MOSI, 0); 
/*    
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
        
    GPIOA->MODER = (GPIOA->MODER 
                    & ~(GPIO_MODER_MODER5 | \
                    GPIO_MODER_MODER6 | GPIO_MODER_MODER7))\
                    | (GPIO_MODER_MODER5_1 |\
                    GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); // (1) //
    GPIOA->AFR[0] = (GPIOA->AFR[0] & \
                    ~(GPIO_AFRL_AFRL5 |\
                    GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7)); // (2) // 
*/
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1 ; 
    SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_RXNEIE | SPI_CR2_FRXTH | SPI_CR2_NSSP | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;
    SPI1->CR1 |= SPI_CR1_SPE;

    NVIC_SetPriority(SPI1_IRQn, 0); 
    NVIC_EnableIRQ(SPI1_IRQn); 
}

void init_structNFC(struct_NFCteg strNFC) {
    strNFC.len_UID = 0;
    strNFC.mem_size = 0;
    strNFC.SAK = 0;
    strNFC.standart = MIFARE_Ultralight;
    strNFC.ATQA = 0;
}

void init_1356(void) {
    Configure_SPI1();
    PN5180_begin();
    init_structNFC(cur_NFCteg);
    PN5180reset();
}

void SPI1_IRQHandler(void) {
    uint16_t SPI1_Data = 0;
	if((SPI1->SR & SPI_SR_RXNE) == SPI_SR_RXNE) {
        SPI1_Data = SPI1->DR;                                       // receive data, clear flag 
        buffer_SPI[N_bait_spi] = SPI1_Data;
        N_bait_spi++;
        if(N_bait_spi>63)N_bait_spi = 0;
    }
}

void read_tag(uint8_t* buf) {
    uint8_t i = mifare_readCardSerial(outbuf_NFC);
	if(i == 0){
        PN5180reset();
        felica_setupRF();
        i = felica_readCardSerial(outbuf_NFC);
        if(i == 0) {
            PN5180reset();
            mifare_setupRF();
        }
    }
    if(i != 0) {
        struct_NFCteg* p = &cur_NFCteg; 
        uint8_t* v = (uint8_t*)p;
        for(uint8_t k = 0; k < sizeof(struct_NFCteg); k++ ) {
            buf[k+4] = v[k];
        }
    }
    else {
        for(uint8_t k = 0; k < sizeof(struct_NFCteg); k++ ) { buf[k+4] = 0; }
    }
}
