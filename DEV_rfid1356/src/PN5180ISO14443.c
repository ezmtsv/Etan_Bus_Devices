#include "PN5180ISO14443.h"

extern uint8_t outbuf_NFC[64];
extern struct_NFCteg cur_NFCteg;
uint8_t response[20];
struct_RWTrailer struct_TR;

void DelaymS(uint32_t us);

bool mifare_setupRF(void) {
//  PN5180DEBUG(F("Loading RF-Configuration...\n"));
    if (PN5180loadRFConfig(NFCA_RFtxconf, NFCA_RFrxconf)) {             // ISO14443 parameters
        
    }
    else return false;
    if (PN5180setRF_on()) {
//    PN5180DEBUG(F("done.\n"));
    }
    else return false;
    return true;
}
/*
* buffer : must be 10 byte array
* buffer[0-1] is ATQA
* buffer[2] is sak
* buffer[3..6] is 4 byte UID
* buffer[7..9] is remaining 3 bytes of UID for 7 Byte UID tags
* kind : 0  we send REQA, 1 we send WUPA
*
* return value: the uid length:
* -	zero if no tag was recognized
* -	single Size UID (4 byte)
* -	double Size UID (7 byte)
* -	triple Size UID (10 byte) - not yet supported
*/

void search_typeStandart(uint8_t sak, uint8_t lenUID, uint8_t* uid) {
    cur_NFCteg.len_UID = lenUID;
    cur_NFCteg.typeNFC = NFCA;
    cur_NFCteg.SAK = sak;
    cur_NFCteg.ATQA = (uint16_t)uid[1] | ((uint16_t)uid[0]<<8);
    for(uint8_t i = 0; i<lenUID; i++) {
        cur_NFCteg.UID[i] = uid[i+3];
    }
    if(sak & 0x2) { ; }                             //  bit 2	
    else {
        if(sak & 0x8) {                             //  bit 4	
            if(sak & 0x10) {                        //  bit 5
                cur_NFCteg.mem_size = 4096;
                    if(lenUID == 4) cur_NFCteg.standart = MIFARE_Classic;
                    else cur_NFCteg.standart = MIFARE_Classic_CL2;
            }
            else {
                if(sak & 0x1) {                     //  bit 1	
                    cur_NFCteg.mem_size = 308;
                    if(lenUID == 4) cur_NFCteg.standart = MIFARE_Mini;
                    else cur_NFCteg.standart = MIFARE_Mini_CL2;
                }
                else {
                    cur_NFCteg.mem_size = 1024;
                    if(lenUID == 4) cur_NFCteg.standart = MIFARE_Classic;
                    else cur_NFCteg.standart = MIFARE_Classic_CL2;					
                }
            }
        }
        else {
            if(sak & 0x10) {                        // bit 5
                if(sak & 0x1) {                     // bit 1	
                    cur_NFCteg.mem_size = 4096;
                    cur_NFCteg.standart = MIFARE_Plus_CL2;	
                }
                else {
                    cur_NFCteg.mem_size = 2048;
                    cur_NFCteg.standart = MIFARE_Plus_CL2;	
                }
            }
            else {
                if(sak & 0x20) {				///// bit 6
                    switch(cur_NFCteg.ATQA) {
                        case atqa_mfp_s:
                            cur_NFCteg.mem_size = 2048;
                            cur_NFCteg.standart = MIFARE_Plus_S;										
                        break;
                        case atqa_mfc:
                            cur_NFCteg.mem_size = 4096;
                            cur_NFCteg.standart = MIFARE_Plus_S;									
                        break;
                        case atqa_mfp_x:
                            cur_NFCteg.mem_size = 4096;
                            cur_NFCteg.standart = MIFARE_Plus_X;									
                        break;
                        case atqa_desfire:
                            cur_NFCteg.mem_size = 0;
                            cur_NFCteg.standart = MIFARE_DESFire;										
                        break;
                        case atqa_ul:
                            cur_NFCteg.mem_size = 2048;
                            cur_NFCteg.standart = MIFARE_Plus;										
                        break;
                        default:
                            cur_NFCteg.mem_size = 0;
                            cur_NFCteg.standart = MIFARE_14443_4;								
                        break;								
                    }		

                }
                else {
                    cur_NFCteg.mem_size = 0;
                    cur_NFCteg.standart = MIFARE_Ultralight;						
                }						
            }				
        }
    }
}
////////// активация метки по типу NFCA
uint8_t activateTypeA(uint8_t kind) {
    uint8_t cmd[7];
    uint8_t uidLength = 0;
    uint8_t SAK;
	// Load standard TypeA protocol
    if (!PN5180loadRFConfig(NFCA_RFtxconf, NFCA_RFrxconf)) {  return 0; }
	// OFF Crypto
    if (!PN5180writeRegisterWithAndMask(SYSTEM_CONFIG, 0xFFFFFFBF))	{  return 0; }
	// Clear RX CRC
    if (!PN5180writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE))	{ return 0; }
	// Clear TX CRC
    if (!PN5180writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE))	{  return 0; }
	//Send REQA/WUPA, 7 bits in last byte
    cmd[0] = (kind == 0) ? 0x26 : 0x52;
    if (!PN5180sendData(cmd, 1, 0x07)) {  return 0; }
	// READ 2 bytes ATQA into  response
    if (!PN5180_readData(2, response)) {  return 0; }
	//Send Anti collision 1, 8 bits in last byte
    cmd[0] = 0x93;
    cmd[1] = 0x20;
    if (!PN5180sendData(cmd, 2, 0x00)) {  return 0; }
	//Read 5 bytes, we will store at offset 2 for later usage
    if (!PN5180_readData(5, cmd+2)) {  return 0; }
//			showMSG_withDATE("\nresponse0 : ", cmd, 7);
	//Enable RX CRC calculation
    if (!PN5180writeRegisterWithOrMask(CRC_RX_CONFIG, 0x01)) {  return 0; }
	//Enable TX CRC calculation
    if (!PN5180writeRegisterWithOrMask(CRC_TX_CONFIG, 0x01)) {  return 0; }
	//Send Select anti collision 1, the remaining bytes are already in offset 2 onwards
    cmd[0] = 0x93;
    cmd[1] = 0x70;
    if (!PN5180sendData(cmd, 7, 0x00)) { return 0; }
	//Read 1 byte SAK into response[2]
    if (!PN5180_readData(1, response+2)) {  return 0; }
	//// если бит 5(0х20) в байте SAK выставлен, то интерфейс  ISO/IEC14443-4 
	// Check if the tag is 4 Byte UID or 7 byte UID and requires anti collision 2
	// If Bit 3 is 0 it is 4 Byte UID
	
//	send_str_USB("\nread SAK0: ", response[2]);
	
    if ((response[2] & 0x04) == 0) {
		// Take first 4 bytes of anti collision as UID store at offset 3 onwards. job done
        for (uint8_t i = 0; i < 4; i++) response[3+i] = cmd[2 + i];
        SAK = response[2];
        uidLength = 4;
#ifdef DebugMSG
//? 		send_str_USB("\nbuffer[2] & 0x04 = 0, ", uidLength);
		showMSG_withDATE("  Data buffer ", response, 7);
#endif		
    }
	else {
		// Take First 3 bytes of UID, Ignore first byte 88(CT)
        if (cmd[2] != 0x88)	{  return 0; }
        for (uint8_t i = 0; i < 3; i++) response[3+i] = cmd[3 + i];
		// Clear RX CRC
        if (!PN5180writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE)) {  return 0; }
		// Clear TX CRC
        if (!PN5180writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE)) {  return 0; }
		// Do anti collision 2
        cmd[0] = 0x95;
        cmd[1] = 0x20;
        if (!PN5180sendData(cmd, 2, 0x00)) {  return 0; }
		//Read 5 bytes. we will store at offset 2 for later use
        if (!PN5180_readData(5, cmd+2)) { return 0; }
        else { 
#ifdef DebugMSG
//? 		send_str_USB("\nЧтение 5 байт успешно!", 0); 
#endif		
//						showMSG_withDATE("\nresponse2_70 : ", cmd, 7);
		} 
        for (uint8_t i = 0; i < 4; i++) {
            response[6 + i] = cmd[2+i];
        }
		//Enable RX CRC calculation
        if (!PN5180writeRegisterWithOrMask(CRC_RX_CONFIG, 0x01)) {  return 0; }
		//Enable TX CRC calculation
        if (!PN5180writeRegisterWithOrMask(CRC_TX_CONFIG, 0x01)) {  return 0; }
		//Send Select anti collision 2 
        cmd[0] = 0x95;
        cmd[1] = 0x70;
        if (!PN5180sendData(cmd, 7, 0x00)){  return 0; }
		//Read 1 byte SAK into buffer[2]
        if (!PN5180_readData(1, response + 2)) { return 0; }
        uidLength = 7;
        SAK = response[2];
#ifdef DebugMSG
//? 		send_str_USB("\nbuffer[2] & 0x04 != 0, ", uidLength);
		showMSG_withDATE("  Data_buffer ", response, 10);
#endif			
    }
    if(uidLength != 0) search_typeStandart(SAK, uidLength, response);
    return uidLength;
}
///////////////////////////////////////////
bool write_KEY_mifareCL(struct_RWTrailer str_access) {
    uint8_t cmd[7];
    uint8_t buffer[16];
    uint8_t buf[16];
    bool  success = false;
    if(struct_TR.bitCONF>7) { 
//?         send_str_USB("\nНе выбрана конфигурация трейлера!", 0); 
        return false; 
    }
    if((struct_TR.adr__+1)%4 == 0 || struct_TR.adr__ == 0) { 			//// условие, что запись действительно производится в блок 3
        if(MIFARE_classicREAD(struct_TR.keyAUTH, struct_TR.adr__, NFCA, buffer )) {	//MIFARE_classicREAD
            for(uint8_t i = 0; i<6; i++) { buf[i] =  struct_TR.keyA[i]; buf[i+10] =  struct_TR.keyB[i]; }	
            switch(struct_TR.adrBL) {
                case 0:												//// конфигурация блока 0 данных
                    buf[6] = buffer[6] & ~((struct_TR.bitCONF & 0x4)>>2); /// conf bit ~C10
                    buf[6] = buf[6] & ~((struct_TR.bitCONF & 0x2)<<3); 		/// conf bit ~C20
                    buf[7] = buffer[7] & ~(struct_TR.bitCONF & 0x1); 			/// conf bit ~C30
                    buf[7] = buf[7] | ((struct_TR.bitCONF & 0x4)<<2); 		/// conf bit C10
                    buf[8] = buffer[8] | ((struct_TR.bitCONF & 0x2)>>1); 	/// conf bit C20
                    buf[8] = buf[8] | ((struct_TR.bitCONF & 0x1)<<4); 		/// conf bit C30				
                break;
                case 1:												//// конфигурация блока 1 данных
                    buf[6] = buffer[6] & ~((struct_TR.bitCONF & 0x4)>>1); /// conf bit ~C11
                    buf[6] = buf[6] & ~((struct_TR.bitCONF & 0x2)<<4); 		/// conf bit ~C21
                    buf[7] = buffer[7] & ~((struct_TR.bitCONF & 0x1)<<1); /// conf bit ~C31
                    buf[7] = buf[7] | ((struct_TR.bitCONF & 0x4)<<3); 		/// conf bit C11
                    buf[8] = buffer[8] | (struct_TR.bitCONF & 0x2); 			/// conf bit C21
                    buf[8] = buf[8] | ((struct_TR.bitCONF & 0x1)<<5); 		/// conf bit C31						
                break;
                case 2:												//// конфигурация блока 2 данных
                    buf[6] = buffer[6] & ~(struct_TR.bitCONF & 0x4); 			/// conf bit ~C12
                    buf[6] = buf[6] & ~((struct_TR.bitCONF & 0x2)<<5); 		/// conf bit ~C22
                    buf[7] = buffer[7] & ~((struct_TR.bitCONF & 0x1)<<2); /// conf bit ~C32
                    buf[7] = buf[7] | ((struct_TR.bitCONF & 0x4)<<4); 		/// conf bit C12
                    buf[8] = buffer[8] | ((struct_TR.bitCONF & 0x2)<<1); 	/// conf bit C22
                    buf[8] = buf[8] | ((struct_TR.bitCONF & 0x1)<<6); 	/// conf bit C32						
                break;
                case 3:												//// конфигурация управления ключами и битами доступа
                    buf[6] = buffer[6] & ~((struct_TR.bitCONF & 0x4)<<1); 		/// conf bit ~C13
                    buf[6] = buf[6] & ~((struct_TR.bitCONF & 0x2)<<6); 				/// conf bit ~C23
                    buf[7] = buffer[7] & ~((struct_TR.bitCONF & 0x1)<<3); 		/// conf bit ~C33
                    buf[7] = buf[7] | ((struct_TR.bitCONF & 0x4)<<5); 				/// conf bit C13
                    buf[8] = buffer[8] | ((struct_TR.bitCONF & 0x2)<<2); 			/// conf bit C23
                    buf[8] = buf[8] | ((struct_TR.bitCONF & 0x1)<<7); 				/// conf bit C33							
                break;	
            }
            showMSG_withDATE("\nTr ", buf, 16);
			if(!MIFARE_AUTH(struct_TR.keyAUTH, struct_TR.adr__, NFCA)) {
                cmd[0] = 0xA0;
                cmd[1] = struct_TR.adr__;
                PN5180sendData(cmd, 2, 0x00);
                PN5180_readData(1, cmd);
			
                if(PN5180sendData(buf,16, 0x00))success = true;
                DelaymS(pause*2);
	// Read ACK/NAK
                PN5180_readData(1, cmd);
                mifare_mifareHalt();			
			}
        }
    }
    else { 
//?         send_str_USB("\n Неправильно выбран адрес блока!", 0); 
    }
    return success;
}
///////////////// авторизация по стандарту MIFARE_classic
/*
если конфиг. позволяет читать ключ В, то авторизация через этот ключ и соответственно любые другие операции не пройдут, нужно установить 
биты доступа так, чтобы ключ не читался
*/
uint8_t MIFARE_AUTH(uint8_t* key, uint8_t adr, uint8_t typeNFC){
    uint8_t answ = 99;
	/*
			answ = 0				 - Authentication successful.
			answ = 1				 - Authentication failed (permission denied).
			answ = 2				 - Timeout waiting for card response (card not present).
			answ = 3...0xff  - RFU	
	*/
    uint8_t cmd[13];
#ifdef key_def
    for(uint8_t i = 0; i<6; i++)key[i] = 0xff; 
#endif	
	
    cmd[0] = PN5180_MIFARE_AUTHENTICATE;
    for(uint8_t i = 0; i<6; i++)cmd[i+1] = key[i]; 
    if(typeNFC == NFCA) cmd[7] = 0x60;
    if(typeNFC == NFCB) cmd[7] = 0x61;
    cmd[8] = adr;
    if(activateTypeA(1)){
        for(uint8_t i = 0; i<4; i++){ cmd[9+i] = response[3+i]; }
//		for(uint8_t i = 0; i<4; i++){ cmd[9+i] = response[6 - i]; }
#ifdef debug_READcmd
        showMSG_withDATE("\nAUTH cmd[] ", cmd, 13);
#endif		
        if(!PN5180transceiveCommand(cmd, 13, (uint8_t*)&answ, 1)) return 88;
	}
	if(answ != 0) { PN5180reset();	mifare_setupRF(); }
    return answ;
}
///// чтение блока с адресом adr по стандарту MIFARE_classic
bool MIFARE_classicREAD(uint8_t* key, uint8_t adr, uint8_t typeNFC, uint8_t* buffer) {
    bool success = false;
    uint16_t len;
    uint8_t cmd[7];
    if(!MIFARE_AUTH(key, adr, typeNFC)) {
        cmd[0] = 0x30;
        cmd[1] = adr;
        if (!PN5180sendData(cmd, 2, 0x00)){ return false; }
	//Check if we have received any data from the tag
        DelaymS(pause);
        len = rxBytesReceived();
        if (len == 16) {
		// READ 16 bytes into  buffer
            if (PN5180_readData(16, buffer)) success = true;
        }
        else { ; }	
//		if (PN5180_readData(len, buffer)) success = true;
        mifare_mifareHalt();
    }
    return success;	
}
///// запись данных по стандарту MIFARE_classic значения value по адресу блока adr
bool MIFARE_classicWRITE(uint8_t* key, int8_t adr, uint8_t typeNFC, int32_t value) {
    bool success = false;
    uint8_t cmd[7];
    uint8_t buffer[16];
    if((adr+1)%4 == 0 || adr == 0) { 
//?         send_str_USB("\nПопытка записи битов доступа!", 0); 
        return false;
    }
    if(!MIFARE_AUTH(key, adr, typeNFC)) {
        cmd[0] = 0xA0;
        cmd[1] = adr;
        PN5180sendData(cmd, 2, 0x00);
        PN5180_readData(1, cmd);
	// Mifare write part 2
		//////////////////////
        for(uint8_t i = 0; i<4; i++){ buffer[i] = (uint8_t)(value>>(i*8));  buffer[i+8] = buffer[i]; }	
        value = ~value; 
        for(uint8_t i = 0; i<4; i++){ buffer[i+4] = (uint8_t)(value>>(i*8)); }
        buffer[12] = adr; buffer[14] = buffer[12];
        buffer[13] = ~adr; buffer[15] = buffer[13];
		//////////////////////
        if(PN5180sendData(buffer,16, 0x00))success = true;
		DelaymS(pause*2);
	// Read ACK/NAK
        PN5180_readData(1, cmd);
        mifare_mifareHalt();
	}
    return success;	
}

bool mifare__Read(uint8_t blockno, uint8_t *buffer) {
    bool success = false;
    uint16_t len;
    uint8_t cmd[7];
    activateTypeA(1);
    cmd[0] = 0x30;
    cmd[1] = blockno;
    if (!PN5180sendData(cmd, 2, 0x00)){ return false; }
	//Check if we have received any data from the tag
	DelaymS(pause);
	len = rxBytesReceived();
    if (len == 16) {
		// READ 16 bytes into  buffer
        if (PN5180_readData(16, buffer)) success = true;
    } else {  ; }	

//		if (PN5180_readData(len, buffer)) success = true;
    mifare_mifareHalt();
	return success;
}

uint8_t mifareBlockWrite4(uint8_t blockno, uint8_t *buffer) {
    uint8_t cmd[2];
    for(uint8_t k = 4; k<16; k++) { buffer[k] = 0; }
    if(activateTypeA(1) !=0 ) {
	// Mifare write part 1
        cmd[0] = 0xA0;
        cmd[1] = blockno;
        PN5180sendData(cmd, 2, 0x00);
        PN5180_readData(1, cmd);
	// Mifare write part 2
        PN5180sendData(buffer,16, 0x00);
        DelaymS(pause*2);
	// Read ACK/NAK
        PN5180_readData(1, cmd);
        mifare_mifareHalt();
    }
    return cmd[0];
}
uint8_t mifareBlockWrite16(uint8_t blockno, uint8_t *buffer) {
    uint8_t buf[16];
    uint8_t cmd = 0;
    uint8_t cntpack = 0;
	for(uint8_t i = 0; i<4; i++) {
        for(uint8_t k = 0; k<4; k++){ buf[k] = buffer[k+cntpack]; }
        cmd = mifareBlockWrite4(blockno+i, buf);
        if(cmd != 0xff) return 0;
        cntpack +=4;
    }
    return cmd;
}

bool mifare_mifareHalt(void) {
    uint8_t cmd[2];
	//mifare Halt
    cmd[0] = 0x50;
    cmd[1] = 0x00;
    PN5180sendData(cmd, 2, 0x00);	
    return true;
}

uint8_t mifare_readCardSerial(uint8_t *buffer) {
    uint8_t uidLength;
	// Always return 10 bytes
    // Offset 0..1 is ATQA
    // Offset 2 is SAK.
    // UID 4 bytes : offset 3 to 6 is UID, offset 7 to 9 to Zero
    // UID 7 bytes : offset 3 to 9 is UID
    for (uint8_t i = 0; i < 10; i++) response[i] = 0;
//    uidLength = activateTypeA(response, 1);
    uidLength = activateTypeA(1);
    if ((response[0] == 0xFF) && (response[1] == 0xFF)) return 0;
	// check for valid uid
    if ((response[3] == 0x00) && (response[4] == 0x00) && (response[5] == 0x00) && (response[6] == 0x00)) return 0;
    if ((response[3] == 0xFF) && (response[4] == 0xFF) && (response[5] == 0xFF) && (response[6] == 0xFF)) return 0;
    if ((response[6] == 0xFF) && (response[7] == 0xFF) && (response[8] == 0xFF) && (response[9] == 0xFF)) return 0;		
    for (uint8_t i = 0; i < 7; i++) buffer[i] = response[i+3];
    mifare_mifareHalt();
    return uidLength;  
}

bool mifare_isCardPresent(void) {
    return (mifare_readCardSerial(outbuf_NFC)>=4);
}
