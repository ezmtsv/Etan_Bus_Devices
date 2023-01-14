#include "PN5180FeliCa.h"
#include "PN5180.h"

extern uint16_t buffer_SPI[64];
extern uint16_t N_bait_spi;
extern struct_NFCteg cur_NFCteg;

void DelaymS(uint32_t us);
void send_dim_SPI(uint8_t* dim, uint16_t len);

bool felica_setupRF(void) {		///  инициализация RF модуля чипа
//  PN5180DEBUG(F("Loading RF-Configuration...\n"));
    if (PN5180loadRFConfig(0x09, 0x89)) {  // FeliCa 424 parameters
//    PN5180DEBUG(F("done.\n"));
    }
    else return false;
//  PN5180DEBUG(F("Turning ON RF field...\n"));
    if (PN5180setRF_on()) {
//    PN5180DEBUG(F("done.\n"));
    }
    else return false;
    return true;
}

/*
* buffer : must be 20 byte array
* buffer[0-1] is length and 01
* buffer[2..9] is IDm
* buffer[10..17] is PMm.
* buffer[18..19] is POL_RES data
*
* return value: the uid length in bytes:
* -	zero if no tag was recognized
* -	8 if a FeliCa tag was recognized
*/

uint8_t pol_req(uint8_t *buffer) {
    uint8_t cmd[6];
    uint8_t getRX = 0;	
	// Load FeliCa 424 protocol
    if(!PN5180loadRFConfig(0x09, 0x89)) return 0;
	// OFF Crypto
	if(!PN5180writeRegisterWithAndMask(SYSTEM_CONFIG, 0xFFFFFFBF)) return 0;
	//send FeliCa request (every packet starts with length)
    cmd[0] = 0x06;             //total length
    cmd[1] = 0x00;             //POL_REQ command
    cmd[2] = 0xFF;             //
    cmd[3] = 0xFF;             // any target
    cmd[4] = 0x01;             // System Code request // при отправке 1 последние 2 байта сообщают системный код SC
    cmd[5] = 0x00;             // 1 timeslot only
    if (!PN5180sendData(cmd, 6, 0x00)) return 0;
    //wait a little to avoid bug with some cards
    DelaymS(50);
    //response packet should be 0x14 (20 bytes total length), 0x01 Response Code, 8 IDm bytes, 8 PMm bytes, 2 Request Data bytes
    //READ 20 bytes reply
    uint8_t *internalBuffer = PN5180readData(20);
    getRX = rxBytesReceived(); 
    if (!internalBuffer) return 0;
    for (int i=0; i<getRX; i++) {
	//	for (int i=0; i<20; i++) {
        buffer[i] = internalBuffer[i];
    }
     //check Response Code
    if ( buffer[1] != 0x01 ){
 //       uidLength = 0;
        getRX = 0;
    } 
    else {
     //   uidLength = 8;
		 //send_str_USB("\nuidLength ", getRX);
    }
    return getRX; 		//uidLength;
}

uint8_t felica_readCardSerial(uint8_t *buffer) {
    uint8_t response[20];
    uint8_t getByte;
    
    for (int i = 0; i < 20; i++) response[i] = 0;
	getByte = pol_req(response);
    if (getByte == 0) return 0;
    else {
        for (int i = 0; i < 8; i++) { buffer[i] = response[i+2]; cur_NFCteg.UID[i] = buffer[i]; }
        for (int i = 0; i < 8; i++) { cur_NFCteg.PMm[i] = response[i+10]; }
        cur_NFCteg.SC = ((uint16_t)(response[18])<<8) | (uint16_t)response[19];
        cur_NFCteg.typeNFC = NFCF;
        if(cur_NFCteg.SC == 0x88B4) { cur_NFCteg.standart = Sony_Felica_LiteS; cur_NFCteg.mem_size = 224; }
        if(cur_NFCteg.SC == 0xFE00) cur_NFCteg.standart = Sony_Felica_CommonArea;
        if(cur_NFCteg.SC == 0xFEE1) cur_NFCteg.standart = Sony_Felica_plug;
        if(cur_NFCteg.SC > 0x3FFF && cur_NFCteg.SC < 0x5000) cur_NFCteg.standart = Sony_Felica_Host;
        cur_NFCteg.len_UID = 8;		// uidLength;
//		showMSG_withDATE("\n: ", response, 20);
///////////////////////////////////////////	
    }
    return getByte;
}
uint8_t felica_readBlock(uint8_t num_bl, uint8_t * buf) {
    uint8_t cmd[32];	
    uint8_t getRX = 0;
    if(felica_readCardSerial(cmd) != 0) {
        cmd[0] = 16;
        cmd[1] = 6;   /// /// команда чтения без использования крипто.
        for(uint8_t i = 0; i<8; i++) { cmd[i+2] = cur_NFCteg.UID[i]; }
        cmd[10] = 1;				//// number service
        cmd[11] = 0x0B;			//// service code
        cmd[12] = 0;				//// service code
        cmd[13] = 1;				//// number of block
        cmd[14] = 0x80;			//// Block list
        cmd[15] = num_bl;		//// Block list
        if (!PN5180sendData(cmd, 16, 0x00)) return 0;
        DelaymS(1);
        uint8_t *internalBuffer = PN5180readData(29);
        for(uint8_t i = 0; i<29; i++)buf[i] = internalBuffer[i];
        showMSG_withDATE("\n: ", buf, 16);
        getRX = rxBytesReceived();
//?        send_str_USB("\nuidLength ", getRX);
    }
    return getRX;
}
bool felica_isCardPresent(void) {
    uint8_t buffer[8];
    return (felica_readCardSerial(buffer) != 0);
}
