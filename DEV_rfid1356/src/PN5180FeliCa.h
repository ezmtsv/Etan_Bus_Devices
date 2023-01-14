#ifndef PN5180FeliCa_H
#define PN5180FeliCa_H

#include "PN5180.h"

uint8_t pol_req(uint8_t *buffer);
bool felica_setupRF(void);
uint8_t felica_readCardSerial(uint8_t *buffer);    
uint8_t felica_readBlock(uint8_t num_bl, uint8_t * buf);
bool felica_isCardPresent(void);    

#endif /* PN5180FeliCa_H */
