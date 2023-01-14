#ifndef PN5180ISO14443_H
#define PN5180ISO14443_H

#include "PN5180.h"

#define NFCA_RFrxconf                           0x80
#define	NFCA_RFtxconf                           0 
//#define key_def

#define trailerCONF1                            0	// разрешено чтение ключа В и битов доступа через ключ А, запись ключей А и В через ключ А. Запись битов доступа запрещена
#define trailerCONF2                            2	// разрешено чтение ключа В и битов доступа через ключ А. Любая запись запрещена
#define trailerCONF3                            4	// разрешено чтение только битов доступа через ключи А или В. Запись ключей А и В разрешена через ключ В, запись битов доступа запрещена.
#define trailerCONF4                            6	// разрешено чтение только битов доступа через ключи А или В. Любая запись запрещена.
#define trailerCONF5                            1	// разрешено чтение всего, кроме ключа А. Запись всего тоже разрешена, все действия через ключ А. ОСНОВНАЯ КОНФИГУРАЦИЯ, по умолчанию стоит на новых чипах.
#define trailerCONF6                            3	// разрешено чтение только битов доступа через ключи А или В. Любая запись разрешена через ключ В.
#define trailerCONF7                            5	// разрешено чтение битов доступа через ключи А или В. Запись битов доступа через ключ В, запись ключей запрещена. 
#define trailerCONF8                            7	// разрешено чтение только битов доступа через ключи А или В. Любая запись запрещена. 

#define access_dataBL_CONF1                     0	// разрешены все операции блока через ключ А или ключ В
#define access_dataBL_CONF2                     2	// разрешено только чтение блока через ключ А или ключ В
#define access_dataBL_CONF3                     4	// разрешено чтение блока через ключ А или ключ В, запись разрешена через ключ В. Инкремент, декремент, транзакция и восстановление запрещены.
#define access_dataBL_CONF4                     6	// разрешено чтение, декремент, транзакция, восстановление блока через ключ А или В. Инкремент и запись разрешены через ключ В.
#define access_dataBL_CONF5                     1	// разрешено чтение, декремент, транзакция, восстановление блока через ключ А или В. Инкремент и запись запрещены.
#define access_dataBL_CONF6                     3	// разрешено чтение и запись блока 0 через ключ В. Инкремент, декремент, транзакция и восстановление запрещены.
#define access_dataBL_CONF7                     5	// разрешено только чтение блока 0 через ключ В. Все остальные операции запрещены. 
#define access_dataBL_CONF8                     7	// Все операции запрещены.

typedef struct {
    uint8_t keyAUTH[6];
    uint8_t keyA[6];
    uint8_t keyB[6];
    uint8_t adr__;                                  // адрес от 0 и до последнего 16-ти байтного сектора в памяти
    uint8_t adrBL;			                        // адрес блока от 0 до 3, 3 - трейлер(доступ к ключам и битам доступа), 0-2 - доступ к данным
    uint8_t bitCONF;
}struct_RWTrailer;

uint8_t activateTypeA(uint8_t kind);
uint8_t mifareBlockWrite4(uint8_t blockno, uint8_t *buffer);
uint8_t mifareBlockWrite16(uint8_t blockno, uint8_t *buffer);
bool mifare__Read(uint8_t blockno, uint8_t *buffer);
uint8_t MIFARE_AUTH(uint8_t* key, uint8_t adr, uint8_t typeNFC);
bool MIFARE_classicREAD(uint8_t* key, uint8_t adr, uint8_t typeNFC, uint8_t* buffer);
bool MIFARE_classicWRITE(uint8_t* key, int8_t adr, uint8_t typeNFC, int32_t value);
bool mifareHalt(void);
bool write_KEY_mifareCL(struct_RWTrailer str_access);

void init_structNFC(struct_NFCteg strNFC); 
bool mifare_setupRF(void);
uint8_t mifare_readCardSerial(uint8_t *buffer);    
bool mifare_isCardPresent(void);    
bool mifare_mifareHalt(void);

#endif /* PN5180ISO14443_H */
