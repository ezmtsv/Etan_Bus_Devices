#include "rfid_125khz.h"
//РУс
uint8_t SEL_MODL;
bool rec_TAG = false;
uint8_t RFID_Config [4];

static void init_PIN(void) {
    GPIOA->MODER |= GPIO_AFN(CLK_125K) | GPIO_INP(IN_RFID);
    GPIOA->AFR[0] |= GPIO_AF(CLK_125K, 5); 
	GPIOA->PUPDR = GPIO_PUP(IN_RFID);
}

void init_RFID125khz(void) {
    
    init_PIN();
    
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    TIM16->PSC = 0;
    TIM16->ARR = 383;
    TIM16->CCR1 = 191;
    TIM16->CCER |= TIM_CCER_CC1E;
    TIM16->BDTR |= TIM_BDTR_MOE;
    TIM16->CCMR1|=(TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
    TIM16->CR1 &= ~TIM_CR1_DIR;
    TIM16->CR1 |= TIM_CR1_CEN;    
   
//  GPIOA->MODER |= GPIO_OUT(CLK_125K);
        
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->PSC = 48;
    TIM14->ARR = 10200;
/*    
    TIM14->DIER |= TIM_IT_Update;
    NVIC_SetPriority (TIM14_IRQn, 1);
    NVIC_EnableIRQ(TIM14_IRQn);
    TIM14->CR1 |= TIM_CR1_CEN; 
*/    
}   
/*
void TIM14_IRQHandler(void){
    CLK_125kHz_toggle;
    TIM14->SR = (uint16_t)~TIM_IT_Update;
}
*/

uint16_t find_time(void) {                               //FIND TIME PERIOD BIT
    uint16_t dim[256];
    uint16_t dim_tmp[10];
    uint16_t last_max = 0xffff;
    uint16_t cur_max = 0;
    uint16_t dim_cntmax[10+1];
    bool flag_eq = false;
    uint16_t time_tmp = 0;
    uint32_t timeM = 0;					
    uint16_t timeT = 0;					/// период бита
	///////////выстраивание самых больших 10 чисел из массива, включая равные максимальные числа(они тоже входят в десятку)
    for(int i = 0; i<10; i++)dim_tmp[i] = 0;
    for(int i = 0; i<10; i++)dim_cntmax[i] = 0;	
    for(uint16_t i =0; i<127; i++) {		//127
        while(!stat_IN_data){
            time_tmp = countTIM14;
            if(time_tmp>2000){ timeT = 0; goto end_time; }
        }
        countTIM14 = 0;
        dim[i*2]= time_tmp;
        while(stat_IN_data) {
            time_tmp = countTIM14;
            if(time_tmp>2000){ timeT = 0; goto end_time; }
        }	
        countTIM14 = 0;
        dim[(i*2)+1]= time_tmp;
    }
    for(int8_t j = 9; j>=0; j--) {
        for(uint8_t i = 0; i<254; i++) { 
            timeM = dim[i];
            if(timeM <= last_max ) {
                if(cur_max< timeM) { 
                    for(uint8_t k = 0; k<10; k++){ if(dim_cntmax[k] == i){ flag_eq = true;} }
                    if(!flag_eq) {
                        cur_max = timeM; 
                        dim_cntmax[j] = i;				//// сохраняем индексы массива с максимумами
                    }
                    flag_eq = false;
                }
            }
        }
        dim_tmp[j] = cur_max; 
        last_max = cur_max; cur_max = 0; 
    }
    for(int i = 0; i<10; i++){ timeT += dim_tmp[i]; }
    timeT /= 10; 
end_time:		
		
    return timeT;
}

uint64_t read_TAG64bitManch(void) {                         //MANCHESTER CODE
    uint64_t dataTAG = 0;
    uint16_t cnt_bit = 0;
    uint16_t cnt_startbit = 0;	/// счетчик первых 9 бит преамбулы
    uint16_t timeT = 0;					/// период бита
    uint16_t time_tmp = 0;
    uint16_t timeHalf = 0;
    uint16_t timeSET = 0;				/// период времени измерения
    uint16_t timeLong = 0;
    uint8_t  cnttry = 0;
    uint8_t  par_coulumn[11];
    uint8_t ERR_rec = 0;
    bool start_r = false;
    bool errparcolumn = false;
    uint8_t five_bit = 0;
    uint8_t fivePARval = 0;
    uint8_t five_bitCNT = 0;
    uint8_t bit_cur = 0;
    uint8_t val_tmp = 0;
//    uint8_t Buffer[6];

    TIM14->CR1 |= TIM_CR1_CEN;
    while(stat_IN_data) {
        if(countTIM14>4000) break;
    }	
    countTIM14 = 0;
    timeT = find_time();
    timeHalf = timeT/2;
    timeSET = (timeT*3)/4;
    timeLong = (timeT*5)/4;

    /////////////////////////////// определяем точку начала передачи бита по длинному импульсу, при смене состояния (0->1, 1->0) countTIM14 = timeHalf
STREAD:	
    countTIM14 = 0; time_tmp = 0; start_r = false;
    five_bitCNT = 0; five_bit = 0;
    while(1) {
        while(!stat_IN_data){
            time_tmp = countTIM14;
            if(time_tmp>2000)goto end_tag;
        }
        if(time_tmp>timeSET){ countTIM14 = timeHalf; goto SYNC; }
        countTIM14 = 0;
        while(stat_IN_data) {
            time_tmp = countTIM14;
            if(time_tmp>2000)goto end_tag;
		}	
        if(time_tmp>timeSET){ countTIM14 = timeHalf; goto SYNC; }
        countTIM14 = 0;
        cnt_bit++;
        if(cnt_bit>128)goto end_tag;
    }
SYNC:	
    ///////////
    /*
    dataTAG = (uint64_t)timeT; 
    dataTAG |= (uint64_t)(timeHalf)<<16;
    dataTAG |= (uint64_t)(timeSET)<<32;
    dataTAG |= (uint64_t)(timeLong)<<48;
    */
    ///////////    
	////////////////////////////////////// ждем прихода 9 единиц, начала пакета 
    cnt_bit = 0;
	while(!start_r){
        time_tmp = countTIM14;
        if(time_tmp>timeT)countTIM14 = 0;
        if(time_tmp>timeSET) { 
            if(stat_IN_data){ cnt_startbit++; bit_cur = 1; }
            else{ cnt_startbit = 0; bit_cur = 0; }
            cnt_bit++; 
            if(bit_cur == 1) {											//// синхронизируем при перходе 0->1
                while(stat_IN_data) {
                    time_tmp = countTIM14;
                    if(time_tmp>2000)goto end_tag;
                }
                if(time_tmp>timeLong)countTIM14 = timeHalf;
                else countTIM14 = 0;					
			}
            else {
                while(!stat_IN_data) {
                    time_tmp = countTIM14;
                    if(time_tmp>2000)goto end_tag;
                }
                if(time_tmp>timeLong)countTIM14 = timeHalf;
                else countTIM14 = 0;					
            }
        }
        if(cnt_startbit == 9) start_r = true;
        if(cnt_bit > 128)goto end_tag;
    }
    if(start_r) {																			//// получили 9 ед. начинаем чтение TAG
        dataTAG = 0x3FE;
        cnt_bit = 9;
        while(cnt_bit != 64) {
            time_tmp = countTIM14;
            if(time_tmp>timeT)countTIM14 = 0;
            if(time_tmp>timeSET) { 												///// время для момента измерения уровня линии
                if(stat_IN_data) {  
                    bit_cur = 1; dataTAG |= 1; 
                    five_bit ^= 1;					//// четность строк
                }
                else { 
                    bit_cur = 0;
                    five_bit ^= 0;					//// четность строк
                }
                dataTAG<<=1;
                cnt_bit++;
                five_bitCNT++;
                if(bit_cur == 1) {											//// синхронизируем при перходе 0->1
                    while(stat_IN_data) {
                        time_tmp = countTIM14;
                        if(time_tmp>2000)goto end_tag;
                    }
                    if(time_tmp>timeLong)countTIM14 = timeHalf;
                    else countTIM14 = 0;					
                }
                else {
                    while(!stat_IN_data){
                        time_tmp = countTIM14;
                        if(time_tmp>2000)goto end_tag;
                    }
                    if(time_tmp>timeLong)countTIM14 = timeHalf;
                    else countTIM14 = 0;					
                }
                //////////////////////////проверка четности строк///////	
                if(five_bitCNT == 5 && (cnt_bit<61)) {
                    if(five_bit != 0) { 
//                        Send_Buffer[5+cnttry] = cnt_bit; 
                        cnttry++; 
                        ERR_rec = 1;															//// ошибка четности строк 
                        if(cnttry > 30){ dataTAG = 0; goto end_tag; }
                        else {  goto STREAD; }  /// debugcnt = 0;						
                    }
                    else { ERR_rec = 3; }														//// нет ошибок по четности строк
                    five_bit = 0;
                    five_bitCNT = 0;
                }	
            }	
        }
        dataTAG>>=1;
        dataTAG |= 0x8000000000000000;
        /////////////проверка четности столбцов//////////////////
        for(uint16_t i = 0; i<11; i++){	par_coulumn[i] = (uint8_t)(dataTAG>>(i*5))&(0x1f);	}
        for(uint8_t j = 0; j<4; j++) {
            fivePARval = 0;
            for(uint16_t i = 1; i<11; i++) {
                val_tmp = par_coulumn[i];
                val_tmp >>= 1+j; 
                if(val_tmp & 1)fivePARval++;
            }
            val_tmp = par_coulumn[0];
            val_tmp >>= 1+j;
            if(val_tmp & 1) {
                if(fivePARval%2 == 0){ errparcolumn = true; }
            }
            else { 
                if(fivePARval%2 == 1) { errparcolumn = true; }
            }	
            if(errparcolumn) { 
                ERR_rec = 2;			                            // ошибка четности столбцов
            }                                                       // если обнаружена ошибка четности столбцов
            else{ ERR_rec = 3; }
        }
    }
	/////////////////////////////////////////
    
    end_tag:	
    
    /*
    Buffer[0] = cnttry;
    Buffer[1] = (uint8_t)(timeT>>8); Buffer[2] = (uint8_t)timeT;
    Buffer[3] = (uint8_t)(cnt_bit); 
    Buffer[4] = (uint8_t)cnt_startbit;
    */
    
    if(ERR_rec != 3) {                                              // если обнаружена ошибка четности строк или столбцов - сбрасываем кол- принятых ьит
//        Buffer[0] = ERR_rec; Buffer[3] = 0; 
        dataTAG = 0; 
    }  
    else { 
        if(cnt_bit == 64) rec_TAG = true; 
    }
//    Buffer[0] = ERR_rec;
    
    delay_us(4*800);
    TIM14->CR1 &= ~TIM_CR1_CEN;
    return dataTAG;
}

/////////////////////////////////////BiPhase COD
uint64_t read_TAG64bitBIphase(bool modul_invers) {  //// если бит modul_invers выставлен, то инверсная бифазная ( 1 - два коротких имп., 0 - один длинный)
    uint64_t dataTAG = 0;
    uint16_t cnt_bit = 0;
    uint16_t cnt_startbit = 0;	/// счетчик первых 9 бит преамбулы
    uint16_t timeT = 0;					/// период бита
    uint16_t time_tmp = 0;
    uint16_t timeSET = 0;				/// период времени измерения
    uint8_t  cnttry = 0;
    uint8_t  par_coulumn[11];
    uint8_t ERR_rec = 0;
    uint8_t val_tmp;
    uint8_t last_level = 0;
    bool start_r = false;
    bool errparcolumn = false;
    uint8_t five_bit = 0;
    uint8_t fivePARval = 0;
    uint8_t five_bitCNT = 0;
    uint8_t cnt2bit = 0;
    uint8_t err_synchro = 0;
    TIM14->CR1 |= TIM_CR1_CEN;
//    uint8_t Buffer[6];

    while(stat_IN_data) {
        if(countTIM14>4000)break;
    }	
    countTIM14 = 0; 	
/////////////вычисление среднего времени периода бита
    timeT = find_time();		///////////////////// находим среднее время передачи импульса (период)
    if(timeT<380) timeT = (uint16_t)((double)timeT*0.85); 		
    timeSET = (timeT*3)/4;
	/////////////////////////////// определяем точку начала передачи бита по длинному импульсу
    STREAD:	
    five_bit = 0;
    five_bitCNT = 0;
    time_tmp = 0; start_r = false;
    countTIM14 = 0;
    if(stat_IN_data) last_level = 1;
    else last_level = 0;
    while(1) {
        while(stat_IN_data == last_level) {
            time_tmp = countTIM14;
            if(time_tmp>2000){ goto end_tag; }
        }
        last_level = stat_IN_data;
        countTIM14 = 0;
        cnt_bit++;
        if(time_tmp>timeSET){ countTIM14 = 0; goto SYNC; }
        if(cnt_bit>128) goto end_tag; 
    }	
	////////////////////////////////////// ждем прихода 9 единиц, начала пакета 
SYNC:		
    cnt_bit = 0; 
    while(!start_r) {
        while(stat_IN_data == last_level) {
            time_tmp = countTIM14;
            if(time_tmp>2000){ goto end_tag; }
        }
        last_level = stat_IN_data;
        if(time_tmp>timeSET) {
            if(cnt2bit == 1)err_synchro++;
            if(modul_invers) cnt_startbit = 0;	
            else cnt_startbit++;
            countTIM14 = 0;								//// синхронизация
        }
        else {
            countTIM14 = 0;							  //// синхронизация
            cnt2bit++; 
            if(cnt2bit == 3) err_synchro++;
                if(cnt2bit == 2) { 
                    cnt2bit = 0;  
                    if(modul_invers) cnt_startbit++;	
                    else cnt_startbit = 0;		
                }   
        }		
        cnt_bit++;
        if(cnt_startbit == 9) start_r = true;
        if(cnt_bit > 128){ ERR_rec = 4; goto end_tag; }
    }	
    if(start_r) {																			//// получили 9 ед. начинаем чтение TAG
        dataTAG = 0x3FE;
        cnt_bit = 9;	
        while(cnt_bit != 64) {	
            while(stat_IN_data == last_level) {
                time_tmp = countTIM14;
                if(time_tmp>2000){ goto end_tag; }
            }
            last_level = stat_IN_data;		
            if(time_tmp>timeSET) {
                if(cnt2bit == 1)err_synchro++;
                if(modul_invers){ five_bit ^= 0; }  //// четность строк
                else {
                    dataTAG |=1;
                    five_bit ^= 1;					//// четность строк
                }
                dataTAG<<=1;
                cnt_bit++;
                five_bitCNT++;
                countTIM14 = 0;										//// синхронизация
            }
            else {
                cnt2bit++; 
                if(cnt2bit == 3)err_synchro++;
                if(cnt2bit == 2) { 
                    if(modul_invers){ 
                        dataTAG |=1; five_bit ^=  1; 
                    }
                    else{
                        five_bit ^=  0;				//// четность строк
                    }
                    dataTAG<<=1;
                    cnt_bit++; 
                    five_bitCNT++;
                    cnt2bit = 0; countTIM14 = 0;   	//// синхронизация	
                }   
                countTIM14 = 0; 	
            }		
            if(five_bitCNT == 5 && (cnt_bit<61)) {
                if(five_bit != 0){ 
                    err_synchro = 111; 
//                    Send_Buffer[5+cnttry] = cnt_bit; 
                    cnttry++; 
                    ERR_rec = 1;															//// ошибка четности строк 
                    if(cnttry > 30){ dataTAG = 0; goto end_tag; }
                    else { goto STREAD; }  /// debugcnt = 0;						
                }
                else { 
                    ERR_rec = 3; 
                }														//// нет ошибок по четности строк
                five_bit = 0;
                five_bitCNT = 0;
            }	
        }
        dataTAG>>=1;
        dataTAG |= 0x8000000000000000;
			/////////////проверка четности столбцов//////////////////
        for(uint16_t i = 0; i<11; i++) {	par_coulumn[i] = (uint8_t)(dataTAG>>(i*5))&(0x1f); }
        for(uint8_t j = 0; j<4; j++) {
            fivePARval = 0;
            for(uint16_t i = 1; i<11; i++) {
                val_tmp = par_coulumn[i];
                val_tmp >>= 1+j; 
                if(val_tmp & 1)fivePARval++;
            }
            val_tmp = par_coulumn[0];
            val_tmp >>= 1+j;
            if(val_tmp & 1) {
                if(fivePARval%2 == 0) { errparcolumn = true; }
            }
            else { 
                if(fivePARval%2 == 1){ errparcolumn = true; }
            }	
            if(errparcolumn) { 
                ERR_rec = 2;			// ошибка четности столбцов
            }                           // если обнаружена ошибка четности столбцов
            else { ERR_rec = 3; }
        }
    }
end_tag:	
/*    
    Buffer[1] = (uint8_t)(timeT>>8); Buffer[2] = (uint8_t)timeT;
    Buffer[3] = (uint8_t)(cnt_bit); 
    Buffer[4] = (uint8_t)cnt_startbit;
    Buffer[0] = err_synchro;
*/    
    if(ERR_rec != 3) {                                              // если обнаружена ошибка четности строк или столбцов - сбрасываем кол- принятых ьит
//        Buffer[0] = ERR_rec; Buffer[3] = 0; 
        rec_TAG = false; 
        dataTAG = 0; 
    }  
    else { 
        if(cnt_bit == 64) rec_TAG = true; 
    }
//    Buffer[0] = ERR_rec;    /// маркер операции чтения - 3 - успех, 2 - ошибка четности столбцов, 1 - ошибка четности строк 
    delay_us(4*800);
    TIM14->CR1 &= ~TIM_CR1_CEN;
    return dataTAG;
}

void read_tag(uint8_t* buf) {
    uint64_t data_tmp64bit = 0;
    SEL_MODL = manch;
    data_tmp64bit = read_TAG64bitManch();
    if(data_tmp64bit == 0) {
        SEL_MODL = biPhaseD;
        data_tmp64bit = read_TAG64bitBIphase(false);
    }
    if(data_tmp64bit == 0) {
        SEL_MODL = biPhaseR;
        data_tmp64bit = read_TAG64bitBIphase(true);
    }
//    if(data_tmp64bit != 0) {
        for(uint8_t i = 0; i<8; i++) { buf[i+4] = (uint8_t)(data_tmp64bit>>(56-(8*i))); }
        buf[12] = SEL_MODL;
//    }    
//    data_tmp64bit = read_TAG64bitManch();
//    for(uint8_t i = 0; i<8; i++) { buf[i+4] = (uint8_t)(data_tmp64bit>>(56-(8*i))); }
}
///////////////////////
void StartGap(void) {
    TIM16->CCR1 = 0;
    delay_us(300);
    TIM16->CCR1 = 191;       
}

void SendOne(void) {
    delay_us(420);
    TIM16->CCR1 = 0;
    delay_us(300);
    TIM16->CCR1 = 191;        
}

void SendZero(void) {
    delay_us(150);
    TIM16->CCR1 = 0;
    delay_us(300);
    TIM16->CCR1 = 191;    
}
void OpCode_LockBit(void) {
    SendOne();
    SendZero();
    SendZero();
}

void WriteReset(void) {
    SendZero();
    SendZero();
}

void AddressSelect(uint8_t data) {
    if ( testbit(data,2) == 1 ) { SendOne(); } else { SendZero(); } 
    if ( testbit(data,1) == 1 ) { SendOne(); } else { SendZero(); } 
    if ( testbit(data,0) == 1 ) { SendOne(); } else { SendZero(); } 
}

void init_configRFID(uint8_t bitrate, uint8_t modul, uint8_t pskCF) {
    RFID_Config[0] = 0;
    RFID_Config[1] = bitrate;
    RFID_Config[2] = (modul<<1); RFID_Config[1] |= (modul>>7);
    RFID_Config[2] |= pskCF; 
    RFID_Config[3] = 0x40;
}
void WritePage(uint8_t *data) {
    StartGap();
    OpCode_LockBit();
    for (uint8_t i = 0; i < 4; i++) {
        for (int8_t j = 7; j >= 0; j--) {
            if ( testbit(data[i],j) == 1 ) { SendOne(); } 
            else { SendZero(); }
        }
    } 
    AddressSelect(1);
    delay_us(100*800);
    StartGap();
    OpCode_LockBit();
    for (uint8_t i = 4; i < 8; i++) {
        for (int8_t j = 7; j >= 0; j--) {
            if ( testbit(data[i],j) == 1 ) { SendOne(); } else { SendZero(); }
        }
    } 
    AddressSelect(2);
    delay_us(100*800);
}

void WriteConfig(uint8_t *data) {
    StartGap();
    OpCode_LockBit();
    for (uint8_t i = 0; i < 4; i++) {
        for (int8_t j = 7; j >= 0; j--) {
            if ( testbit(data[i],j) == 1 ) { 
                SendOne(); 
            } 
            else { 
                SendZero(); 
            }
        }
    } 
    AddressSelect(0);
    delay_us(100*800);
}

uint8_t CalculateCRC(uint8_t *buffer) {
    uint8_t crc = 0;
    for (uint8_t i=0; i<5; i++){
        crc = crc ^ ( buffer[i] & 0x0F ) ^ ( ( buffer[i] >> 4 ) & 0x0F );    
    }
    return crc; 
}

void PrepareSendBuffer(uint8_t *source_data, uint8_t *send_data) {
	uint8_t j;
    uint8_t send_crc; 
    uint8_t source_bits = 0; 
    uint8_t send_bits = 9; 
    uint8_t send_bit;
    //m=1;
    send_data[0]=0xFF;
    send_data[1]=0x80;
    do {
        j=0;
        send_crc = 0;
        do {
            send_bit = testbit(source_data[source_bits/8], (7 - source_bits%8));
            send_crc = send_crc ^ send_bit;                          
            if ( send_bit == 1 ) { send_data[send_bits/8]|=(1<<(7 - send_bits%8)); }
            source_bits++;
            send_bits++;
            j++;
        } while (j<4);
        if ( send_crc == 1 ) { send_data[send_bits/8]|=(1<<(7 - send_bits%8)); }
        send_bits++;
    } while (source_bits<40);
    send_data[7] |= ( CalculateCRC(source_data) << 1 );
}
/*
Для записи в ТЕГ конфигурируем значения в буфере RFID_Config выбранной модуляцией, битрейтом и битами PSK. Затем заполняем 5 байт данных в буфере
buf и вызываем функцию записи. Пример ниже.
				init_configRFID(bitrate_rf, mod_rf, pskCF_RF_2);
				buf[0] =  0; buf[1] =  0;  buf[2] =  31; buf[3] =  159; buf[4] = 9;
				writeRFID(buf);
*/
void writeRFID(uint8_t* buf) {
    uint8_t RFID_Send_Buffer [8];
    delay_us(100*800);
/*	
    RFID_Config[0]=0x00;
    RFID_Config[1]=0x14;
    RFID_Config[2]=0x80;
    RFID_Config[3]=0x40;
*/
    PrepareSendBuffer(buf, RFID_Send_Buffer);
    WritePage(RFID_Send_Buffer);  
    WriteConfig(RFID_Config);  
    StartGap();
    WriteReset();	
	delay_us(300*800);
}
