#include "OW.h"
//РУс
uint8_t command[80];
uint8_t buf_d[80];
uint8_t out_DMA[10];
uint64_t key[100];

static void init_PIN(void) {
    #define GPIO_AFN(pin)   (0x02U << ((pin) << 1))
    #define GPIO_AF(pin, func) ((0x0FU & (func)) << ((0x07U & (pin)) << 2))
	
    GPIOA->MODER |= GPIO_AFN(TX_PIN_OW) | GPIO_AFN(RX_PIN_OW);
    GPIOA->AFR[0] |= GPIO_AF(TX_PIN_OW, 1) | GPIO_AF(RX_PIN_OW, 1); 
}

void init_USART(uint32_t BR) {
		uint32_t divider = 0, tmpreg = 0;	
	
				  // Disable USART //
    //USART1->CR1 &= (uint32_t)~((uint32_t)USART_CR1_UE);
	  USART1->CR1 &= ~((uint32_t)USART_CR1_UE);
   //---------------------------- USART CR2 Configuration -----------------------//
    tmpreg = USART1->CR2;
  // Clear STOP[13:12] bits //
    tmpreg &= (uint32_t)~((uint32_t)USART_CR2_STOP);
  // Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit ------------//
  // Set STOP[13:12] bits according to USART_StopBits value //
    tmpreg |= (uint32_t)USART_StopBits_1;
  // Write to USART CR2 //
    USART1->CR2 = tmpreg;
  //---------------------------- USART CR1 Configuration -----------------------//
    tmpreg = USART1->CR1;
  // Clear M, PCE, PS, TE and RE bits //
    tmpreg &= (uint32_t)~((uint32_t)CR1_CLEAR_MASK);
    tmpreg |= (uint32_t)USART_WordLength_8b | USART_Parity_No | (USART_Mode_Tx | USART_Mode_Rx);
  // Write to USART CR1 //
    USART1->CR1 = tmpreg;
  //---------------------------- USART BRR Configuration Oversampling mode is 8 Samples -----------------------//
    divider = (uint32_t)((48000000) / BR);		// BaudRate == 115200
    tmpreg  = (uint32_t)((48000000) % BR);
    if (tmpreg >=  BR / 2) { divider++; } 
    // get the LSB of divider and shift it to the right by 1 bit //
    tmpreg = (divider & (uint16_t)0x000F) >> 1;
    divider = (divider & (uint16_t)0xFFF0) | tmpreg;
  // Write to USART BRR //
    USART1->BRR = (uint16_t)divider;
    USART1->CR1 |= USART_CR1_UE;			
}	
void init_OW(void) {
	_BST(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
	_BST(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
	RCC->APB2ENR |= RCC_USART1EN;
	init_PIN();
	init_USART(115200);
	_BMD(SYSCFG->CFGR1, SYSCFG_CFGR1_USART1TX_DMA_RMP_Msk, SYSCFG_CFGR1_USART1TX_DMA_RMP_Msk);
	_BMD(SYSCFG->CFGR1, SYSCFG_CFGR1_USART1RX_DMA_RMP_Msk, SYSCFG_CFGR1_USART1RX_DMA_RMP_Msk);
	RCC->AHBENR |= RCC_AHBPeriph_DMA1; // DMA ON
}

FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint32_t USART_FLAG)
{
  FlagStatus bitstatus = RESET;
	
  if ((USARTx->ISR & USART_FLAG) != (uint16_t)RESET) {
    bitstatus = SET;
  }
  else {
    bitstatus = RESET;
  }
  return bitstatus;
}

uint16_t USART_ReceiveData(USART_TypeDef* USARTx)
{
  return (uint16_t)(USARTx->RDR & (uint16_t)0x01FF);
}

uint8_t reset_OW(void) {													//			сброс 1-wire
    uint8_t ow_presence;
	
    USART1->CR3 &= (uint32_t)~(USART_CR3_DMTX | USART_CR3_DMRX);	// OFF DMA
    init_USART(9600);				
    USART1->ICR = USART_ISR_TC;		// clean flag Transmission Complete
    USART1->TDR = (0xf0 & (uint16_t)0x01FF);	
    while (USART_GetFlagStatus(USART1, USART_ISR_RXNE) == RESET);		// ожидаем флага о приеме байта 
		ow_presence = USART_ReceiveData(USART1);
    init_USART(115200);				
    if (ow_presence != 0xf0) { return 1; }
    return 0;
}

void DMA_DeInitCH(DMA_Channel_TypeDef* DMAy_Channelx)
{
  /* Disable the selected DMAy Channelx */
  DMAy_Channelx->CCR &= (uint16_t)(~DMA_CCR_EN);
  /* Reset DMAy Channelx control register */
  DMAy_Channelx->CCR  = 0;
  /* Reset DMAy Channelx remaining bytes register */
  DMAy_Channelx->CNDTR = 0;
  /* Reset DMAy Channelx peripheral address register */
  DMAy_Channelx->CPAR  = 0;
  /* Reset DMAy Channelx memory address register */
  DMAy_Channelx->CMAR = 0;
  if (DMAy_Channelx == DMA1_Channel1) {
    /* Reset interrupt pending bits for DMA1 Channel1 */
    DMA1->IFCR |= DMA1_CHANNEL1_IT_MASK;
  } 
	else if (DMAy_Channelx == DMA1_Channel2) {
    /* Reset interrupt pending bits for DMA1 Channel2 */
    DMA1->IFCR |= DMA1_CHANNEL2_IT_MASK;
  }
  else if (DMAy_Channelx == DMA1_Channel3) {
    /* Reset interrupt pending bits for DMA1 Channel3 */
    DMA1->IFCR |= DMA1_CHANNEL3_IT_MASK;
  }
  else if (DMAy_Channelx == DMA1_Channel4) {
    /* Reset interrupt pending bits for DMA1 Channel4 */
    DMA1->IFCR |= DMA1_CHANNEL4_IT_MASK;
  }
  else if (DMAy_Channelx == DMA1_Channel5) {
    /* Reset interrupt pending bits for DMA1 Channel5 */
    DMA1->IFCR |= DMA1_CHANNEL5_IT_MASK;
  }
}

FlagStatus DMA_GetFlagStatus(uint32_t DMA_FLAG)
{
  FlagStatus bitstatus = RESET;
	
  /* Check the status of the specified DMA flag */
  if ((DMA1->ISR & DMA_FLAG) != (uint32_t)RESET) {
    /* DMA_FLAG is set */
    bitstatus = SET;
  }
  else {
    /* DMA_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the DMA_FLAG status */
  return  bitstatus;
}

static void wr_bitOW(uint8_t bt, uint8_t len) {													//запись нескольких бит( кол-во бит = num_bit, передаваемые биты - bt) для устройства 1-wire
    uint16_t cnt = 0;
    uint32_t tmpreg;	
	
    for(uint8_t i = 0; i<8; i++) {
      if(bt & 0x01) {
			  command[i] = 0xff;
		  }
		  else { command[i] = 0; }
		  bt>>=1;
	  }				
    DMA_DeInitCH(DMA1_Channel5);	
    DMA_DeInitCH(DMA1_Channel4);
    tmpreg = DMA1_Channel5->CCR;
  // Clear MEM2MEM, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits //
    tmpreg &= CCR_CLEAR_MASK;
    tmpreg |= DMA_DIR_PeripheralSRC | DMA_Mode_Normal |
            DMA_PeripheralInc_Disable | DMA_MemoryInc_Enable |
            DMA_PeripheralDataSize_Byte | DMA_MemoryDataSize_Byte |
            DMA_Priority_Low | DMA_M2M_Disable;
  // Write to DMAy Channelx CCR //
    DMA1_Channel5->CCR = tmpreg;
//--------------------------- DMAy Channelx CNDTR Configuration --------------//
  // Write to DMAy Channelx CNDTR //
    DMA1_Channel5->CNDTR = len;
//--------------------------- DMAy Channelx CPAR Configuration ---------------//
  // Write to DMAy Channelx CPAR //
    DMA1_Channel5->CPAR = (uint32_t) &(USART1->RDR);
//--------------------------- DMAy Channelx CMAR Configuration ---------------//
  // Write to DMAy Channelx CMAR //
    DMA1_Channel5->CMAR = (uint32_t) buf_d;
    tmpreg = DMA1_Channel4->CCR;
  // Clear MEM2MEM, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits //
    tmpreg &= CCR_CLEAR_MASK;
    tmpreg |= DMA_DIR_PeripheralDST | DMA_Mode_Normal |
            DMA_PeripheralInc_Disable | DMA_MemoryInc_Enable |
            DMA_PeripheralDataSize_Byte | DMA_MemoryDataSize_Byte |
            DMA_Priority_Low | DMA_M2M_Disable;
  // Write to DMAy Channelx CCR //
    DMA1_Channel4->CCR = tmpreg;
//--------------------------- DMAy Channelx CNDTR Configuration --------------//
  // Write to DMAy Channelx CNDTR //
    DMA1_Channel4->CNDTR = len;
//--------------------------- DMAy Channelx CPAR Configuration ---------------//
  // Write to DMAy Channelx CPAR //
    DMA1_Channel4->CPAR = (uint32_t) &(USART1->TDR);
//--------------------------- DMAy Channelx CMAR Configuration ---------------//
  // Write to DMAy Channelx CMAR //
    DMA1_Channel4->CMAR = (uint32_t) command;
    USART1->CR3 |= (uint32_t)(USART_CR3_DMTX | USART_CR3_DMRX);	// ON DMA
		DMA1_Channel4->CCR |= DMA_CCR_EN;
		DMA1_Channel5->CCR |= DMA_CCR_EN;
    while (DMA_GetFlagStatus(DMA1_FLAG_TC5) == RESET) { 
      cnt++; delay_us(25); if(cnt>20000)break;
		};
}

void wr_byteOW(uint8_t bt) {																		//запись байта bt для устройства 1-wire
		wr_bitOW(bt, 8);
}

static void init_readOW(uint16_t len) {
	  uint16_t cnt = 0;
	  uint32_t tmpreg;
	
    for(uint8_t i = 0; i<len; i++){	command[i] = 0xff; }
    DMA_DeInitCH(DMA1_Channel5);	
    DMA_DeInitCH(DMA1_Channel4);	
		
    tmpreg = DMA1_Channel5->CCR;
  // Clear MEM2MEM, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits //
    tmpreg &= CCR_CLEAR_MASK;
    tmpreg |= DMA_DIR_PeripheralSRC | DMA_Mode_Normal |
            DMA_PeripheralInc_Disable | DMA_MemoryInc_Enable |
            DMA_PeripheralDataSize_Byte | DMA_MemoryDataSize_Byte |
            DMA_Priority_Low | DMA_M2M_Disable;
  // Write to DMAy Channelx CCR //
    DMA1_Channel5->CCR = tmpreg;
//--------------------------- DMAy Channelx CNDTR Configuration --------------//
  // Write to DMAy Channelx CNDTR //
    DMA1_Channel5->CNDTR = len;
//--------------------------- DMAy Channelx CPAR Configuration ---------------//
  // Write to DMAy Channelx CPAR //
    DMA1_Channel5->CPAR = (uint32_t) &(USART1->RDR);
//--------------------------- DMAy Channelx CMAR Configuration ---------------//
  // Write to DMAy Channelx CMAR //
    DMA1_Channel5->CMAR = (uint32_t) buf_d;
    tmpreg = DMA1_Channel4->CCR;
  // Clear MEM2MEM, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits //
    tmpreg &= CCR_CLEAR_MASK;
    tmpreg |= DMA_DIR_PeripheralDST | DMA_Mode_Normal |
            DMA_PeripheralInc_Disable | DMA_MemoryInc_Enable |
            DMA_PeripheralDataSize_Byte | DMA_MemoryDataSize_Byte |
            DMA_Priority_Low | DMA_M2M_Disable;
  // Write to DMAy Channelx CCR //
    DMA1_Channel4->CCR = tmpreg;
//--------------------------- DMAy Channelx CNDTR Configuration --------------//
  // Write to DMAy Channelx CNDTR //
    DMA1_Channel4->CNDTR = len;
//--------------------------- DMAy Channelx CPAR Configuration ---------------//
  // Write to DMAy Channelx CPAR //
    DMA1_Channel4->CPAR = (uint32_t) &(USART1->TDR);
//--------------------------- DMAy Channelx CMAR Configuration ---------------//
  // Write to DMAy Channelx CMAR //
    DMA1_Channel4->CMAR = (uint32_t) command;

    USART1->CR3 |= (uint32_t)(USART_CR3_DMTX | USART_CR3_DMRX);	// ON DMA
		DMA1_Channel4->CCR |= DMA_CCR_EN;
		DMA1_Channel5->CCR |= DMA_CCR_EN;		
    while (DMA_GetFlagStatus(DMA1_FLAG_TC5) == RESET) { 
      cnt++; delay_us(25); if(cnt>20000)break;
		};
}

uint8_t rdbit_dma_OW(uint8_t len) {						// битовое чтение для устройства 1-wire, до 8 бит читать	
	  uint8_t dat, out = 0;
		init_readOW(len);

		for(uint8_t i = 0; i<len; i++) {
				if(buf_d[i] == 0xff)dat = 1;
				else dat = 0;
				out += dat << i;
		}
		out_DMA[0] = out;
		if(buf_d[1] == 0xff) out_DMA[1] = 1;
		else out_DMA[1] = 0;
		return out;			
}

void rd_dma_OW(uint8_t bt){												//чтение bt байт
    uint8_t len = 8*bt;
    uint8_t dat, out;
	
    init_readOW(len);
    for(uint8_t i = 0; i<len/8; i++){
		    out = 0;
			    for(uint16_t j =0; j<8; j++){
				    if(buf_d[j+i*8] == 0xff)dat = 1;
				    else dat = 0;
				  out += dat << j;
			    }
			  out_DMA[i] = out;
		}
}

uint8_t iButtonCRC( uint8_t *code, uint8_t numb){   /// подсчет контрольной суммы,  *code - массив, numb - количество байт 
    uint8_t j, i, Data, tmp, sumCRC = 0;
	
	  for (j = 0; j < numb; j++) {
		    Data = code[j];
		    for (i = 0; i < 8; i++) {
			      tmp = 1 & (Data ^ sumCRC);
			      sumCRC >>= 1;
			      Data >>= 1;
			      if ( 0 != tmp ) sumCRC ^= 0x8c;
		    }
	  }
	  return sumCRC;
}

uint8_t read_key(void){							//чтение ROM устройства при подключении к шине 1-го устройства, результат в out_DMA
    uint8_t ok = 1, i;
	
    for(i = 0; i<8; i++){ out_DMA[i] = 0; }
    if(reset_OW()) {										// если на приветствие есть отклик продолжаем работу
		    wr_byteOW(com_Read_ROM);				// записываем команду $33 – "чтение ROM", команда используется, если мы точно знаем, что у нас только одно подчинённое устройство
		    rd_dma_OW(8);										// чтение 8 байт от устройства
    }
	  else { ok = 0;}
	  return ok;
}

///////////////////// поиск устройств на шине
uint8_t search_OW_dev(void){
	  int16_t LastFork = -1;				// уровень, на котором расположена последняя развилка, из которой мы поворачивали влево при предыдущем проходе по дереву (в этот раз на ней нужно повернуть направо);
	  int16_t ZeroFork = -1; 				// сюда пишем уровни всех развилок, в которых мастер при текущем проходе по дереву отвечал нулём (поворачивал налево), в конце прохода отсюда мы узнаем уровень последней такой развилки.
	  uint8_t step = 0;							// переменная, показывающая, на каком шаге поиска мы находимся 
	  uint8_t MasterBit = 0; 			// бит выставляемый мастером для дальнейшего направления поиска
//	  uint16_t LastFamilyFork = 0;	// здесь хранится уровень последней развилки, относящейся к семейству устройств (младший байт ROM)
	  uint16_t count_dev = 0;				// количество найденных на шине девайсов
	  uint8_t answ = 0; 						// ответ от устройств на шине	
	  uint8_t ROM[64];							// массив для записи ключа
	  uint64_t bit = 0;
	  bool LastDev =	false;				// флаг, выставляется при нахождении последнего девайса на шине
	
	  for(step = 0; step<64; step++){ ROM[step] = 0; } 
	
		while (!LastDev) {
		    if(reset_OW()){
				    wr_byteOW(com_Search_ROM); 
				    ZeroFork = -1;
///////////////////////////////////////
				    for(step = 0; step<64; step++) {
					      answ = rdbit_dma_OW(2);
                switch(answ) {
						        case 0:
							          if(step == LastFork) {
								            MasterBit = 1; 
							          } 
												else {
								            if(step > LastFork) {
									              MasterBit = 0;
														}
														else {
									              MasterBit = ROM[step];
								            }
								            if(MasterBit == 0) {
									              ZeroFork = step;
									              if(step<8){ 
//																    LastFamilyFork = step; 
																}
								            }
							          }
							          wr_bitOW(MasterBit, 1); 
						        break;
						        case 1:
							          MasterBit = 1; wr_bitOW(MasterBit, 1); 
						        break;
						        case 2:
							          MasterBit = 0; wr_bitOW(MasterBit, 1); 
						        break;
						        case 3:
							          LastDev = true;
						        break;				
                }
					      ROM[step] = MasterBit;
				    }
				    LastFork = ZeroFork;
					  if(LastFork<0){ LastDev = true; }
					  else{}
		    } 
        else{ LastDev = true; }	
			  key[count_dev] = 0;
			  for(step = 0; step<64; step++){ bit = ROM[step]; key[count_dev] += bit<< step; }  /// сохраняем вновь найденный ключ устр. в массиве key[100]
			  count_dev++;
		}
	  return count_dev;
}

void rw1990_write(uint8_t data){
    for(uint8_t i=0;i<8;i++) {
        wr_bitOW((data & 0x01), 1);
        data>>=1;
        delay_ms(12);
    }
}

void write_TEG1990_1(uint8_t* dim, uint8_t* buffer, bool vers) { //// запись болванки домофонного ключа
    bool writeEN = false;
    if(reset_OW()) {
		    if(vers) {
				    wr_byteOW(com_write1);
				    wr_bitOW(0, 1);							////disable write protect 
			    }
        else {
				    wr_byteOW(com_write2);
				    wr_bitOW(1, 1);							////disable write protect 
        }
        delay_ms(10);
        if(reset_OW()) {
				    if(vers){
                wr_byteOW(com_wrEN_1);
                rd_dma_OW(1);
                buffer[8] = out_DMA[0];
                if(buffer[8] != 0xff){ led_g_on(); }
                else { led_g_off(); writeEN = true; }
            }
            else {
                wr_byteOW(com_wrEN_2);
                rd_dma_OW(1);
                buffer[8] = out_DMA[0];
                if(buffer[8] != 0xfe){ led_g_on(); }
                else { led_g_off(); writeEN = true; }
            }
        }		
        if(writeEN) {			
				    if(reset_OW()) {
                wr_byteOW(com_write3);
//                __disable_irq();	
                for(uint8_t i=0;i<8;i++) {
                    rw1990_write(~dim[i]);
                }
//					__enable_irq();
            }
            if(reset_OW()) {
                if(vers) {
						        wr_byteOW(com_write1);
						        wr_bitOW(1, 1);								//// protect write
                }
                else {
                    wr_byteOW(com_write2);				
                    wr_bitOW(0, 1);								//// protect write
                }
            }	
        }	
        delay_ms(20);			
        if(read_key()) {
				    for(uint8_t i = 0; i<8; i++){ buffer[i+4] = out_DMA[7-i]; }
				    buffer[8] = out_DMA[7];	
        }				
    }
}

uint8_t status_sens(uint8_t* buffer) {
    uint8_t b = 0;
    if(read_key()) {
        if(out_DMA[7] == iButtonCRC(out_DMA, 7)) {
            for(uint8_t i = 0; i<8; i++) { buffer[i+4] = out_DMA[7-i]; }
//				buffer[8] = out_DMA[7];	
            }
    }
		delay_ms(1);	
		return b;
}

void delay_ms(uint32_t us) {
    delay_us(us*800);
}
