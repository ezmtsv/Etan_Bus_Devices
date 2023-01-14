#include "prog_I2C.h"



uint8_t TimeX = 8;



void init_i2c(void) {
  _BST (RCC->AHBENR, RCC_AHBENR_GPIOAEN);
  I2C_SDA_PORT->MODER |= GPIO_OUT (I2C_SDA_PIN) | GPIO_OUT (I2C_SCL_PIN);
  GPIOA->OTYPER |= GPIO_ODN (I2C_SDA_PIN) | GPIO_ODN (I2C_SCL_PIN);
}



static void I2C_Start(void) {
  I2C_SDA_PORT->BSRR |= (1<<I2C_SDA_PIN);    //_BV (I2C_SDA_PIN);  //
  delay_us (TimeX);
  I2C_SCL_PORT->BSRR |= (1<<I2C_SCL_PIN);    //_BV (I2C_SCL_PIN);  //
  delay_us (TimeX);
  while (!(I2C_SDA_PORT->IDR & (1<<I2C_SDA_PIN))) {
    I2C_SCL_PORT->BRR  |= (1<<I2C_SCL_PIN);
    delay_us (TimeX);
    I2C_SCL_PORT->BSRR |= (1<<I2C_SCL_PIN);
    delay_us (TimeX);
  }
  I2C_SDA_PORT->BRR |= (1<<I2C_SDA_PIN);
  delay_us (TimeX);
  I2C_SCL_PORT->BRR |= (1<<I2C_SCL_PIN);
  delay_us (TimeX);
}



static void I2C_Stop (void) {
  I2C_SDA_PORT->BRR |= (1<<I2C_SDA_PIN);
  delay_us (TimeX);
  I2C_SCL_PORT->BSRR |= (1<<I2C_SCL_PIN);
  delay_us (TimeX);
  I2C_SDA_PORT->BSRR |= (1<<I2C_SDA_PIN);
  delay_us (TimeX);
}



uint8_t I2C_Write_Byte (uint8_t data) {
  uint8_t i;
  uint8_t ACK;
  for (i=0; i<8; i++) {
    if (data & 0x80) {
      I2C_SDA_PORT->BSRR |= (1<<I2C_SDA_PIN);
    } else {
      I2C_SDA_PORT->BRR |= (1<<I2C_SDA_PIN);
    }
    delay_us (TimeX);
    I2C_SCL_PORT->BSRR |= (1<<I2C_SCL_PIN);
    delay_us (TimeX);
    I2C_SCL_PORT->BRR |= (1<<I2C_SCL_PIN);
    data = data<<1;
  }
  delay_us (TimeX);
  I2C_SCL_PORT->BSRR |= (1<<I2C_SCL_PIN);
  delay_us (TimeX);
  ACK = !(I2C_SDA_PORT->IDR & (1<<I2C_SDA_PIN));
  I2C_SCL_PORT->BRR |= (1<<I2C_SCL_PIN);
  I2C_SDA_PORT->BRR |= (1<<I2C_SDA_PIN);
  return ACK;
}



void I2C_Write_Block (uint8_t address_dev, uint8_t length, uint8_t* I2C_Buffer) {
  uint8_t i ;
  I2C_Start();
  I2C_Write_Byte (address_dev);
  for (i=0; i<length; i++) {
    I2C_Write_Byte (I2C_Buffer[i]);
  }
  I2C_Stop();
}



uint8_t I2C_Read_Byte (uint8_t ACK) {
  uint8_t i;
  uint8_t data = 0;
  I2C_SDA_PORT->BSRR |= (1<<I2C_SDA_PIN);
  for (i=0;i<8;i++) {
    delay_us (TimeX);
    I2C_SCL_PORT->BSRR |= (1<<I2C_SCL_PIN);
    delay_us (TimeX);
    data<<=1;
    if (I2C_SDA_PORT->IDR & (1<<I2C_SDA_PIN)) data++;
    I2C_SCL_PORT->BRR |= (1<<I2C_SCL_PIN);
  }
  if (ACK) I2C_SDA_PORT->BRR |= (1<<I2C_SDA_PIN);
  delay_us (TimeX);
  I2C_SCL_PORT->BSRR |= (1<<I2C_SCL_PIN);
  delay_us (TimeX);
  I2C_SCL_PORT->BRR |= (1<<I2C_SCL_PIN);
  I2C_SDA_PORT->BSRR |= (1<<I2C_SDA_PIN);
  return data;
}



void I2C_Read_Block (uint8_t address_dev, uint8_t address_reg, uint8_t length, uint8_t* I2C_Buffer) {
  uint8_t i ;
  I2C_Start();
  I2C_Write_Byte (address_dev);
  I2C_Write_Byte (address_reg);
  I2C_Start();
  I2C_Write_Byte (address_dev+1);
  for (i=0; i<length; i++) {
    I2C_Buffer[i]  = I2C_Read_Byte (i!=(length-1));
  }
  I2C_Stop();
}



uint8_t read_byte_DEV (uint8_t adr_dev, uint8_t address_reg) {
  uint8_t buf[3];
  I2C_Read_Block (adr_dev, address_reg, 1, buf);
  return buf[0];
}



void write_byte_DEV (uint8_t adr_dev, uint8_t address_reg, uint8_t val) {
  uint8_t buf[2];
  buf[0] = address_reg;
  buf[1] = val;
  I2C_Write_Block (adr_dev, 2, buf);
}
