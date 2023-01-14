#ifndef _PROG_I2C_H
#define _PROG_I2C_H

#include "system.h"
#include "platform.h"
#include "stm32.h"

#define adr_MCP47A1         0x5c
#define I2C_SDA_PIN         3
#define I2C_SCL_PIN         2
#define I2C_SDA_PORT        GPIOA
#define I2C_SCL_PORT        GPIOA



void delay_us (uint32_t us);
void init_i2c (void);
uint8_t I2C_Write_Byte (uint8_t data);
void I2C_Write_Block (uint8_t address_dev, uint8_t length, uint8_t* I2C_Buffer);
uint8_t I2C_Read_Byte (uint8_t ACK);
void I2C_Read_Block (uint8_t address_dev, uint8_t address_reg, uint8_t length, uint8_t* I2C_Buffer);
void write_byte_DEV (uint8_t adr_dev, uint8_t address_reg, uint8_t val) ;
uint8_t read_byte_DEV (uint8_t adr_dev, uint8_t address_reg);

#endif
