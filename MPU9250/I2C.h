#ifndef I2C_H
#define I2C_H
#include "stm32f10x.h"
void I2Cinit(void);
void I2CWriteLCD(uint8_t address,uint8_t Data);
void LCDIni(void);
void WriteLCD(uint8_t address,uint8_t COMAND_DATA,uint8_t Data);

void I2C_Write1(uint8_t deviceAddr, uint8_t regAddr, uint8_t data);
void I2C_Read1(uint8_t deviceAddr, uint8_t regAddr,uint16_t *diachi,uint8_t len) ;

int I2C_EEPROMWRITE(uint8_t deviceAddr, uint8_t regAddr,uint8_t *diachi,uint8_t len);
int I2C_EEPROMREAD(uint8_t deviceAddr, uint8_t regAddr,uint8_t *diachi,uint8_t len);
#endif
