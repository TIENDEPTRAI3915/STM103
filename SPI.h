#ifndef SPI_H
#define SPI_H
#include "stm32f10x.h"
void SPIinit(void);
void SPI_gui(uint8_t c);
void SPI_write(uint8_t data,uint8_t len);
void SPI_read(uint8_t *array ,uint8_t len);
void SPI_MPU_read_lientuc(uint8_t data,uint8_t *array ,uint8_t len);
void SPI_MPU_write (uint8_t reg,uint8_t data);
void SPI_led_write(uint8_t data,uint8_t reg);
#endif