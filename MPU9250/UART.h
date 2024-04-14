#ifndef UART_H
#define UART_H
#include "stm32f10x.h"
void UARTinit(void);
void gui(uint8_t c);
void guichu(char *text);
#endif
