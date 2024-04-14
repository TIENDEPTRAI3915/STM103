#ifndef UART_H
#define UART_H
#include "stm32f10x.h"
void UART_CONFIG(void);
void UART_TX (char c);
void UART_TXstring(char *c);
char UART_RXchar(void);

#endif

