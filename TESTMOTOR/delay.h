#ifndef delay_H
#define delay_H
#include "stm32f10x.h"
void delayinit(void);
void delayms(uint16_t timems);
void delayus(uint16_t timeus);
#endif
