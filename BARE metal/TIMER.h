#ifndef TIMER_H
#define TIMER_H
#include "stm32f10x.h"
void TIMinit(void);
__STATIC_INLINE void TIMER_DELAYUS(uint16_t count);
void TIMER_DELAYMS(uint16_t count);
void systick_config(void);
void sysdelay(uint32_t count);
#endif
