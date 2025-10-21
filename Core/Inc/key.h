#ifndef __KEY_H__
#define __KEY_H__

#include "main.h"

void key_Tick(void);
uint8_t Key_GetNum(void);
uint8_t Key_GetState(void);

extern volatile uint8_t Key_Num;

#endif

