#ifndef CONTROL_H
#define	CONTROL_H

#include "stm32f1xx.h"
#include "main.h"

extern __IO float rpm1_actual;
extern __IO float rpm2_actual;

extern void pid_control(uint8_t c, uint32_t CCR_num);
extern void pid_reset(uint8_t c, uint16_t target_t);

#endif
