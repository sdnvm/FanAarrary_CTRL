#ifndef CONTROL_H
#define	CONTROL_H

#include "stm32f1xx.h"
#include "main.h"

extern __IO float rpm1_actual;
extern __IO float rpm2_actual;

typedef struct
{
  uint16_t dutyfactor;      // PWM 输出占空比
  uint8_t is_enable;        // 使能电机
  uint32_t lock_timeout;    // 电机堵转计时
}bldcm_data_t;

extern void bldc_pid_control(uint8_t c,uint32_t CCR_num);
extern void pid_reset(uint8_t ccc,uint16_t target_t);

#endif 
