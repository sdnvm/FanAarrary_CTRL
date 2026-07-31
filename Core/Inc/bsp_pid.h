#ifndef __BSP_PID_H
#define	__BSP_PID_H
#include "stm32f1xx.h"
#include <stdio.h>
#include <stdlib.h>

typedef struct
{
	float target_rpm; //目标值
	float actual_rpm; //实际值
	float err; //当前偏差值
	float err_next; //下一个偏差值
	float err_last; //上一个偏差值
	float Kp, Ki, Kd;
} PidPara;

extern PidPara pid_para[4];

extern void PID_param_init(void);
extern void set_pid_target(uint8_t c, float temp_target_rpm);
extern void set_p_i_d(uint8_t c, float p, float i, float d);
extern float PID_realize(PidPara *pid);

// extern float get_pid_actual(PidPara *pid);
// extern void time_period_fun(void);

#endif
