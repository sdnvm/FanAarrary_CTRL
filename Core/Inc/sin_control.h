#ifndef __SINCONTROL_H
#define __SINCONTROL_H

#include "stm32f1xx.h"
#include <math.h>
#include "main.h"

// ??????????
typedef struct
{
	float mid; // 中值（）
	int period; // 周期(单位:ms)
	float amplitude; //幅值
	uint16_t ctrl_num;
	uint16_t sampl_num;
} SinParams;

#define M_PAI 3.141593f
#define STEP 0.5f
#define SIN_SAMPLE_NUM 2

// ??????
extern uint8_t sin_point_index; // ?????????
extern SinParams Sin_Para1;
// ????
void cacul_sin_params();
void TrackCurve();
#endif
