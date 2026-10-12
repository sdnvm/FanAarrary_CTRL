#include "sin_control.h"
#include "control.h"
#include <math.h>
#include "OLED.h"
SinParams Sin_Para1;
float sin_control_values[MAX_SIN_POINTS];
uint8_t sin_point_index = 0;
// 更新正弦波参数并启动分批计算
void cacul_sin_params(void) 
{
	TrackCurve_Para.ctrl_num = Sin_Para1.ctrl_num;
	Sin_Para1.sampl_num = (uint16_t)(Sin_Para1.period / (Sin_Para1.ctrl_num*Sys_Para.timer_ctrl_T));	//tim单位都是ms
	sin_point_index = 0;	// 重置使用索引
				
	// 确保点数不超过最大值
	if (Sin_Para1.sampl_num > MAX_SIN_POINTS) 
	{
			Sin_Para1.sampl_num = MAX_SIN_POINTS;
	}
	// 确保至少有4个点
	else if (Sin_Para1.sampl_num < 4) 
	{
			Sin_Para1.sampl_num = 4;
	}
  for (uint8_t j = 0; j < Sin_Para1.sampl_num ; j++) 
	{
        const float step = 2 * M_PAI / Sin_Para1.sampl_num; // 将一个T分为n_points个点
        float angle = j * step;
        float sin_value = sin(angle);
        
        // 使用中值+增益*正弦值
        sin_control_values[j] = Sin_Para1.mid + Sin_Para1.amplitude * sin_value;
        
        // 限幅
        if (sin_control_values[j] > Sys_Para.Speed_MX_Fan) 
				{
            sin_control_values[j] = Sys_Para.Speed_MX_Fan;
        }
        else if (sin_control_values[j] < 1800) 
				{
            sin_control_values[j] = 1800;
        }	
	}
	TrackCurve_Para.enable_TrackCurve = 1;
	OLED_ShowString(4, 16, "O");
	return;
}
