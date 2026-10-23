#include "sin_control.h"
#include "control.h"
#include "OLED.h"

SinParams Sin_Para1;
uint8_t sin_point_index = 0;
// 更新正弦波参数并启动分批计算
void cacul_sin_params()
{
	TrackCurve_Para.ctrl_num = Sin_Para1.ctrl_num;
	Sin_Para1.sampl_num = (uint16_t)(Sin_Para1.period / (Sin_Para1.ctrl_num * Sys_Para.timer_ctrl_T)); //tim单位都是ms
	sin_point_index = 0; // 重置使用索引

	// 确保点数不超过最大值
	if (Sin_Para1.sampl_num > CURVE_SAMPLE_MX)
	{
		Sin_Para1.sampl_num = CURVE_SAMPLE_MX;
	}
	// 确保至少有4个点
	else if (Sin_Para1.sampl_num < 4)
	{
		Sin_Para1.sampl_num = 4;
	}
	for (uint8_t j = 0; j < Sin_Para1.sampl_num; j++)
	{
		const float step = 2 * M_PAI / Sin_Para1.sampl_num; // 将一个T分为n_points个点
		float angle = j * step;
		float sin_value = sin(angle);

		// 使用中值+增益*正弦值
		TrackCurve_Data_p->curve_data[j] = Sin_Para1.mid + Sin_Para1.amplitude * sin_value;

		// 限幅
		if (TrackCurve_Data_p->curve_data[j] > FANSPEED_MAX)
		{
			TrackCurve_Data_p->curve_data[j] = FANSPEED_MAX;
		}
		else if (TrackCurve_Data_p->curve_data[j] < 1800)
		{
			TrackCurve_Data_p->curve_data[j] = 1800;
		}
	}
	TrackCurve_Para.enable_TrackCurve = 1;
	OLED_ShowString(4, 16, "O");
}

/*跟踪曲线控制量更新函数
 * 每次流控定时器中断调用一次，更新目标值
 * sin判断的代码在（**-can文件）里
 */
void TrackCurve()
{
	static uint16_t ctrl_tim = 0; //决定采样时间
	ctrl_tim++; //每次定时器中断+1,决定采样时间
	if (ctrl_tim >= Sin_Para1.ctrl_num && Sys_State.RunMode == S_TrackingCurve)
	{
		if (sin_point_index == Sin_Para1.sampl_num)
		{
			sin_point_index = 0; // 重置索引，为下一个正弦波做准备
		}
		// 从跟踪曲线数据中获取当前点的值
		/*if (c == 1) {
		pid_para[0].target_rpm = sin_control_values[sin_point_index];
		} else if (c == 2) {*/
		for (int i = 0; i <= 4; i++)
		{
			set_rpm_incloseloopmode(i, TrackCurve_Data_p->curve_data[sin_point_index]);
		}
		// }
		sin_point_index++; // 移动到下一个点
		ctrl_tim = 0; // 重置溢出计数器
	}
}
