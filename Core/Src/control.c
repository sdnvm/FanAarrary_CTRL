#include "control.h"
#include "bsp_pid.h"
#include <math.h>
#include <stdlib.h>
#include "can.h"
#include "OLED.h"

/* 私有变量 */
static float cont_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};    // 通道0-3计算得的控制值 rpm
/* 局部函数 */
extern void set_rpm(uint8_t temp_cc, float rpm);

/*控制量计算*/
void pid_control(const uint8_t c, const uint32_t CCR_num)
{
	pid_para[c].actual_rpm = (float)(3.6e8 / (double)CCR_num); //计算实际转速 RPM
	if ((uint32_t)pid_para[c].actual_rpm == 65535)pid_para[c].actual_rpm = 0.0f; //溢出处理
	pid_para[c].err = pid_para[c].target_rpm - pid_para[c].actual_rpm;//计算偏差
	/*PID计算*/
	if (Ctrl_Para.enable_fan[c + 1] && Ctrl_Para.enable_pid[0])
	{
		cont_rpm[c] += PID_realize(&pid_para[c]); //增量式pid计算
		cont_rpm[c] = ((cont_rpm[c] > FANSPEED_MAX) ? FANSPEED_MAX : cont_rpm[c]); // 上限处理
		cont_rpm[c] = ((cont_rpm[c] < 0) ? 0 : cont_rpm[c]); // 下限处理
		set_rpm(c, cont_rpm[c]);
		//return;
	}
	//误差小于0.1时启用pid
	if (Ctrl_Para.enable_pid[c + 1] == 0 && Ctrl_Para.enable_pid[0] == 1)
	{
		if (10.0f * fabsf(pid_para[c].err) < pid_para[c].target_rpm)
		{
			Ctrl_Para.enable_pid[c + 1] = 1;
		}
	}
	MyCAN_Transmit_RPM(pid_para[c].actual_rpm, cont_rpm[c], c);
	OLED_ShowNum(3, 1 + c * 4, (uint16_t)pid_para[c].actual_rpm, 4);
}

/*目标值改变时PID初始化*/
void pid_reset(const uint8_t c, const float target_t)
{
	Ctrl_Para.enable_pid[c+1] = 0;//复位子开关
	cont_rpm[c] = target_t;//强制传入实际值
	pid_para[c].target_rpm = target_t;//更新目标值
	pid_para[c].err = 0.0;
	pid_para[c].err_last = 0.0;
	pid_para[c].err_next = 0.0;
}
/*设定目标值，并重置pid*/
void set_rpm_incloseloopmode(const uint8_t c_p, const float rpm_p)
{
	set_rpm(c_p, rpm_p);
	pid_reset(c_p, rpm_p);
}