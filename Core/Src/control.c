#include "control.h"
#include "bsp_pid.h"
#include <math.h>
#include <stdlib.h>
#include "protocol.h"
#include "OLED.h"

/* 私有变量 */
__IO float cont_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};    // 通道1-4控制值 rpm
/* 局部函数 */
extern void set_rpm(uint8_t temp_cc, float rpm);

/*上传数据*/
void send2pc_optional(uint8_t cc)
{
	if (Sys_Para.enable_send == 1)
	{
		send_computer(0x02, 0x01, &pid_para[cc].actual_rpm, 4); // 给通道 1 发送实际值
		if (Ctrl_Para.enable_pid[cc] == 1)
		{
			float temp11 = cont_rpm[cc];
			send_computer(0x01, 0x01, &temp11, 4); // 给通道 1 发送控制值
		}
	}
}

/*控制量计算*/
void pid_control(uint8_t c, uint32_t CCR_num)
{
	pid_para[c].actual_rpm = (float)(3.6e8 / (double)CCR_num); //计算实际转速 RPM
	/*误差小于0.1时启用pid*/
	pid_para[c].err = pid_para[c].target_rpm - pid_para[c].actual_rpm;

	/*PID计算*/
	if (Ctrl_Para.enable_fan[c] && Ctrl_Para.enable_pid[0])
	{
		cont_rpm[c] += PID_realize(&pid_para[c]);
		cont_rpm[c] = ((cont_rpm[c] > Sys_Para.Speed_MX_Fan) ? Sys_Para.Speed_MX_Fan : cont_rpm[c]); // 上限处理
		cont_rpm[c] = ((cont_rpm[c] < 0) ? 0 : cont_rpm[c]); // 下限处理
		set_rpm(c, cont_rpm[c]);
		send2pc_optional(c);
		return;
	}
	if (Ctrl_Para.enable_pid[c] == 0 && Ctrl_Para.enable_pid[0] == 1)
	{
		if (10.0f * fabsf(pid_para[c].err) < pid_para[c].target_rpm)
		{
			Ctrl_Para.enable_pid[c] = 1;
		}
	}
	//OLED_ShowNum(1, 7, (uint16_t)pid_para[c].actual_rpm, 5);

}

/*目标值改变时PID初始化*/
void pid_reset(uint8_t c,uint16_t target_t)
{
	Ctrl_Para.enable_pid[c] = 0;
	c = c - 1;
	cont_rpm[c] = target_t;
	pid_para[c].target_rpm = target_t;
	pid_para[c].err = 0.0;
	pid_para[c].err_last = 0.0;
	pid_para[c].err_next = 0.0;
}
