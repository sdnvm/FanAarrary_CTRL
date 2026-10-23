#include "bsp_pid.h"
#include "math.h"
// 定义全局变量 */
__IO PidPara pid_para[4];//pid参数
/*初始化pid参数*/
void PID_param_init()
{
	for(int i=0;i<4;i++)
	{
		pid_para[i].target_rpm=0.0f;
		pid_para[i].actual_rpm=0.0f;
		pid_para[i].err = 0.0f;
		pid_para[i].err_last = 0.0f;
		pid_para[i].err_next = 0.0f;
		pid_para[i].Kp = 0.1f;
		pid_para[i].Ki = 0.035f;
		pid_para[i].Kd = 0.27f;
	}
}
/*更新目标值*/
void set_pid_target(uint8_t c, float temp_target_rpm)
{
	c = c - 1;
	pid_para[c].target_rpm = temp_target_rpm;
}
/*更新pid参数*/
void set_p_i_d(float p, float i, float d)
{
	for(int j=0;j<4;j++)
	{
		pid_para[j].Kp = p;
		pid_para[j].Ki = i;
		pid_para[j].Kd = d;
	}
}
/*pid开关*/
void pid_switch(uint8_t pidswitch)
{
	switch (pidswitch)
	{
	case 0:
		break;
	case 1:
		break;
	}
}
/*增量式pid计算*/
float PID_realize(__IO PidPara *pid)
{
	//传入实际值
	// pid->actual_rpm = temp_RPM;
	//计算偏差
	// pid->err=pid->target_rpm-pid->actual_rpm;
	//计算校正所需增量
	float add = pid->Kp*(pid->err - pid->err_next) \
	          + pid->Ki*pid->err \
			  + pid->Kd*(pid->err - 2 * pid->err_next + pid->err_last);
	//误差传递
	pid->err_last = pid->err_next;
	pid->err_next = pid->err;
	return add;
}
