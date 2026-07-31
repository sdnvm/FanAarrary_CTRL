#include "bsp_pid.h"
#include "math.h"
#include "protocol.h"
// 定义全局变量 */
PidPara pid_para[4];

void PID_param_init()//初始化pid参数
{
	for(int i=0;i<4;i++)
	{
		pid_para[i].target_rpm=0.0;
		pid_para[i].actual_rpm=0.0;
		pid_para[i].err = 0.0;
		pid_para[i].err_last = 0.0;
		pid_para[i].err_next = 0.0;
		pid_para[i].Kp = 0.1;
		pid_para[i].Ki = 0.035;
		pid_para[i].Kd = 0.27;
	}
}

void set_pid_target(uint8_t c, float temp_target_rpm) // 更新目标值
{
	c = c - 1;
	pid_para[c].target_rpm = temp_target_rpm;
}

void set_p_i_d(uint8_t c, float p, float i, float d)//更新pid参数
{
	c = c - 1;
	pid_para[c].Kp = p;
	pid_para[c].Ki = i;
	pid_para[c].Kd = d;
}

float PID_realize(PidPara *pid) //增量式pid计算
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
