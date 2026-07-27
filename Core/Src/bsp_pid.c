#include "bsp_pid.h"
#include "math.h"
#include "protocol.h"
// 定义全局变量 */
_pid pid_1,pid_2,pid_3,pid_4;

//float set_point=0.0;
//int pid_status=0;
void PID_param_init()
{
  //初始化通道1 PID参数
  pid_1.target_rpm=0.0;
  pid_1.actual_rpm=0.0;
  pid_1.err = 0.0;
  pid_1.err_last = 0.0;
  pid_1.err_next = 0.0;
  pid_1.Kp = 0.1;
  pid_1.Ki = 0.035;
  pid_1.Kd = 0.27;
//  float pid_1_temp[3] = {pid_1.Kp, pid_1.Ki, pid_1.Kd};
  //send_computer(0x03, 0x01, pid_1_temp, 3);// 向上位机同步实际 P I D 值
  HAL_Delay(10);
  //初始化通道2 PID参数
  pid_2.target_rpm=0.0;				
  pid_2.actual_rpm=0.0;
  pid_2.err = 0.0;
  pid_2.err_last = 0.0;
  pid_2.err_next = 0.0;
  pid_2.Kp = 0.1;
  pid_2.Ki = 0.035;
  pid_2.Kd = 0.27;
 // float pid_2_temp[3] = {pid_2.Kp, pid_2.Ki, pid_2.Kd};
  //send_computer(0x03, 0x02, pid_2_temp, 3);// 向上位机同步实际 P I D 值
	pid_3.target_rpm=0.0;
  pid_3.actual_rpm=0.0;
  pid_3.err = 0.0;
  pid_3.err_last = 0.0;
  pid_3.err_next = 0.0;
  pid_3.Kp = 0.1;
  pid_3.Ki = 0.035;
  pid_3.Kd = 0.27;
	
	pid_4.target_rpm=0.0;
  pid_4.actual_rpm=0.0;
  pid_4.err = 0.0;
  pid_4.err_last = 0.0;
  pid_4.err_next = 0.0;
  pid_4.Kp = 0.1;
  pid_4.Ki = 0.035;
  pid_4.Kd = 0.27;
}
void set_pid_target(uint8_t c, float temp_val)// 更新目标值
{
	switch(c){
		case 0X01:pid_1.target_rpm = temp_val;break;
		case 0X02:pid_2.target_rpm = temp_val;break;
		case 0X03:pid_3.target_rpm = temp_val;break;
		case 0X04:pid_4.target_rpm = temp_val;break;
	}
}

void set_p_i_d(uint8_t ccc, float p, float i, float d)//更新pid参数
{
	switch(ccc){
		case 0X01:{
			pid_1.Kp = p;
			pid_1.Ki = i;
			pid_1.Kd = d;
		}break;
		case 0X02:{
			pid_2.Kp = p;
			pid_2.Ki = i;
			pid_2.Kd = d;
		}break;
		case 0X03:{
			pid_3.Kp = p;
			pid_3.Ki = i;
			pid_3.Kd = d;
		}break;
		case 0X04:{
			pid_4.Kp = p;
			pid_4.Ki = i;
			pid_4.Kd = d;
		}break;
  }
}

float PID_realize(_pid *pid) 
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
