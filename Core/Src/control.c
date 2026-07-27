#include "control.h"
#include "bsp_pid.h"
#include <math.h>
#include <stdlib.h>
#include "protocol.h"
#include "OLED.h"

/* 私有变量 */
__IO float cont_rpm1 = 0.0f;    // 通道1控制值 rpm
__IO float cont_rpm2 = 0.0f;    // 通道2控制值 rpm
__IO float cont_rpm3 = 0.0f;    // 通道3控制值 rpm
__IO float cont_rpm4 = 0.0f;    // 通道4控制值 rpm

/* 局部函数 */
extern void set_rpm(uint8_t temp_cc, float rpm);

void bldc_pid_control(uint8_t c,uint32_t CCR_num)
{
	switch(c){
		case 1:
		{	
			pid_1.actual_rpm=(float)(3.6e8/(double)CCR_num);//计算实际转速 RPM
			/*误差小于0.1时启用pid*/
			pid_1.err = pid_1.target_rpm - pid_1.actual_rpm;
			if(Ctrl_Para.enable_pid[c]==0 && Ctrl_Para.enable_pid[0] == 1)
			{
				if(10.0f*fabsf(pid_1.err)<pid_1.target_rpm)
				{
					Ctrl_Para.enable_pid[c] = 1;
				}
			}
			/*PID计算*/
			if(Ctrl_Para.enable_fan[c] && Ctrl_Para.enable_pid[c] && Ctrl_Para.enable_pid[0])
			{	
				cont_rpm1 += PID_realize(&pid_1);	
				cont_rpm1 = ((cont_rpm1 > Sys_Para.Speed_MX_Fan) ? Sys_Para.Speed_MX_Fan : cont_rpm1);  // 上限处理
				cont_rpm1 = ((cont_rpm1 <0) ? 0 : cont_rpm1);  // 下限处理
				set_rpm(c, cont_rpm1);
			}
			//OLED_ShowNum(1, 7, (uint16_t)pid_1.actual_rpm, 5);
			if(Sys_Para.enable_send == 1)
			{
				send_computer(0x02, 0x01, &pid_1.actual_rpm, 4);     // 给通道 1 发送实际值
//				if(Ctrl_Para.enable_pid[c] == 1)
//				{
//					float temp11 = cont_rpm1;
//					send_computer(0x01, 0x01, &temp11, 4);     // 给通道 1 发送控制值
//				}
			}					
		}break;
		case 2:
		{
			//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
			pid_2.actual_rpm=3.6e8f/CCR_num;//计算实际转速 RPM
			/*误差小于0.1时启用pid*/
			pid_2.err = pid_2.target_rpm - pid_2.actual_rpm;
			if(Ctrl_Para.enable_pid[c]==0 && Ctrl_Para.enable_pid[0] == 1)
			{
				if(10.0f * fabsf(pid_2.err) < pid_2.target_rpm)
				{
					Ctrl_Para.enable_pid[c] = 1;
				}
			}
			/*PID计算*/
			if(Ctrl_Para.enable_fan[c] && Ctrl_Para.enable_pid[c] && Ctrl_Para.enable_pid[0])
			{
				cont_rpm2 += PID_realize(&pid_2);	
				cont_rpm2 = (cont_rpm2 > Sys_Para.Speed_MX_Fan) ? Sys_Para.Speed_MX_Fan : cont_rpm2;  // 上限处理
				cont_rpm2 = (cont_rpm2 <0) ? 0 : cont_rpm2;  // 下限处理
				set_rpm(2, cont_rpm2);
			}
			//OLED_ShowNum(2, 7, (uint16_t)pid_2.actual_rpm, 5);
			if(Sys_Para.enable_send == 1)
			{
				send_computer(0x02, 0x02, &pid_2.actual_rpm, 4);     // 给通道 2 发送实际值
				if(Ctrl_Para.enable_pid[c] == 1)
				{
					float temp21 = cont_rpm2;
					send_computer(0x01, 0x02, &temp21, 4);     // 给通道 2 发送控制值
				}
			}
		}break;
		case 3:
		{	
			pid_3.actual_rpm = 3.6e8f / CCR_num;//计算实际转速 RPM
			/*误差小于0.1时启用pid*/
			pid_3.err = pid_3.target_rpm - pid_3.actual_rpm;
			if(Ctrl_Para.enable_pid[c] == 0 && Ctrl_Para.enable_pid[0] == 1)
			{
				if(10.0f*fabsf(pid_3.err)<pid_3.target_rpm)
				{
					Ctrl_Para.enable_pid[c]=1;
				}
			}
			/*PID计算*/
			if(Ctrl_Para.enable_fan[c] && Ctrl_Para.enable_pid[c] && Ctrl_Para.enable_pid[0])
			{	
				cont_rpm3 += PID_realize(&pid_3);	
				cont_rpm3 = ((cont_rpm3 > Sys_Para.Speed_MX_Fan) ? Sys_Para.Speed_MX_Fan : cont_rpm3);  // 上限处理
				cont_rpm3 = ((cont_rpm3 <0) ? 0 : cont_rpm3);  // 下限处理
				set_rpm(c, cont_rpm3);
			}
			OLED_ShowNum(1, 7, (uint16_t)pid_3.actual_rpm, 5);
			if(Sys_Para.enable_send == 1)
			{
				send_computer(0x02, 0x03, &pid_3.actual_rpm, 4);     // 给通道 3 发送实际值
				if(Ctrl_Para.enable_pid[c] == 1)
				{
					float temp31 = cont_rpm3;
					send_computer(0x01, 0x03, &temp31, 4);     // 给通道 3 发送控制值
				}
			}
		}break;
		case 4:
		{	
			pid_4.actual_rpm=3.6e8f/CCR_num;//计算实际转速 RPM
			/*误差小于0.1时启用pid*/
			pid_4.err = pid_4.target_rpm-pid_4.actual_rpm;
			if(Ctrl_Para.enable_pid[c]==0 && Ctrl_Para.enable_pid[0] == 1)
			{
				if(10.0f*fabsf(pid_4.err)<pid_4.target_rpm)
				{
					Ctrl_Para.enable_pid[c]=1;
				}
			}
			/*PID计算*/
			if(Ctrl_Para.enable_fan[c] && Ctrl_Para.enable_pid[c] && Ctrl_Para.enable_pid[0])
			{	
				cont_rpm4 += PID_realize(&pid_4);	
				cont_rpm4 = ((cont_rpm4 > Sys_Para.Speed_MX_Fan) ? Sys_Para.Speed_MX_Fan : cont_rpm4);  // 上限处理
				cont_rpm4 = ((cont_rpm4 <0) ? 0 : cont_rpm4);  // 下限处理
				set_rpm(c, cont_rpm4);
			}
			OLED_ShowNum(2, 7, (uint16_t)pid_4.actual_rpm, 5);
			if(Sys_Para.enable_send == 1)
			{
				send_computer(0x02, 0x04, &pid_4.actual_rpm, 4);     // 给通道 3 发送实际值
				if(Ctrl_Para.enable_pid[c] == 1)
				{
					float temp31 = cont_rpm4;
					send_computer(0x01, 0x04, &temp31, 4);     // 给通道 3 发送控制值
				}
			}					
		}break;
		default: break;
	}
}
/*目标值改变时PID初始化*/
void pid_reset(uint8_t ccc,uint16_t target_t)
{
	
	Ctrl_Para.enable_pid[ccc] = 0;
	switch(ccc){
			case 1:
			{
				cont_rpm1 = target_t;
				pid_1.target_rpm = target_t;
				pid_1.err = 0.0;
				pid_1.err_last = 0.0;
				pid_1.err_next = 0.0;
			}break;
			case 2:
			{
				cont_rpm2 = target_t;
				pid_2.target_rpm = target_t;
				pid_2.err = 0.0;
				pid_2.err_last = 0.0;
				pid_2.err_next = 0.0;
			}break;
			case 3:
			{
				cont_rpm3 = target_t;
				pid_3.target_rpm = target_t;
				pid_3.err = 0.0;
				pid_3.err_last = 0.0;
				pid_3.err_next = 0.0;
			}break;
			case 4:
			{
				cont_rpm4 = target_t;
				pid_4.target_rpm = target_t;
				pid_4.err = 0.0;
				pid_4.err_last = 0.0;
				pid_4.err_next = 0.0;
			}break;
			default: break;
	}
}
