#ifndef FA_CTRL_SAVE_FLASH_H
#define FA_CTRL_SAVE_FLASH_H

#include "main.h"

#define SYS_PARA_FLASH_ADDRESS 0x0803F800U//参数存储地址（对于256KB的Flash）
#define SYS_PARA_FLASH_MAGIC   0x53595041U//魔术字：用于判断该页是否保存过有效参数（0xFFFFFFFF 表示空页）
#define SYS_PARA_FLASH_VERSION 1U//结构体版本号：结构体定义变化时修改此值，旧数据会自动失效

/*结构体变量*/
typedef struct
{
	uint32_t ID_Thisboard;		//本机ID
	uint16_t timer_ctrl_T;		//控制定时器溢出时间/ms
	uint16_t timer_fg_f;		//fg捕获定时器时钟/mhz
	double rpm_to_arr;			//转速转换arr
	uint16_t FanSpeed_max;		//风机最大转速
	uint16_t timer_pwm_ARR;		//pwm定时器溢出值
	bool enable_send;			//使能向上位机同步数据？
	bool enable_OLED;			//使能OLED显示？
	bool enable_key;			//使能按键？
	bool enable_pid;			//使能pid？
}SysPara_noio;

typedef struct
{
	SysPara_noio sys_para_t;//系统参数副本
	PidPara_base pid_para_base_t;//pid基础参数
} DATA_TO_SAVE;

typedef struct
{
	uint32_t magic;//魔术字：用于判断该页是否保存过有效参数（0xFFFFFFFF 表示空页）
	uint32_t version;//版本号：结构体定义变化时修改此值，旧数据会自动失效
	uint32_t data_size;//数据大小
	DATA_TO_SAVE data;//系统参数副本
	uint32_t checksum;//校验和
} SysParaFlashRecord;
/*函数*/
HAL_StatusTypeDef Flash_SaveSysPara(const SysPara *sys_para, const PidPara_base *pid_para_base_t);
bool Flash_LoadSysPara(SysPara *sys_para, PidPara_base *pid_para_base_t);
#endif
