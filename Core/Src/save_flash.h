#ifndef FA_CTRL_SAVE_FLASH_H
#define FA_CTRL_SAVE_FLASH_H

#include "main.h"

#define SYS_PARA_FLASH_ADDRESS 0x0803F800U//参数存储地址（对于256KB的Flash）
#define SYS_PARA_FLASH_MAGIC   0x53595041U//魔术字：用于判断该页是否保存过有效参数（0xFFFFFFFF 表示空页）
#define SYS_PARA_FLASH_VERSION 1U//结构体版本号：结构体定义变化时修改此值，旧数据会自动失效

/*结构体变量*/


typedef struct
{
	SysPara sys_para_t;//系统参数副本
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
HAL_StatusTypeDef Flash_SaveSysPara(const SysPara *sys_para, const PidPara_base *pid_para_base);
bool Flash_LoadSysPara(SysPara *sys_para, PidPara_base *pid_para_base);
#endif
