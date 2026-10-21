#ifndef FA_CTRL_SAVE_FLASH_H
#define FA_CTRL_SAVE_FLASH_H

#include "main.h"

#define SYS_PARA_FLASH_ADDRESS 0x0803F800U//参数存储地址（对于256KB的Flash）
#define SYS_PARA_FLASH_MAGIC   0x53595041U//魔术字：用于判断该页是否保存过有效参数（0xFFFFFFFF 表示空页）
#define SYS_PARA_FLASH_VERSION 1U//结构体版本号：结构体定义变化时修改此值，旧数据会自动失效

HAL_StatusTypeDef Flash_SaveSysPara(const SysPara *sys_para);
bool Flash_LoadSysPara(SysPara *sys_para);

#endif
