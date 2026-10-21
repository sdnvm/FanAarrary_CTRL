#include "save_flash.h"
#include <string.h>

typedef struct
{
	uint32_t magic;//魔术字：用于判断该页是否保存过有效参数（0xFFFFFFFF 表示空页）
	uint32_t version;//版本号：结构体定义变化时修改此值，旧数据会自动失效
	uint32_t data_size;//数据大小
	SysPara data;//系统参数副本
	uint32_t checksum;//校验和
} SysParaFlashRecord;

/*计算检验和*/
static uint32_t SysPara_Checksum(const uint8_t *data, uint32_t size)
{
	uint32_t checksum = 0;
	while (size-- > 0U)
	{
		checksum = (checksum << 5) - checksum + *data++;
	}
	return checksum;
}

/*保存参数*/
HAL_StatusTypeDef Flash_SaveSysPara(const SysPara *sys_para)
{
	Sys_Para.enable_pid = Ctrl_Para.enable_pid[0];//保存总开关状态
	SysParaFlashRecord record = {0};
	FLASH_EraseInitTypeDef erase_init = {0};
	uint32_t page_error = 0;
	HAL_StatusTypeDef status;

	if (sys_para == NULL)
	{
		return HAL_ERROR;
	}
	//构造数据块
	record.magic = SYS_PARA_FLASH_MAGIC;
	record.version = SYS_PARA_FLASH_VERSION;
	record.data_size = sizeof(SysPara);
	memcpy(&record.data, sys_para, sizeof(SysPara));
	record.checksum = SysPara_Checksum((const uint8_t *)&record.data, sizeof(record.data));
	//解锁flash
	status = HAL_FLASH_Unlock();
	if (status != HAL_OK)
	{
		return status;
	}

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	//擦除目标页
	erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
	erase_init.PageAddress = SYS_PARA_FLASH_ADDRESS;
	erase_init.NbPages = 1;
	status = HAL_FLASHEx_Erase(&erase_init, &page_error);
	if (status != HAL_OK)
	{
		HAL_FLASH_Lock();
		return status;
	}
	//按半字（16 位）写入 —— STM32F1 硬件粒度为半字
	for (uint32_t index = 0; index < sizeof(record); index += sizeof(uint16_t))
	{
		uint16_t half_word = 0xFFFFU;
		uint32_t bytes_to_copy = sizeof(record) - index;

		if (bytes_to_copy > sizeof(half_word))
		{
			bytes_to_copy = sizeof(half_word);
		}
		memcpy(&half_word, (const uint8_t*)&record + index, bytes_to_copy);
		uint32_t address = SYS_PARA_FLASH_ADDRESS + index;
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, half_word);
		if (status != HAL_OK)
		{
			break;
		}
	}
	//锁定flash
	HAL_FLASH_Lock();
	return status;
}

/*读取参数*/
bool Flash_LoadSysPara(SysPara *sys_para)
{
	const SysParaFlashRecord *record = (const SysParaFlashRecord *)SYS_PARA_FLASH_ADDRESS;

	if (sys_para == NULL ||
		record->magic != SYS_PARA_FLASH_MAGIC ||
		record->version != SYS_PARA_FLASH_VERSION ||
		record->data_size != sizeof(SysPara) ||
		record->checksum != SysPara_Checksum((const uint8_t *)&record->data,sizeof(record->data)))
	{
		return false;
	}

	memcpy(sys_para, &record->data, sizeof(SysPara));
	return true;
}
