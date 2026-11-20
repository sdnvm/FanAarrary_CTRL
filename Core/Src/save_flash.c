#include "../Inc/save_flash.h"
#include <string.h>
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
HAL_StatusTypeDef Flash_SaveSysPara(const SysPara *sys_para, const PidPara_base *pid_para_base_t)
{
	SysPara_noio sys_para_noio;//系统参数影子结构体
	Sys_Para.enable_pid = Ctrl_Para.enable_pid[0];//保存总开关状态
	SysParaFlashRecord record = {0};
	FLASH_EraseInitTypeDef erase_init = {0};
	uint32_t page_error = 0;
	HAL_StatusTypeDef status;

	if ((sys_para == NULL ) || (pid_para_base_t == NULL))
	{
		return HAL_ERROR;
	}
	//构造数据块
	record.magic = SYS_PARA_FLASH_MAGIC;
	record.version = SYS_PARA_FLASH_VERSION;
	record.data_size = sizeof(SysPara);
	//memcpy(&record.data.sys_para_t, sys_para, sizeof(SysPara));
	sys_para_noio.FanSpeed_max = sys_para -> FanSpeed_max;
	sys_para_noio.rpm_to_arr = sys_para -> rpm_to_arr;
	sys_para_noio.timer_ctrl_T = sys_para -> timer_ctrl_T;
	sys_para_noio.timer_fg_f = sys_para -> timer_fg_f;
	sys_para_noio.timer_pwm_ARR = sys_para -> timer_pwm_ARR;
	sys_para_noio.enable_OLED = sys_para -> enable_OLED;
	sys_para_noio.enable_key = sys_para -> enable_key;
	sys_para_noio.enable_pid = sys_para -> enable_pid;
	sys_para_noio.enable_send = sys_para -> enable_send;
	sys_para_noio.ID_Thisboard = sys_para -> ID_Thisboard;
	memcpy(&record.data.sys_para_t, &sys_para_noio, sizeof(SysPara_noio));
	memcpy(&record.data.pid_para_base_t, pid_para_base_t, sizeof(PidPara_base));
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
bool Flash_LoadSysPara(SysPara *sys_para, PidPara_base *pid_para_base_t)
{
	SysPara_noio sys_para_noio;//系统参数影子结构体
	const SysParaFlashRecord *record = (const SysParaFlashRecord *)SYS_PARA_FLASH_ADDRESS;

	if (sys_para == NULL ||
		record->magic != SYS_PARA_FLASH_MAGIC ||
		record->version != SYS_PARA_FLASH_VERSION ||
		record->data_size != sizeof(SysPara) ||
		record->checksum != SysPara_Checksum((const uint8_t *)&record->data,sizeof(record->data)))
	{
		return false;
	}

	//memcpy(sys_para, &record->data.sys_para_t, sizeof(SysPara));
	memcpy(&sys_para_noio, &record->data.sys_para_t, sizeof(SysPara_noio));
	memcpy(pid_para_base_t, &record->data.pid_para_base_t, sizeof(PidPara_base));
	sys_para->FanSpeed_max = sys_para_noio.FanSpeed_max;
	sys_para->rpm_to_arr = sys_para_noio.rpm_to_arr;
	sys_para->timer_ctrl_T = sys_para_noio.timer_ctrl_T;
	sys_para->timer_fg_f = sys_para_noio.timer_fg_f;
	sys_para->timer_pwm_ARR = sys_para_noio.timer_pwm_ARR;
	sys_para->enable_OLED = sys_para_noio.enable_OLED;
	sys_para->enable_key = sys_para_noio.enable_key;
	sys_para->enable_pid = sys_para_noio.enable_pid;
	sys_para->enable_send = sys_para_noio.enable_send;
	sys_para->ID_Thisboard = sys_para_noio.ID_Thisboard;
	return true;
}
