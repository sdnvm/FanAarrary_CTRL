/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "OLED.h"
extern bool oled_init;
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW2_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW2_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
//按键扫描
uint8_t Key_GetState(void)
{
	uint8_t Keystate = 0;
	uint32_t PressDuration;
	bool is_key_timeout = false;
	if (HAL_GPIO_ReadPin(GPIOA, SW2_Pin) == 1)
	{
		HAL_Delay(15); //消抖
		if (HAL_GPIO_ReadPin(GPIOA,SW2_Pin) == 1) //非扰动
		{
			PressDuration = HAL_GetTick();
			while (HAL_GPIO_ReadPin(GPIOA, SW2_Pin) == 1) //等待松手
			{
				if ((is_key_timeout == false) && ((HAL_GetTick() - PressDuration) >= 2200))
				{
					is_key_timeout = true;
					OLED_ShowString(4, 7, "NEXT");
				}
				if ((HAL_GetTick() - PressDuration) >= 8000)
				{
					break;
				}
			}
			PressDuration = HAL_GetTick() - PressDuration; //记录按下时长
			if (PressDuration < 400)//单击
			{
				Keystate = SW2_SINGLE_CLICK;
			}
			else if (PressDuration < 2000)//长按
			{
				Keystate = SW2_LONG_PRESS;
			}
			else//超级长按,更改按键模式
			{
				RunMode_Switch(S_Wait);
				Sys_State.manu = (enum Manu)(Sys_State.manu + 1) % count;//更改按键菜单
				oled_init = true;//请求重写屏幕背景
				Keystate = SW2_LONGLONG_PRESS;
			}
		}
	}
	else if (HAL_GPIO_ReadPin(GPIOA, SW3_Pin) == 1)
	{
		HAL_Delay(15); //消抖
		if (HAL_GPIO_ReadPin(GPIOA,SW3_Pin) == 1) //非扰动
		{
			PressDuration = HAL_GetTick();
			while (HAL_GPIO_ReadPin(GPIOA, SW3_Pin) == 1) //等待松手
			{
				if ((HAL_GetTick() - PressDuration) >= 6000)
				{
					if ((is_key_timeout == false) && ((HAL_GetTick() - PressDuration) >= 2200))
					{
						is_key_timeout = true;
						OLED_ShowString(4, 7, "LONG");
					}
					if ((HAL_GetTick() - PressDuration) >= 8000)
					{
						break;
					}
				}
			}
			PressDuration = HAL_GetTick() - PressDuration; //记录按下时长
			HAL_Delay(15); //消抖
			if (PressDuration < 500)
			{
				Keystate = SW3_SINGLE_CLICK; //单击
			}
			else if (PressDuration < 2000)
			{
				Keystate = SW3_LONG_PRESS; //长按
			}
			else
			{
				Keystate = SW3_LONGLONG_PRESS;//超级长按
			}
		}
	}
	return Keystate;
}
/* USER CODE END 2 */
