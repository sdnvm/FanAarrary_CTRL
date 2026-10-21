/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"
#include "tim.h"
#include "bsp_pid.h"
#include "OLED.h"
#include "sin_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
__IO uint16_t OverFlow[4] = {0, 0, 0, 0};//当前比较寄存器溢出次数
__IO uint16_t OverFlow_next[4] = {0, 0, 0, 0};//上一次比较寄存器溢出次数
__IO uint16_t IC_now[4] = {0, 0, 0, 0};//当前捕获值
__IO uint16_t IC_last[4] = {0, 0, 0, 0};//上一次捕获值

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/*处理fg信号*/
void fg_process(const uint8_t j, const TIM_HandleTypeDef* htimj)
{
	//溢出中断
	if (__HAL_TIM_GET_FLAG(htimj, TIM_FLAG_UPDATE))
	{
		OverFlow_next[j] += 1;
		__HAL_TIM_CLEAR_FLAG(htimj, TIM_FLAG_UPDATE);
	}
	//输入捕获中断
	if (__HAL_TIM_GET_FLAG(htimj, TIM_FLAG_CC3))
	{
		IC_last[j] = IC_now[j];
		if (j == 3)
		{
			IC_now[j] = htimj->Instance->CCR4;
		}
		else
		{
			IC_now[j] = htimj->Instance->CCR3;
		}
		OverFlow[j] = OverFlow_next[j];
		OverFlow_next[j] = 0;
		__HAL_TIM_CLEAR_FLAG(htimj, TIM_FLAG_UPDATE);
		__HAL_TIM_CLEAR_FLAG(htimj, TIM_FLAG_CC3);
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	fg_process(1, &htim2);
  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	fg_process(2, &htim3);
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	fg_process(0, &htim4);
  /* USER CODE END TIM4_IRQn 0 */
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
	//这个用的是CCR4
	fg_process(3, &htim5);
  /* USER CODE END TIM5_IRQn 0 */
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */
	if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE))
	{
		static uint8_t chanel = 0;
		if (chanel >= 4)chanel = 0;
		__disable_irq();
		uint32_t CCR_NUM = (uint32_t)OverFlow[chanel]<<16 | (IC_now[chanel] & 0xFFFFu);
		CCR_NUM = CCR_NUM - IC_last[chanel];
		__enable_irq();
		if (Ctrl_Para.enable_pid[0] == 0 && Sys_State.manu == fan_test)
		{
			OLED_ShowNum(1,1 + chanel * 4, OverFlow[chanel],3);
			//OLED_ShowNum(1,5, IC_now[chanel],5);
		}
		
		/*闭环控制*/
		pid_control(chanel, CCR_NUM);

		/*跟踪曲线
		 * sin判断的代码在-can里??
		 */
		static uint16_t ctrl_tim = 0;//决定采样时间
		ctrl_tim++;
		if (Sys_State.RunMode == S_TrackingCurve && ctrl_tim >= Sin_Para1.ctrl_num)
		{
			if (sin_point_index == Sin_Para1.sampl_num)
			{
				sin_point_index = 0; // 重置索引，为下一个正弦波做准备
			}
			/*if (c == 1) {
			pid_para[0].target_rpm = sin_control_values[sin_point_index];
			} else if (c == 2) {*/
			if (Ctrl_Para.enable_pid[0])
			{
				set_rpm(0, sin_control_values[sin_point_index]);
				pid_para[0].target_rpm = sin_control_values[sin_point_index];
			}
			else
			{
				set_rpm(0, sin_control_values[sin_point_index]);
			}
			// }
			sin_point_index++; // 移动到下一个点
			ctrl_tim = 0; // 重置溢出计数器
		}
		chanel += 1;
		//清中断标志位
		__HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
		/*TIM4->SR &= ~(TIM_SR_UIF | TIM_SR_CC3IF);
		TIM2->SR &= ~(TIM_SR_UIF | TIM_SR_CC3IF);
		TIM3->SR &= ~(TIM_SR_UIF | TIM_SR_CC3IF);
		TIM5->SR &= ~(TIM_SR_UIF | TIM_SR_CC4IF);*/
	}
  /* USER CODE END TIM6_IRQn 0 */
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
