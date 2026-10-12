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
__IO uint16_t overflow_num1=0;//当前比较寄存器溢出次数
__IO uint16_t overflow_num2=0;
__IO uint16_t overflow_num3=0;
__IO uint16_t overflow_num4=0;
__IO uint16_t OVERFLOW_LAST1=0;//上一次比较寄存器溢出此时
__IO uint16_t OVERFLOW_LAST2=0;
__IO uint16_t OVERFLOW_LAST3=0;
__IO uint16_t OVERFLOW_LAST4=0;

__IO uint16_t IC_1 = 0;//捕获值
__IO uint16_t IC_2 = 0;
__IO uint16_t IC_3 = 0;
__IO uint16_t IC_4 = 0;
__IO uint16_t IC_last1 = 0;//上一次的捕获值
__IO uint16_t IC_last2 = 0;
__IO uint16_t IC_last3 = 0;
__IO uint16_t IC_last4 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

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
extern UART_HandleTypeDef huart3;
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
	//溢出中断
	if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE))
	{
		overflow_num2 += 1;
		__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
	}
	//输入捕获中断
	if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC3))
	{
		//cc_flag[2] = 1;
		//static uint8_t n=1;
		IC_last2 = IC_2;
		IC_2 = (&htim2)->Instance->CCR3;
		OVERFLOW_LAST2 = overflow_num2;
		overflow_num2 = 0;
		/*if(n>2)n=1;
		switch(n){
			case 1:
			{
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
			}break;
			case 2:
			{
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
			}break;
		}
		n+=1;*/
		__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
	//	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
	}

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
	if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE))
	{
		overflow_num3 += 1;
		__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
	}
	if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC3))
	{
		IC_last3 = IC_3;
		IC_3 = (&htim3)->Instance->CCR3;
		OVERFLOW_LAST3 = overflow_num3;
		overflow_num3 = 0;
		//__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
	}
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
	if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_CC3))
	{
		IC_last1 = IC_1;
		IC_1 = (&htim4)->Instance->CCR3;
		OVERFLOW_LAST1 = overflow_num1;
		overflow_num1=0;
		__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
	}
	//溢出中断
	else if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE))
	{
		overflow_num1+=1;
		__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
	}
  /* USER CODE END TIM4_IRQn 0 */
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	uint8_t dr = __HAL_UART_FLUSH_DRREGISTER(&huart3);
	//protocol_data_recv(&dr, 1);
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
	if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_CC4))
	{
		IC_last4 = IC_4;
		IC_4 = (&htim5)->Instance->CCR4;
		OVERFLOW_LAST4 = overflow_num4;
		overflow_num4 = 0;
		__HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
	}
		 //溢出中断
	if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE))
	{
		overflow_num4 += 1;
		__HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
	}
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
		uint32_t CCR_NUM = 0;
		static uint8_t chanel = 0;
		if (chanel >= 2)chanel = 0;
		switch (chanel)
		{
		case 0:
			{
				//CCR_NUM = overflow_num1*65535+__HAL_TIM_GET_COMPARE(&htim1,TIM_CHANNEL_1);
				//HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
				//HAL_TIM_Base_Start_IT(&htim2);
				//HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_1);
				CCR_NUM = (OVERFLOW_LAST1 * 65535 + IC_1) - IC_last1;
				/*if(CCR_NUM > 2000000){//超时处理
					fan_enable[c] = 0;
					pid_reset(c,0);
					pid_enable[0] = 0;
					set_rpm(c,0);					
					c+=1;	
					return;
				}*/
			}
			break;
		case 1:
			{
				//	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
				//	HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_1);
				//HAL_TIM_Base_Stop_IT(&htim2);				
				CCR_NUM = (OVERFLOW_LAST2 * 65535 + IC_2) - IC_last2;
			}
			break;
		case 2:
			{
				CCR_NUM = (OVERFLOW_LAST3 * 65535 + IC_3) - IC_last3;
			}
			break;
		case 3:
			{
				CCR_NUM = (OVERFLOW_LAST4 * 65535 + IC_4) - IC_last4;
			}
			break;
		default:
			break;
		} //sin判断的代码在-can里
		//		OLED_ShowNum(4,6,IC_2,5);
		//		OLED_ShowNum(4,12,(uint16_t)OVERFLOW_LAST2,5);
		pid_control(chanel, CCR_NUM);

		static uint16_t ctrl_tim = 0;//决定采样时间
		ctrl_tim++;
		if (Sys_Para.RunMode == S_TrackingCurve && ctrl_tim >= Sin_Para1.ctrl_num)
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
		TIM4->SR &= ~(TIM_SR_UIF | TIM_SR_CC3IF);
		TIM2->SR &= ~(TIM_SR_UIF | TIM_SR_CC3IF);
		TIM3->SR &= ~(TIM_SR_UIF | TIM_SR_CC3IF);
		TIM5->SR &= ~(TIM_SR_UIF | TIM_SR_CC4IF);
	}
  /* USER CODE END TIM6_IRQn 0 */
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
