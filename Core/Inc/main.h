/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct 
{
	uint16_t Speed_MX_Fan;		//风机最大转速
	uint8_t ctrl_mode;			//跟踪曲线、开闭环、准备接收长数据、快速单通道采样、
	bool enable_send;			//向上位机同步数据？
	uint16_t timer_ctrl_T;		//控制定时器溢出时间/ms
	uint16_t timer_fg_f;		//fg捕获定时器时钟/mhz
	uint16_t timer_pwm_ARR;		//pwm定时器溢出值
}SysPara;
typedef struct 
{
	uint16_t rpm_want[4];		//目标转速
	bool enable_pid[5];			//pid开关状态，[0]为开关
	bool enable_fan[5];			//风机开关状态，[0]为开关
}CtrlPara;
typedef struct 
{
	uint16_t ID_ThisBoard;		//本机ID
	uint8_t PendingMsg;			//待解析数据量
	bool is_Can_TXSec;			//
}CANPara;
typedef struct 
{
	uint32_t TxID;				//发信ID
	uint8_t TxData[8];			//发信data
	uint8_t RxData[8];			//接收到的data
	CAN_RxHeaderTypeDef can_RxHeader;//接收到的报文头
}CANData;
typedef struct
{
	bool enable_TrackCurve;		//失能跟踪
	uint16_t sample_time;		//采样时间 ms
	uint16_t phase[4];			//曲线相位 ms
	uint16_t ctrl_num;			//乘ctrl_time=sin_ctrl_time
	uint16_t num_data;			//数据点个数
	uint32_t num_data_received	//当前收到的有效数据包数量
	uint16_t buffer_byte;		//缓冲区字节数
	bool is_assembling;			//正在接收曲线数据？
	bool start_cacul;			//开始计算

	//bool is_received[CURVE_SAMPLE_MX];//成功收到数据？
}TrackCurvePara;
#define CURVE_SAMPLE_MX 256			//拟合曲线缓冲区大小
typedef struct
{
	bool enable_TrackCurve;		//失能跟踪
	float curve_data[CURVE_SAMPLE_MX]; //跟踪曲线数据缓冲区
	uint16_t sample_time;		//采样时间 ms
	uint16_t phase[4];			//曲线相位 ms
	uint16_t length;			//帧个数
	uint16_t ctrl_num;			//乘ctrl_time=sin_ctrl_time
}TrackCurveData;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOA
#define FG4_Pin GPIO_PIN_3
#define FG4_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_4
#define SW2_GPIO_Port GPIOA
#define SW3_Pin GPIO_PIN_5
#define SW3_GPIO_Port GPIOA
#define FG3_Pin GPIO_PIN_0
#define FG3_GPIO_Port GPIOB
#define FG2_Pin GPIO_PIN_10
#define FG2_GPIO_Port GPIOB
#define PWM4_Pin GPIO_PIN_6
#define PWM4_GPIO_Port GPIOC
#define PWM3_Pin GPIO_PIN_8
#define PWM3_GPIO_Port GPIOC
#define PWM2_Pin GPIO_PIN_8
#define PWM2_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOA
#define CAN_RX_Pin GPIO_PIN_11
#define CAN_RX_GPIO_Port GPIOA
#define CAN_TX_Pin GPIO_PIN_12
#define CAN_TX_GPIO_Port GPIOA
#define UART3_TX_Pin GPIO_PIN_10
#define UART3_TX_GPIO_Port GPIOC
#define UART3_RX_Pin GPIO_PIN_11
#define UART3_RX_GPIO_Port GPIOC
#define FG1_Pin GPIO_PIN_8
#define FG1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SW2_SINGLE_CLICK 20
#define SW2_DOUBLE_CLICK 21
#define SW2_LONG_PRESS 22
#define SW2_LONGLONG_PRESS 23
#define SW3_SINGLE_CLICK 30
#define SW3_DOUBLE_CLICK 31
#define SW3_LONG_PRESS 32
#define SW3_LONGLONG_PRESS 33

#define START_RPM 0					//开机风速
#define DC_POWER 12					//电机供电电压
#define TS 500						//采样周期
//#define KTS (60.0f*DC_POWER/12)
//#define KCCR (30*72000000)/2/8/10
#define ID_ThisBoard 0x001			//本驱动板ID

extern bool pid_enable[5];
extern bool fan_enable[5];
extern SysPara Sys_Para;
extern CtrlPara Ctrl_Para;
extern CANPara CAN_Para;
extern CANData CAN_Data;
extern TrackCurvePara TrackCurve_Para;
//extern struct SYS_PARA Sys_Para;
//extern struct CTRL_PARA Ctrl_Para;
//extern struct CAN_PARA Can_Para;
//extern struct CAN_DATA CAN_Data;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
