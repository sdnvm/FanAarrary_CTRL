/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
typedef enum
{
  TX_Heartbeat = 1, //心跳包
  TX_ERR, //发生错误
  TX_Err_CAN, //CAN错误
  TX_Err_RXTrackData, //请求重发跟踪曲线数据
  TX_Err_SavePara, //保存失败
  TX_RPMWant, //目标转速
  TX_RPMReal, //实际转速
  TX_SYSPara, //系统各种参数
} TXType;
/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
void filter_init(void);
void MyCAN_Transmit(uint32_t ID, uint8_t *Data, uint32_t Length);
void MyCAN_Transmit_RPM(float rpm_r, float rpm_c, uint8_t c2);
void MyCAN_Transmit_enum(TXType TX_Tpye);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

