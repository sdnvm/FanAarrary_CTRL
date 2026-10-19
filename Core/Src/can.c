/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef can_TxHeader;	//发信数据头
uint32_t TxMailbox;					//发信邮箱
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 48;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  //过滤器配置
  /*16位列表-FIFO0===大多数指令*/
  CAN_FilterTypeDef filter1;
  filter1.FilterScale = CAN_FILTERSCALE_16BIT;//配置为16位
  filter1.FilterActivation = CAN_FILTER_ENABLE;
  filter1.FilterFIFOAssignment = CAN_FilterFIFO0;//选择fifo0
  filter1.FilterMode = CAN_FILTERMODE_IDLIST;//配置为列表模式
  filter1.FilterBank = 0;
  filter1.FilterIdHigh = (0x001 << 5); //广播的复位、启停
  filter1.FilterIdLow = (0x002 << 5); //广播的使能/失能、保存参数
  filter1.FilterMaskIdHigh = (ID_ThisBoard << 9 | 0x001 << 5); //复位、启停
  filter1.FilterMaskIdLow = (ID_ThisBoard << 9 | 0x002 << 5); //使能/失能、保存参数、上传状态/转速
  HAL_CAN_ConfigFilter(&hcan, &filter1);
  filter1.FilterBank = 1;
  filter1.FilterIdHigh = (0x003 << 5); //广播的目标值首帧
  filter1.FilterIdLow = (0x004 << 5); //广播的目标值尾帧
  filter1.FilterMaskIdHigh = (ID_ThisBoard << 9 | 0x003 << 5); //目标值首帧
  filter1.FilterMaskIdLow = (ID_ThisBoard << 9 | 0x004 << 5); //目标值尾帧
  HAL_CAN_ConfigFilter(&hcan, &filter1);
  filter1.FilterBank = 2;
  filter1.FilterIdHigh = (0x07F << 5); //相位首帧
  filter1.FilterIdLow = (0x07F << 5); //相位尾帧
  filter1.FilterMaskIdHigh = (ID_ThisBoard << 9 | 0x00A << 5); //相位首帧
  filter1.FilterMaskIdLow = (ID_ThisBoard << 9 | 0x00B << 5); //相位尾帧
  HAL_CAN_ConfigFilter(&hcan, &filter1);
  /*16位列表-FIFO1===跟踪曲线采样点、pid参数*/
  filter1.FilterBank = 3;
  filter1.FilterFIFOAssignment = CAN_FilterFIFO1;//改为fifo1
  filter1.FilterIdHigh = 0x006 << 5; //首帧
  filter1.FilterIdLow = 0x007 << 5; //续帧
  filter1.FilterMaskIdHigh = 0x008 << 5; //尾帧
  filter1.FilterMaskIdLow = (0x009 << 5); //set pid参数
  HAL_CAN_ConfigFilter(&hcan, &filter1);
  /*/*16位屏蔽-FIFO0===指定风机帧#1#
  filter1.FilterBank = 4;
  filter1.FilterFIFOAssignment = CAN_FilterFIFO0;//改为fifo0
  filter1.FilterMode = CAN_FILTERMODE_IDMASK;//改为屏蔽模式
  filter1.FilterIdHigh = ID_ThisBoard << (5 + 4); //本机的数据帧
  filter1.FilterIdLow = ID_ThisBoard << 9 | 0x10; //本机的遥控帧
  filter1.FilterMaskIdHigh = (0x7F0 << 5) | 0x10 | 0x8;
  filter1.FilterMaskIdLow = (0x7F0 << 5) | 0x10 | 0x8;
  HAL_CAN_ConfigFilter(&hcan, &filter1);*/
  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = CAN_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CAN_RX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CAN_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CAN_TX_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, CAN_RX_Pin|CAN_TX_Pin);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*发送一帧数据*/
void MyCAN_Transmit(uint32_t ID, uint8_t* Data, uint32_t Length)
{
	if (Length == 0)
	{
		can_TxHeader.RTR = CAN_RTR_REMOTE;
	}
	else
	{
		can_TxHeader.RTR = CAN_RTR_DATA;
	}
	can_TxHeader.DLC = Length;
	can_TxHeader.ExtId = ID;
	can_TxHeader.IDE = CAN_ID_STD;
	can_TxHeader.StdId = ID;
	can_TxHeader.TransmitGlobalTime = DISABLE;

	uint32_t TimeOut = 0;
	while (HAL_CAN_AddTxMessage(&hcan, &can_TxHeader, Data, &TxMailbox) != HAL_OK) //等待发送完毕
	{
		TimeOut++;
		if (TimeOut > 10)
		{
			break;
		}
	}
}
/*上传转速数据
 * c2 == 0|2时id为0E，1|3时，id为0F
 */
void MyCAN_Transmit_RPM(float* rpm_r, float* rpm_c, const uint8_t c2)
{
	if (Sys_Para.enable_send == 1)
	{
		uint32_t txid = 0x00000000; //id
		uint8_t txdata[8]; //data(一帧)
		txid |= ((ID_ThisBoard & 0x7F) << 4);
		if (c2 == 0 || c2 == 2)
		{
			txid |= 0x0E & 0x0F; //上传A转速数据
		}
		else
		{
			txid |= 0x0F; //上传B转速数据
			memcpy(&txdata[0], rpm_r, 4);//实际值
			memcpy(&txdata[4], rpm_c, 4);//控制值
			MyCAN_Transmit(txid, txdata, 8);
		}
	}
}
/*按类型直接上传数据*/
void MyCAN_Transmit_enum(TXType TX_Tpye)
{
	switch (TX_Tpye)
	{
	case TX_Heartbeat://上传心跳包
	 	break;
	case TX_SYSPara://上传系统参数
		//todo：上传本机当前状态
		break;
	case TX_Resend_Track://请求重新下方跟踪曲线数据
		uint8_t request_type = 1;
		MyCAN_Transmit(0x07FD, &request_type, 1);
		break;
	default:
		break;
	}
}
/*fifo0收信中断回调*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);

	if (rxHeader.StdId == 0x001 && rxHeader.RTR == CAN_RTR_REMOTE)
	{
		// TODO: 遥控帧处理
	}
}

/*fifo1收信中断回调*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData);

	switch (rxHeader.StdId)
	{
	case 0x200: // 曲线首帧
	case 0x201: // 曲线中帧
	case 0x202: // 曲线尾帧
		// TODO: 曲线数据处理
		break;

	case 0x210: // 定长数据首帧
	case 0x211: // 定长数据尾帧
		//  TODO: 定长数据处理
		break;
	default:
		break;
	}
}
/* USER CODE END 1 */
