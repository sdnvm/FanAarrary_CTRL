/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_pid.h"
#include "control.h"
#include "sin_control.h"
#include <string.h>
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
SysPara Sys_Para;
CtrlPara Ctrl_Para;
CANPara CAN_Para;
CANData CAN_Data;
TrackCurvePara TrackCurve_Para;
TrackCurveData TrackCurve_Data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Para_init(void); //初始化结构体参数
void key_process_motortest(void); //按键扫描与处理函数
void key_process_cantest(void); //按键扫描与处理函数
void State_Switch(SM_SysState state_next);//系统运行状态切换
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  //MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	PID_param_init();
	OLED_Init();
	OLED_Clear();
	Para_init();
	Start_Timer();
	HAL_CAN_Start(&hcan);
	//__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
	State_Switch(S_Wait);//初始状态设为“wait”

	uint16_t curve_sum_receveied;
	extern int capture_end_flag;

	//OLED_ShowString(1, 1, "RxID:");
	//OLED_ShowString(2, 1, "Data:");
	//OLED_ShowString(4, 1, "TxID:");
	OLED_ShowString(3, 5, "|");
	OLED_ShowString(3, 10, "|");
	OLED_ShowString(4, 6, "|");
	OLED_ShowString(4, 11, "|");
	OLED_ShowNum(4, 12, (uint16_t)250, 3);
	OLED_ShowString(4, 15, "ms");
	OLED_ShowNum(4, 7, (Sys_Para.Speed_MX_Fan / 10),4);
	OLED_ShowNum(3, 1, (uint32_t)(pid_para[3].Kp*1000),4);
	OLED_ShowNum(3, 6, (uint32_t)(pid_para[3].Ki*1000),4);
	OLED_ShowNum(3, 11, (uint32_t)(pid_para[3].Kd*1000),4);
	//can调试用的
	OLED_ShowString(1, 4, "|");
	OLED_ShowString(1, 8, "|");
	OLED_ShowString(1, 10, "|");
	OLED_ShowString(1, 1,"rid");
	OLED_ShowString(1, 5, "tid");
	OLED_ShowString(1, 9, "c");
	OLED_ShowString(1, 11, "d");
	OLED_ShowString(2, 1,"data(hex)");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(TrackCurve_Para.start_cacul)
		{
			cacul_sin_params();
		}
		key_process_cantest();
		//key_process_motortest();
		/* FIFO0扫描接收 */
		CAN_Para.PendingMsg = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
		while (CAN_Para.PendingMsg > 0)
		{
			if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &CAN_Data.can_RxHeader, CAN_Data.RxData) == HAL_OK) //接收成功
			{
				uint16_t cmd_can; //指令类型
				OLED_ShowHexNum(1, 5, CAN_Data.can_RxHeader.StdId, 3);//打印id
				OLED_ShowNum(1, 9, CAN_Data.can_RxHeader.DLC , 1);//打印DLC
				uint32_t RawData;//原始数据
				memcpy(&RawData, &CAN_Data.RxData[0], 4);
				OLED_ShowHexNum(2, 1, RawData, 8);//打印原始data1
				memcpy(&RawData, &CAN_Data.RxData[4], 4);
				OLED_ShowHexNum(2, 9, RawData, 8);//打印原始data2
				//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				switch (CAN_Data.can_RxHeader.StdId)
				{
				case 0x001: //广播的复位、启停
				case (ID_ThisBoard << 4) | 0x001: //单播CMD
					memcpy(&cmd_can, &CAN_Data.RxData[0], 2);//cmd是数据段2字节
					switch (cmd_can)
					{
					case 0x000: //复位
						HAL_NVIC_SystemReset();
						break;
					case 0x001: //风机停止
						State_Switch(S_Wait);
						break;
					case 0x002: //风机启动
						State_Switch(S_NormRuning);
						break;
					default:
						break;
					}
					break;
				case 0x002: //广播的使能/失能、保存参数
				case (ID_ThisBoard << 4) | 0x002: //使能、失能、保存参数、上传状态、转速
					memcpy(&cmd_can, &CAN_Data.RxData[0], 2);//cmd是数据段2字节
					switch (cmd_can)
					{
					case 0x003: //pid关闭
					case 0x004: //pid启动
						State_Switch(S_Wait);//停机
						Ctrl_Para.enable_pid[0] = cmd_can - 3;//pid总使能位赋值
						break;
					case 0x005: //失能跟踪曲线
					case 0x006: //使能跟踪曲线
						TrackCurve_Para.enable_TrackCurve = cmd_can - 5;
						if (TrackCurve_Para.enable_TrackCurve)
						{
							State_Switch(S_TrackingCurve);
						}
						else
						{
							State_Switch(S_Wait);
						}
						break;
					case 0x007: //保存参数
						//todo:数据保存置flash
						break;
					case 0x008: //上传一次本机状态
					//todo：上传本机当前状态
					case 0x009: //停止上传
					case 0x00A://开始上传转速
						Sys_Para.enable_send =  cmd_can - 9;
						break;
					default:
						break;
					}
					break;
				case 0x03://广播目标转速
				case (ID_ThisBoard << 4) | 0x03: //本机目标转速
					static uint8_t rxcount_rpm = 0;//接收到的帧个数
					if (rxcount_rpm == 0) //第一帧
					{
						for (int i = 0; i < 2; i++)
						{
							memcpy(&Ctrl_Para.rpm_want[i], &CAN_Data.RxData[i * 4], 4);
							OLED_ShowNum(3, 1 + i * 4, Ctrl_Para.rpm_want[i], 4);
						}
					}
					else //第二帧
					{
						for (int i = 2; i < 4; i++)
						{
							memcpy(&Ctrl_Para.rpm_want[i], &CAN_Data.RxData[i * 4], 4);
							OLED_ShowNum(3, 1 + i * 4, Ctrl_Para.rpm_want[i], 4);
						}
					}
					if (rxcount_rpm >= 1)//接收到两帧
					{
						rxcount_rpm = 0;
						OLED_ShowNum(3, 1, (uint16_t)(pid_para[0].Kp * 1000.0f), 4);
						OLED_ShowNum(3, 6, (uint16_t)(pid_para[0].Ki * 1000.0f), 4);
						OLED_ShowNum(3, 11, (uint16_t)(pid_para[0].Kd * 1000.0f), 4);
						OLED_ShowString(3, 5, "|");
						OLED_ShowString(3, 10, "|");
					}
					rxcount_rpm += 1;
					break;
				case (ID_ThisBoard << 4) | 0x0A: //跟踪曲线相位
					for (int i = 0; i < 4; i++)
					{
						memcpy(&TrackCurve_Para.phase[i], &CAN_Data.RxData[i * 2], 2);
						OLED_ShowNum(3, 1 + i * 4, TrackCurve_Para.phase[i], 4);
					}
					break;
				default:
					break;
				}
			}
			else //接收错误
			{
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				HAL_Delay(500);
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				HAL_Delay(500);
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			}
			CAN_Para.PendingMsg--;
		}

		/* FIFO1扫描接收 */
		CAN_Para.PendingMsg = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO1);
		if (CAN_Para.PendingMsg > 0)
		{
			if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO1, &CAN_Data.can_RxHeader, CAN_Data.RxData) == HAL_OK) //接收成功
			{
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				switch (CAN_Data.can_RxHeader.StdId)
				{
				case 0x006: //curve首帧
					{
						memcpy(&TrackCurve_Para.num_data, CAN_Data.RxData, 2); //帧个数
						memcpy(&TrackCurve_Para.sample_time, &CAN_Data.RxData[2], 2); //采样时间ms
						memset(&TrackCurve_Data.curve_data, 0xF, sizeof(TrackCurve_Data.curve_data)); //初始化接收缓冲区
						TrackCurve_Para.is_assembling = true; //开始接收
						TrackCurve_Para.num_data_received = 0; //初始化已收到数据量
						//todo:关闭风机、关闭高消耗中断，进入接收状态
						break;
					}
				case 0x007: //curve中间帧
					{
						if (!TrackCurve_Para.is_assembling)
						{
							//todo:申请重新发数据
							break;
						}
						memcpy(&TrackCurve_Data.curve_data[TrackCurve_Para.num_data_received], CAN_Data.RxData, 4); //
						memcpy(&TrackCurve_Data.curve_data[TrackCurve_Para.num_data_received + 1], &CAN_Data.RxData[4],
							   4); //
						TrackCurve_Para.num_data_received += 2;
						break;
					}
				case 0x008: //curve尾帧
					{
						//数据完整性判断
						if (TrackCurve_Para.num_data_received - TrackCurve_Para.num_data == 1 | TrackCurve_Para.
							num_data_received == TrackCurve_Para.num_data)
						{
							TrackCurve_Para.is_assembling = false;
						}
						else
						{
							MyCAN_Transmit(0x7FA, NULL, 0); //数据缺失，请求重收
						}
						break;
					}
				case 0x009: //pid参数首帧
					{
						//memcpy(&, &can_RxData[2], 2);//T采样
						static uint8_t rxcount_pid = 0;//接收到的帧个数
						if (rxcount_pid == 0) //第一帧
						{
							for (int i = 0; i < 4; i++)
							{
								//暂时不分通道
								memcpy(&pid_para[i].Kp, &CAN_Data.RxData[4], 4); //p
							}
							State_Switch(S_Wait);
						}
						else //第二帧
						{
							for (int i = 0; i < 4; i++)
							{
								memcpy(&pid_para[i].Ki, CAN_Data.RxData, 4); //i
								memcpy(&pid_para[i].Kd, &CAN_Data.RxData[4], 4); //d
								HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
							}
						}
						if (rxcount_pid >= 1)//接收到两帧
						{
							rxcount_pid = 0;
							OLED_ShowNum(3, 1, (uint16_t)(pid_para[0].Kp * 1000.0f), 4);
							OLED_ShowNum(3, 6, (uint16_t)(pid_para[0].Ki * 1000.0f), 4);
							OLED_ShowNum(3, 11, (uint16_t)(pid_para[0].Kd * 1000.0f), 4);
							OLED_ShowString(3, 5, "|");
							OLED_ShowString(3, 10, "|");
						}
						rxcount_pid += 1;
						break;
					}
				default:
					break;
				}
			}
			else //接收错误
			{
				HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
				HAL_Delay(500);
				HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			}
			CAN_Para.PendingMsg--;
		}
		/* Fifo1扫描接收 */
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//系统参数配置
void Para_init(void)
{
	Sys_Para.Speed_MX_Fan = 16500;
	Sys_Para.enable_send = false;
	Sys_Para.RunMode = S_Wait;
	Sys_Para.timer_ctrl_T = 250;
	Sys_Para.timer_fg_f = 12;//mhz
	Sys_Para.timer_pwm_ARR = 4714;

	/*控速相关变量*/
	memset(&Ctrl_Para.enable_pid, 0, sizeof(Ctrl_Para.enable_pid));
	memset(&Ctrl_Para.rpm_want, 0, sizeof(Ctrl_Para.rpm_want));

	/*CAN相关变量*/
	//ID_ThisBoard去main.h里改
	CAN_Para.ID_Thisboard = 0x001;
	CAN_Para.PendingMsg = 0;
	/*CAN_Data.TxID = ID_ThisBoard << 4;
	uint8_t temp_txid[8] = {0x11, 0x22, 0x33, 0x44};
	memcpy(CAN_Data.TxData, temp_txid, sizeof(CAN_Data.TxData));*/
	/*跟踪曲线相关变量*//*系统相关变量*/
}
//状态切换
void State_Switch(SM_SysState state_next)
{
	switch (state_next)
	{
	case S_Wait://停机等待，进行数据接收等
		OLED_ShowString(4, 1, "wait ");
		OLED_ShowNum(4, 5, Ctrl_Para.enable_pid[0], 1);//0==开环，1==闭环
		memset(Ctrl_Para.enable_fan, false, sizeof(Ctrl_Para.enable_fan));
		for (int i = 0; i < 4; i++)
		{
			set_rpm_incloseloopmode(i, 0);//停机
		}
		break;
	case S_NormRuning://风机正常运行-开闭环。要提前配置rpm_want&pid
		OLED_ShowString(4, 1, "norm ");
		OLED_ShowNum(4, 5, Ctrl_Para.enable_pid[0], 1);//0==开环，1==闭环
		for (int i = 0; i < 4; i++)
		{
			set_rpm_incloseloopmode(i, (float)Ctrl_Para.rpm_want[i]);
		}
		memset(Ctrl_Para.enable_fan, true, sizeof(Ctrl_Para.enable_fan));
		break;
	case S_TrackingCurve://跟踪曲线模式
		OLED_ShowString(4, 1, "track");
		break;
	default:
		break;
	}
	Sys_Para.RunMode = state_next;
}

/*按键处理 —— 电机测试*/
void key_process_motortest(void)
{
	static uint8_t CH = 0;
	uint8_t KeyState = 0;
	KeyState = Key_GetState();
	switch (KeyState)
	{
	case SW2_SINGLE_CLICK:
		memset(Ctrl_Para.enable_fan, 0, sizeof(Ctrl_Para.enable_fan));
		for (int i = 0; i < 4; i++)
		{
			CH = i;
			set_rpm(i, (float)Ctrl_Para.rpm_want[i]);
			pid_reset(i, Ctrl_Para.rpm_want[i]);
		}
		CH = 1;
		OLED_ShowNum(CH, 1, Ctrl_Para.rpm_want[CH - 1], 5);
		OLED_ShowString(4, 1, "Open");
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
		break;
	case SW2_LONG_PRESS:
		CH = 0;
		Ctrl_Para.rpm_want[CH] = Ctrl_Para.rpm_want[CH] + 1000;
		Ctrl_Para.enable_fan[CH + 1] = 1;
		Ctrl_Para.enable_fan[0] = 1;
		set_rpm(CH, (float)Ctrl_Para.rpm_want[CH]);
		pid_reset(CH, Ctrl_Para.rpm_want[CH]);
		OLED_ShowNum(1, 1, Ctrl_Para.rpm_want[CH], 5);
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
		break;
	case SW3_SINGLE_CLICK: //切换开闭环
		{
			static uint8_t mode_control = 0;
			if (mode_control == 0)
			{
				mode_control = 1;
				CH = 1;
				pid_reset(CH, Ctrl_Para.rpm_want[CH - 1]);
				CH = 2;
				pid_reset(CH, Ctrl_Para.rpm_want[CH - 1]);
				CH = 3;
				pid_reset(CH, Ctrl_Para.rpm_want[CH - 1]);
				CH = 4;
				pid_reset(CH, Ctrl_Para.rpm_want[CH - 1]);
				Ctrl_Para.enable_pid[0] = 1;
				OLED_ShowString(4, 1, "Close");
			}
			else
			{
				mode_control = 0;
				CH = 1;
				pid_reset(CH, Ctrl_Para.rpm_want[CH]);
				CH = 2;
				pid_reset(CH, Ctrl_Para.rpm_want[CH]);
				CH = 3;
				pid_reset(CH, Ctrl_Para.rpm_want[CH - 1]);
				CH = 4;
				pid_reset(CH, Ctrl_Para.rpm_want[CH - 1]);
				Ctrl_Para.enable_pid[0] = 0;
				OLED_ShowString(4, 1, "Open");
			}
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
		}
		break;
	case SW3_LONG_PRESS:
		{
			static uint8_t mode_fan = 0;
			if (mode_fan == 0)
			{
				mode_fan = 1;
				Sys_Para.Speed_MX_Fan = 12500;
			}
			else
			{
				mode_fan = 0;
				Sys_Para.Speed_MX_Fan = 16500;
			}
			OLED_ShowNum(4, 7, (Sys_Para.Speed_MX_Fan / 10), 4);
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
		}
		break;
	default:
		break;
	}
}

/*按键处理 —— CAN通讯测试*/
void key_process_cantest(void)
{
	uint8_t KeyState = 0;
	float temp_cantx = 122.2;
	uint32_t temp_cantx2 = 255;
	KeyState = Key_GetState();
	switch (KeyState)
	{
	case SW2_SINGLE_CLICK:
		CAN_Data.TxID = 0x00000000; //初始化为0
		CAN_Data.TxID |= ((ID_ThisBoard & 0x7F) << 4);
		CAN_Data.TxID |= (0x0E & 0x0F); //上传第一组转速数据
		memcpy(&CAN_Data.TxData[0], &temp_cantx, 4);
		memcpy(&CAN_Data.TxData[4], &temp_cantx2, 4);
		MyCAN_Transmit(CAN_Data.TxID, CAN_Data.TxData, 8);
		OLED_ShowHexNum(4, 6, CAN_Data.TxID, 3);
		OLED_ShowHexNum(3, 1, CAN_Data.TxData[4], 1);
		OLED_ShowHexNum(3, 2, CAN_Data.TxData[5], 1);
		OLED_ShowHexNum(3, 3, CAN_Data.TxData[6], 1);
		OLED_ShowHexNum(3, 4, CAN_Data.TxData[7], 1);
		break;
	case SW2_LONG_PRESS:
		CAN_Data.TxID = 0x00000000;
		CAN_Data.TxID |= ((ID_ThisBoard & 0x7F) << 4);
		CAN_Data.TxID |= (0x0E & 0x0F); //上传第一组转速数据
		MyCAN_Transmit(CAN_Data.TxID, CAN_Data.TxData, 8);
		OLED_ShowHexNum(4, 6, CAN_Data.TxID, 3);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		break;
	case SW3_SINGLE_CLICK:
		CAN_Data.TxID = 0x00000000;
		CAN_Data.TxID |= (0x7F << 4);
		CAN_Data.TxID |= (0x0A & 0x0F); //请求重发跟踪曲线
		MyCAN_Transmit(CAN_Data.TxID, CAN_Data.TxData, 0);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		break;
	case SW3_LONG_PRESS:
		break;
	default:
		break;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
