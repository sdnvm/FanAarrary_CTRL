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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_pid.h"
#include "control.h"
#include "sin_control.h"
#include "save_flash.h"
#include <string.h>
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
SysState Sys_State;
SysPara Sys_Para;
CtrlPara Ctrl_Para;
CANPara CAN_Para;
CANData CAN_Data;
TrackCurvePara TrackCurve_Para;
TrackCurveData TrackCurve_Data;
TrackCurveData* TrackCurve_Data_p = &TrackCurve_Data;
PidPara_base pid_para_base;//pid基础参数
bool oled_init = true;//第一次调用或更改模式时，重写oled屏幕背景内容
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
static void Para_init(void); //初始化结构体参数
static void key_process_idsetting(uint8_t keystate); //按键扫描与处理函数
static void key_process_motortest(uint8_t keystate); //按键扫描与处理函数
static void key_process_cantest(uint8_t keystate); //按键扫描与处理函数
//void RunMode_Switch(SM_SysRunMode run_mode);//系统运行状态切换
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
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	PID_param_init();
	OLED_Init();
	OLED_Clear();
	Para_init();
	Start_Timer();
	HAL_CAN_Start(&hcan);
	//__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
	RunMode_Switch(S_Wait);//初始状态设为“wait”

	uint8_t KeyState = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(TrackCurve_Para.start_cacul)
		{
			cacul_sin_params();
		}
		if (Sys_Para.enable_key)
		{
			KeyState = Key_GetState();
			switch (Sys_State.manu)
			{
			case id_setting:
				key_process_idsetting(KeyState);
				break;
			case fan_test:
				key_process_motortest(KeyState);
				break;
			case can_test:
				key_process_cantest(KeyState);
				break;
			case para_setting:
				//todo:参数设定
				break;
			default:
				break;
			}
		}

		/* FIFO0扫描接收 */
		CAN_Para.PendingMsg = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
		while (CAN_Para.PendingMsg > 0)
		{
			if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &CAN_Data.can_RxHeader, CAN_Data.RxData) == HAL_OK) //接收成功
			{
				uint8_t cmd_main = CAN_Data.can_RxHeader.StdId & 0x0000000F; //主指令类型（4bit），截掉id，因为id是变量
				uint16_t cmd_sub; //子指令类型
				if (Sys_State.manu == can_test)
				{
					OLED_ShowHexNum(1, 1, CAN_Data.can_RxHeader.StdId, 3);//打印id
					//OLED_ShowNum(1, 9, CAN_Data.can_RxHeader.DLC , 1);//打印DLC
					/*uint32_t RawData;//原始数据
					memcpy(&RawData, &CAN_Data.RxData[0], 4);
					OLED_ShowHexNum(2, 1, RawData, 8);//打印原始data1
					memcpy(&RawData, &CAN_Data.RxData[4], 4);
					OLED_ShowHexNum(2, 9, RawData, 8);//打印原始data2*/
				}
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				switch (cmd_main)
				{
				/*复位、启停*/
				case 0x01:
					memcpy(&cmd_sub, &CAN_Data.RxData[0], 2);//cmd是数据段2字节
					switch (cmd_sub)
					{
					case 0x000: //复位
						HAL_NVIC_SystemReset();
						break;
					case 0x001: //风机停止
						RunMode_Switch(S_Wait);
						break;
					case 0x002: //风机启动
						RunMode_Switch(S_NormRuning);
						break;
					default:
						break;
					}
					break;
				/*使能/失能、保存参数*/
				case 0x02: //
					memcpy(&cmd_sub, &CAN_Data.RxData[0], 2);//cmd是数据段2字节
					switch (cmd_sub)
					{
					case 0x003: //pid关闭
					case 0x004: //pid启动
						RunMode_Switch(S_Wait);//停机
						Ctrl_Para.enable_pid[0] = cmd_sub - 3;//pid总使能位赋值
						break;
					case 0x005: //失能跟踪曲线
					case 0x006: //使能跟踪曲线
						TrackCurve_Para.enable_TrackCurve = cmd_sub - 5;
						if (TrackCurve_Para.enable_TrackCurve)
						{
							RunMode_Switch(S_TrackingCurve);
						}
						else
						{
							RunMode_Switch(S_Wait);
						}
						break;
					case 0x007: //保存参数
						if (Flash_SaveSysPara(&Sys_Para, &pid_para_base) == HAL_OK)
						{
							OLED_ShowString(4, 7, "S_OK");
						}
						else
						{
							MyCAN_Transmit_enum(TX_Err_SavePara);
							OLED_ShowString(4, 7, "S_ER");
						}
						break;
					case 0x008: //上传一次本机状态
						MyCAN_Transmit_enum(TX_SYSPara);
						break;
					case 0x009: //使能上传转速
						Sys_Para.enable_send = !Sys_Para.enable_send;
						break;
					case 0x00A://使能oled显示
						Sys_Para.enable_OLED = !Sys_Para.enable_OLED;
						break;
					case 0x00B:
						Sys_Para.enable_key = !Sys_Para.enable_key;
					default:
						break;
					}
					break;
				/*目标转速首帧、尾帧*/
				case 0x03: //
					for (int i = 0; i < 2; i++)
					{
						memcpy(&Ctrl_Para.rpm_want[i], &CAN_Data.RxData[i * 4], 4);
					}
					break;
				case 0x04: //目标转速尾帧
					for (int i = 0; i < 2; i++)
					{
						memcpy(&Ctrl_Para.rpm_want[i + 2], &CAN_Data.RxData[i * 4], 4);
					}
					for (int i = 0; i < 3; i++)
					{
						OLED_ShowNum(2, 1 + i * 5, (uint32_t)Ctrl_Para.rpm_want[i] * 10, 5);
					}
					break;
				/*跟踪曲线相位*/
				case 0x0A:
					for (int i = 0; i < 4; i++)
					{
						memcpy(&TrackCurve_Para.phase[i], &CAN_Data.RxData[i * 2], 2);
						OLED_ShowNum(2, 1 + i * 4, TrackCurve_Para.phase[i], 4);
					}
					break;
				default:
					break;
				}
			}
			else //接收错误
			{
				MyCAN_Transmit_enum(TX_Err_CAN);
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
		while (CAN_Para.PendingMsg > 0)
		{
			if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO1, &CAN_Data.can_RxHeader, CAN_Data.RxData) == HAL_OK) //接收成功
			{
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				switch (CAN_Data.can_RxHeader.StdId)
				{
				case 0x006: //curve首帧
					{
						RunMode_Switch(S_TrackingCurve);
						memcpy(&TrackCurve_Para.num_data, CAN_Data.RxData, 2); //帧个数
						memcpy(&TrackCurve_Para.sample_time, &CAN_Data.RxData[2], 2); //采样时间ms
						memset(&TrackCurve_Data.curve_data, 0xF, sizeof(TrackCurve_Data.curve_data)); //初始化接收缓冲区
						TrackCurve_Para.is_assembling = true; //开始接收
						TrackCurve_Para.num_data_received = 0; //初始化已收到数据量
						break;
					}
				case 0x007: //curve中间帧
					{
						if (!TrackCurve_Para.is_assembling)
						{
							MyCAN_Transmit_enum(TX_Err_RXTrackData); //未接收首帧，请求重收
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
						if ((TrackCurve_Para.num_data_received - TrackCurve_Para.num_data == 1) || (TrackCurve_Para.
							num_data_received == TrackCurve_Para.num_data))
						{
							TrackCurve_Para.is_assembling = false;
							TrackCurve_Para.enable_TrackCurve = true;
							RunMode_Switch(S_Wait);
						}
						else
						{
							MyCAN_Transmit_enum(TX_Err_RXTrackData); //数据缺失，请求重收
						}
						break;
					}
				/*pid参数*/
				case 0x009:
					{
						static uint8_t rxcount_pid = 0; //接收到的帧个数
						rxcount_pid += 1;
						if (rxcount_pid == 1) //第一帧
						{
							RunMode_Switch(S_Wait);
							//memcpy(&Sys_Para.timer_pwm_ARR, &CAN_Data.RxData[0], 2); //pwm计数器ARR
							memcpy(&Sys_Para.timer_ctrl_T, &CAN_Data.RxData[2], 2); //采样时间
							SamplingTime_Modify(Sys_Para.timer_ctrl_T, 0);
							for (int i = 0; i < 4; i++)
							{
								memcpy(&pid_para_base.Kp, &CAN_Data.RxData[4], 4); //p
							}
						}
						else //第二帧
						{
							for (int i = 0; i < 4; i++)
							{
								memcpy(&pid_para_base.Ki, CAN_Data.RxData, 4); //i
								memcpy(&pid_para_base.Kd, &CAN_Data.RxData[4], 4); //d
								pid_para[i].Kp = pid_para_base.Kp;
								pid_para[i].Ki = pid_para_base.Ki;
								pid_para[i].Kd = pid_para_base.Kd;
							}
							rxcount_pid = 0;
							OLED_ShowString(2, 5, "|    |");
							OLED_ShowNum(2, 1, (uint16_t)(pid_para[0].Kp * 1000.0f), 4);
							OLED_ShowNum(2, 6, (uint16_t)(pid_para[0].Ki * 1000.0f), 4);
							OLED_ShowNum(2, 11, (uint16_t)(pid_para[0].Kd * 1000.0f), 4);
						}
						break;
					}
				default:
					break;
				}
			}
			else //接收错误
			{
				MyCAN_Transmit_enum(TX_Err_CAN);
				HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
				HAL_Delay(500);
				HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			}
			CAN_Para.PendingMsg--;
		}
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
	if (!Flash_LoadSysPara(&Sys_Para, &pid_para_base)) //读取flash,若失败则使用默认参数
	{
		/*基本变量*/
		Sys_State.RunMode = S_Wait;
		Sys_State.manu = id_setting;
		Sys_Para.ID_Thisboard = 0x00000003;
		//Sys_Para.Speed_MX_Fan = 17500;
		Sys_Para.enable_send = false;
		Sys_Para.enable_OLED = true;
		Sys_Para.enable_key = true;
		Sys_Para.enable_pid = false;
		Sys_Para.timer_ctrl_T = 200;
		Sys_Para.timer_fg_f = 12;//mhz
		//Sys_Para.timer_pwm_ARR = 4130;
		/*pid相关变量*/
		pid_para_base.Kp = 0.1f;
		pid_para_base.Ki = 0.035f;
		pid_para_base.Kd = 0.27f;
		set_p_i_d(pid_para_base.Kp, pid_para_base.Ki, pid_para_base.Kd);
		Ctrl_Para.enable_pid[0] = Sys_Para.enable_pid;
		MyCAN_Transmit_enum(TX_Err_SavePara);
	}
	/*CAN相关变量*/
	CAN_Para.PendingMsg = 0;
	/*跟踪曲线相关变量*//*系统相关变量*/
}
//状态切换
void RunMode_Switch(SM_SysRunMode run_mode)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	switch (run_mode)
	{
	case S_Wait://停机等待，进行数据接收等
		for (int i = 0; i < 4; i++)
		{
			Ctrl_Para.enable_fan[i] = false;
			set_rpm_incloseloopmode(i, 0);//目标转速置0
		}
		OLED_ShowString(4, 1, "wait ");
		OLED_ShowNum(4, 5, Ctrl_Para.enable_pid[0], 1);//0==开环，1==闭环
		break;
	case S_NormRuning://风机正常运行-开闭环。要提前配置rpm_want&pid
		OLED_ShowString(4, 1, "norm ");
		OLED_ShowNum(4, 5, Ctrl_Para.enable_pid[0], 1);//0==开环，1==闭环
		for (int i = 0; i < 4; i++)
		{
			set_rpm_incloseloopmode(i, (float)Ctrl_Para.rpm_want[i]);//传入目标转速
		}
		for (int i = 0; i < 4; i++)
		{
			Ctrl_Para.enable_fan[i] = true;
		}
		break;
	case S_TrackingCurve://跟踪曲线模式
		OLED_ShowString(4, 1, "track");
		break;
	default:
		break;
	}
	Sys_State.RunMode = run_mode;
}
/*按键处理 —— 设定本机id*/
void key_process_idsetting(uint8_t keystate)
{
	if (oled_init)
	{
		OLED_Clear();
		OLED_ShowString(1, 1, "MAX:");OLED_ShowString(1, 10, "| ID:");
		OLED_ShowHexNum(1, 15, Sys_Para.ID_Thisboard, 2);
		OLED_ShowNum(1, 5, FANSPEED_MAX, 5);
		OLED_ShowString(2, 5, "|    |");
		OLED_ShowNum(2, 1, (uint32_t)(pid_para[3].Kp*1000),4);
		OLED_ShowNum(2, 6, (uint32_t)(pid_para[3].Ki*1000),4);
		OLED_ShowNum(2, 11, (uint32_t)(pid_para[3].Kd*1000),4);
		OLED_ShowString(4, 1, "wait");OLED_ShowString(4, 6, "|S_id|");OLED_ShowString(4, 15, "ms");
		OLED_ShowNum(4, 12, Sys_Para.timer_ctrl_T, 3);
		oled_init = false;
	}
	switch (keystate)
	{
	case SW2_SINGLE_CLICK: //+1
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		Sys_Para.ID_Thisboard += 1;
		break;
	case SW2_LONG_PRESS: //+10
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		Sys_Para.ID_Thisboard += 10;
		break;
	case SW3_SINGLE_CLICK: //-1
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		if (Sys_Para.ID_Thisboard >= 1)
		{
			Sys_Para.ID_Thisboard -= 1;
		}
		else
		{
			break;
		}
		break;
	case SW3_LONG_PRESS: //-10
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		if (Sys_Para.ID_Thisboard >= 10)
		{
			Sys_Para.ID_Thisboard -= 10;
		}
		else
		{
			break;
		}
		break;
	case SW3_LONGLONG_PRESS: //重新配置can并保存到flash
		{
			uint8_t result_sw3 = 0;
			result_sw3 = HAL_CAN_Stop(&hcan);
			if (result_sw3 == HAL_OK)
			{
				filter_init();
				HAL_CAN_Start(&hcan);
			}
			else
			{
				OLED_ShowNum(4, 7, result_sw3, 1);
				break;
			}
			result_sw3 = (Flash_SaveSysPara(&Sys_Para, &pid_para_base));
			if (result_sw3 == HAL_OK)
			{
				OLED_ShowString(4, 7, "S_OK");
			}
			else
			{
				OLED_ShowNum(4, 7, result_sw3, 1);
				// OLED_ShowString(4, 7, "S_ER");
			}
			break;
		}
	default:
		return;
	}
	OLED_ShowHexNum(1, 15, Sys_Para.ID_Thisboard, 2);
}
/*按键处理 —— 电机测试*/
void key_process_motortest(uint8_t keystate)
{
	if (oled_init)
	{
		OLED_Clear();
		OLED_ShowString(4, 1, "wait |Tctr|");
		oled_init = false;
	}
	switch (keystate)
	{
	case SW2_SINGLE_CLICK://停机
		RunMode_Switch(S_Wait);
		for (int i = 0; i < 4; i++)
		{
			Ctrl_Para.rpm_want[i] = 0;
			OLED_ShowNum(2, 1 + i * 4, (uint32_t)Ctrl_Para.rpm_want[i], 4);
		}
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		break;
	case SW2_LONG_PRESS: //开闭环切换
		{
			static bool mode_control = 0;
			RunMode_Switch(S_Wait); //停机
			Ctrl_Para.enable_pid[0] = mode_control; //pid总使能位赋值
			mode_control = !mode_control;
			RunMode_Switch(S_Wait); //停机，并更新开闭环
			break;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}
		break;
	case SW3_SINGLE_CLICK: //转速-
		for (int i = 0; i < 4; i++)
		{
			Ctrl_Para.rpm_want[i] -= 1000;
			OLED_ShowNum(2, 1 + i * 4, (uint32_t)Ctrl_Para.rpm_want[i], 4);
		}
		RunMode_Switch(S_NormRuning);
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		break;
	case SW3_LONG_PRESS:
		for (int i = 0; i < 4; i++)
		{
			Ctrl_Para.rpm_want[i] += 1000;
			OLED_ShowNum(2, 1 + i * 4, (uint32_t)Ctrl_Para.rpm_want[i], 4);
		}
		RunMode_Switch(S_NormRuning);
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		break;
	default:
		break;
	}
}
/*按键处理 —— CAN通讯测试*/
void key_process_cantest(uint8_t keystate)
{
	if (oled_init)
	{
		OLED_Clear();
		OLED_ShowString(1, 1, " id|");
		OLED_ShowString(2, 1,"---data(hex)----");
		OLED_ShowString(4, 1, "wait |Tcan|");
		oled_init = false;
	}
	float temp_cantx = 15222.2f;
	float temp_cantx2 = 8990.1f;
	switch (keystate)
	{
	case SW2_SINGLE_CLICK:
		MyCAN_Transmit_RPM(temp_cantx, temp_cantx2, 1231,1200, 1);
		MyCAN_Transmit_RPM(3000, 4000, 5000,6000, 3);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		break;
	case SW2_LONG_PRESS:
		MyCAN_Transmit_enum(TX_Err_RXTrackData);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		break;
	case SW3_LONG_PRESS:
		MyCAN_Transmit_enum(TX_Heartbeat);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
