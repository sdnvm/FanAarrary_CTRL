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
#include "protocol.h"
#include "bsp_pid.h"
#include "control.h"
//#include "sin_control.h"
#include <string.h>
#include "OLED.h"
#include "sin_control.h"
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

uint8_t KeyNum;
uint8_t KeyState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Para_init(void); //初始化结构体参数
void key_process(void); //按键扫描与处理函数
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	protocol_init();
	PID_param_init();
	OLED_Init();
	OLED_Clear();
	Para_init();
	Start_Timer();
	HAL_CAN_Start(&hcan);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);

	uint16_t curve_sum_receveied;
	float temp_cantx = 122.2;
	uint32_t temp_cantx2 = 255;
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
	OLED_ShowString(4,1,"open");
	OLED_ShowNum(3, 1, (uint32_t)(pid_para[3].Kp*1000),4);
	OLED_ShowNum(3, 6, (uint32_t)(pid_para[3].Ki*1000),4);
	OLED_ShowNum(3, 11, (uint32_t)(pid_para[3].Kd*1000),4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(TrackCurve_Para.start_cacul)
		{
			cacul_sin_params();
		}
		receiving_process();
		key_process();

		/* FIFO0扫描接收 */
		CAN_Para.PendingMsg = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
		while (CAN_Para.PendingMsg > 0)
		{
			if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &CAN_Data.can_RxHeader, CAN_Data.RxData) == HAL_OK) //接收成功
			{
				uint16_t cmd_can;
				OLED_ShowHexNum(1, 6, CAN_Data.can_RxHeader.StdId, 8);
				OLED_ShowString(2, 10, "OK");
				switch (CAN_Data.can_RxHeader.StdId)
				{
					case 0x001: //广播CMD
					case (ID_ThisBoard << 4) | 0x001: //单播CMD
						memcpy(&cmd_can, &CAN_Data.RxData[0], 2);
						switch (cmd_can)
						{
							case 0x000:	//复位
								HAL_NVIC_SystemReset();
								break;
							case 0x001:	//风机停止
								memset(Ctrl_Para.enable_fan, false, sizeof(Ctrl_Para.enable_fan));
								for (int i = 1; i < 5; i++)
								{
									set_rpm(i, 0);
								}
								HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
								break;
							case 0x002:	//风机启动
								memset(Ctrl_Para.enable_fan, true, sizeof(Ctrl_Para.enable_fan));
								break;
							case 0x003:	//pid关闭
							case 0x004:	//pid启动
								Ctrl_Para.enable_pid[0] = cmd_can - 3;
								break;
							case 0x005: //失能跟踪曲线
							case 0x006: //使能跟踪曲线
								TrackCurve_Para.enable_TrackCurve = cmd_can - 5;
								break;
							case 0x007: //保存参数
								//todo:数据保存置flash
								HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
								break;
							case 0x008: //上传一次本机状态
								//todo：上传本机当前状态
							case 0x009: //enable|disable上传转速
								if (Sys_Para.enable_send == false)
								{
									Sys_Para.enable_send = true;
								}
								else
								{
									Sys_Para.enable_send = false;
								}
								HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
								break;
							default:
								break;
					}
				case 0x004: //pid参数首帧
					//memcpy(&, &can_RxData[2], 2);//T采样
					for (int i = 0; i < 4; i++)
					{
						memcpy(&pid_para[i].Kp, &CAN_Data.RxData[4], 4); //p
					}
					OLED_ShowNum(2, 8, (uint16_t)(pid_para[0].Kp * 1000.0f), 4);
					break;
				case 0x005: //pid参数尾帧
					for (int i = 0; i < 4; i++)
					{
						memcpy(&pid_para[i].Ki, CAN_Data.RxData, 4); //i
						memcpy(&pid_para[i].Kd, &CAN_Data.RxData[4], 4); //d
						HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
					}
					OLED_ShowNum(3, 6, (uint16_t)(pid_para[0].Ki * 1000.0f), 4);
					OLED_ShowNum(3, 11, (uint16_t)(pid_para[0].Kd * 1000.0f), 4);
				case (ID_ThisBoard << 4) | 0x03: //目标转速
					for (int i = 0; i < 4; i++)
					{
						memcpy(&Ctrl_Para.rpm_want[i], &CAN_Data.RxData[i * 2], 2);
						set_rpm(i + 1, (float)Ctrl_Para.rpm_want[i]);
						OLED_ShowNum(3, 1 + i * 4, Ctrl_Para.rpm_want[i], 4);
					}
					break;
				case (ID_ThisBoard << 4) | 0x06: //跟踪曲线相位
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
					memcpy(&TrackCurve_Para.num_data, CAN_Data.RxData, 2); //帧个数
					memcpy(&TrackCurve_Para.sample_time, &CAN_Data.RxData[2], 2); //采样时间ms
					memset(&TrackCurve_Data.curve_data, 0xF, sizeof(TrackCurve_Data.curve_data)); //初始化接收缓冲区
					TrackCurve_Para.is_assembling = true;//开始接收
					TrackCurve_Para.num_data_received = 0;//初始化已收到数据量
					//todo:关闭风机、关闭高消耗中断，进入接收状态
					break;
				case 0x007: //curve中间帧
					if (!TrackCurve_Para.is_assembling)
					{
						//todo:申请重新发数据
						break;
					}
					memcpy(&TrackCurve_Data.curve_data[TrackCurve_Para.num_data_received], CAN_Data.RxData, 4); //
					memcpy(&TrackCurve_Data.curve_data[TrackCurve_Para.num_data_received + 1], &CAN_Data.RxData[4], 4); //
					TrackCurve_Para.num_data_received += 2;
					break;
				case 0x008: //curve尾帧
					//数据完整性判断
					if (TrackCurve_Para.num_data_received - TrackCurve_Para.num_data == 1 | TrackCurve_Para.num_data_received == TrackCurve_Para.num_data)
					{
						TrackCurve_Para.is_assembling = false;
					}
					else
					{
						MyCAN_Transmit(0x7FA, NULL, 0); //数据缺失，请求重收
					}
					break;
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
	Sys_Para.ctrl_mode = 0;
	Sys_Para.timer_ctrl_T = 250;
	Sys_Para.timer_fg_f = 12;//mhz
	Sys_Para.timer_pwm_ARR = 4714;

	/*控速相关变量*/
	memset(&Ctrl_Para.enable_pid, 0, sizeof(Ctrl_Para.enable_pid));
	memset(&Ctrl_Para.rpm_want, 0, sizeof(Ctrl_Para.rpm_want));

	/*CAN相关变量*/
	//ID_ThisBoard去main.h里改
	CAN_Data.TxID = ID_ThisBoard << 4;
	uint8_t temp_txid[8] = {0x11, 0x22, 0x33, 0x44};
	memcpy(CAN_Data.TxData, temp_txid, sizeof(CAN_Data.TxData));
	/*跟踪曲线相关变量*//*系统相关变量*/
}
//按键扫描与处理
void key_process(void)
{
	uint8_t CH = 0;
	KeyState = Key_GetState();
	switch (KeyState)
	{
		case SW2_SINGLE_CLICK:
			memset(Ctrl_Para.enable_fan, 0, sizeof(Ctrl_Para.enable_fan));
			for(int i =1; i < 5; i ++)
			{
				CH = i;
				set_rpm(i, (float)Ctrl_Para.rpm_want[i-1]);
				pid_reset(i, Ctrl_Para.rpm_want[i-1]);
			}
		CH = 1;
		OLED_ShowNum(CH, 1, Ctrl_Para.rpm_want[CH-1], 5);
		OLED_ShowString(4, 1, "Open");
		//send_computer(0x01,0x01,&rpm_want2,1);
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
	    /*CAN_Data.TxID = 0x00000000;
	    CAN_Data.TxID |= ((ID_ThisBoard & 0x7F) << 4);
	    CAN_Data.TxID |= (0x0E & 0x0F);//上传第一组转速数据
		memcpy(&CAN_Data.TxData[0], &temp_cantx, 4);
		memcpy(&CAN_Data.TxData[4], &temp_cantx2, 4);
		MyCAN_Transmit(CAN_Data.TxID,CAN_Data.TxData,8);
		CAN_Data.TxData[0] ++;
		CAN_Data.TxData[1] ++;
		CAN_Data.TxData[2] ++;
		CAN_Data.TxData[3] ++;
	    OLED_ShowHexNum(4, 6, CAN_Data.TxID, 3);
		OLED_ShowHexNum(3, 1,CAN_Data.TxData[4],1 );
		OLED_ShowHexNum(3, 2,CAN_Data.TxData[5],1 );
		OLED_ShowHexNum(3, 3,CAN_Data.TxData[6],1 );
		OLED_ShowHexNum(3, 4,CAN_Data.TxData[7],1 );
	    CAN_Data.TxID |= (0x0F & 0x0F);//上传第二组转速数据
	    MyCAN_Transmit(CAN_Data.TxID, CAN_Data.TxData,8);
	    OLED_ShowHexNum(4, 11, CAN_Data.TxID, 3);
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	    HAL_Delay(1000);
	    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);*/
		break;

		case SW2_LONG_PRESS:
			CH = 1;
			Ctrl_Para.rpm_want[CH-1] = Ctrl_Para.rpm_want[CH-1] + 1000;
			Ctrl_Para.enable_fan[CH] = 1;
			Ctrl_Para.enable_fan[0] = 1;
			set_rpm(CH, (float)Ctrl_Para.rpm_want[CH-1]);
			pid_reset(CH, Ctrl_Para.rpm_want[CH-1]);
			OLED_ShowNum(1, 1, Ctrl_Para.rpm_want[CH-1],5);
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
			/*CAN_Data.TxID = 0x00000000;
		    CAN_Data.TxID |= ((ID_ThisBoard & 0x7F) << 4);
			CAN_Data.TxID |= (0x0E & 0x0F);//上传第一组转速数据
			MyCAN_Transmit(CAN_Data.TxID,CAN_Data.TxData,8);
			OLED_ShowHexNum(4, 6, CAN_Data.TxID, 3);
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			HAL_Delay(1000);
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);*/
		break;

		case SW3_SINGLE_CLICK://切换开闭环
		{
			static uint8_t mode_control = 0;
			if(mode_control == 0)
			{
				mode_control = 1;
				CH = 1;
				pid_reset(CH, Ctrl_Para.rpm_want[CH-1]);
				CH = 2;
				pid_reset(CH, Ctrl_Para.rpm_want[CH-1]);
				CH = 3;
				pid_reset(CH, Ctrl_Para.rpm_want[CH-1]);
				CH = 4;
				pid_reset(CH, Ctrl_Para.rpm_want[CH-1]);
				Ctrl_Para.enable_pid[0] = 1;
				OLED_ShowString(4, 1, "Close");
			}else
			{
				mode_control = 0;
				CH = 1;
				pid_reset(CH, Ctrl_Para.rpm_want[CH]);
				CH = 2;
				pid_reset(CH, Ctrl_Para.rpm_want[CH]);
				CH = 3;
				pid_reset(CH, Ctrl_Para.rpm_want[CH-1]);
				CH = 4;
				pid_reset(CH, Ctrl_Para.rpm_want[CH-1]);
				Ctrl_Para.enable_pid[0] = 0;
				OLED_ShowString(4, 1, "Open");
			}
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);

		   /*CAN_Data.TxID = 0x00000000;
			CAN_Data.TxID |= (0x7F << 4);
		    CAN_Data.TxID |= (0x0A & 0x0F);//请求重发跟踪曲线
			MyCAN_Transmit(CAN_Data.TxID, CAN_Data.TxData, 0);
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			HAL_Delay(1000);
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);*/
		}break;

		case SW3_LONG_PRESS:
		{
			static uint8_t mode_fan = 0;
			if(mode_fan == 0){
				mode_fan = 1;
				Sys_Para.Speed_MX_Fan = 12500;
			}else{
				mode_fan = 0;
				Sys_Para.Speed_MX_Fan = 16500;
			}
			OLED_ShowNum(4, 7, (Sys_Para.Speed_MX_Fan / 10),4);
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
		}break;
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
