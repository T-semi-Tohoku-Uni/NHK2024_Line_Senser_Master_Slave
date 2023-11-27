/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "filter.h"
#include "R2CANIDList.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
unsigned char txBuf[2] = {0b01111000, 0x00}; // 痩身用のバッファ
unsigned char rxBuf[2]; // 受診用のバッファ
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
/*
 * アナログ・デジタル変換器からSPI通信でデータを読み込む
 *
 * @param bufList それぞれのセンサのバッファを配列にしたもの
 * @param sensorWhiteList それぞれのセンサの白（初期化時に設定される値）
 * @oaram sensorBlackList それぞれのセンサの黒（初期化時に設定される値）
 *
 * @return void
*/
void ReadADCCChannel(NHK2024_Filter_Buffer **bufList, unsigned int* sensorWhiteList, unsigned int* sensorBlackList);

/*
 * CAN通信で上位のマイコンにデータを送る関数
 *
 * @param Identifier CANID
 * @oaram DataLength データ長
 * @param TxData 送るデータ
 */
void SendMessageOnCAN(uint32_t Identifier, uint32_t DataLength, uint8_t* TxData);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ReadADCCChannel(NHK2024_Filter_Buffer **bufList, unsigned int* sensorWhiteList, unsigned int* sensorBlackList) {
	// センサ0からセンサ7までのNCCのリスト
	uint16_t sensorList[8] = {
			Sensor1_Pin,
			Sensor2_Pin,
			Sensor3_Pin,
			Sensor4_Pin,
			Sensor5_Pin,
			Sensor6_Pin,
			Sensor7_Pin,
			Sensor8_Pin
	};
	GPIO_TypeDef* sensorPort[8] = {
			Sensor1_GPIO_Port,
			Sensor2_GPIO_Port,
			Sensor3_GPIO_Port,
			Sensor4_GPIO_Port,
			Sensor5_GPIO_Port,
			Sensor6_GPIO_Port,
			Sensor7_GPIO_Port,
			Sensor8_GPIO_Port
	};

	// センサの値を格納する
	double filterdSensorVal[8];

	for(int pin = 0; pin < 8; pin++ ) {
		HAL_GPIO_WritePin(sensorPort[pin], sensorList[pin], GPIO_PIN_RESET); // 通信するスレーブを選択する
		HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY); // SPI通信
		HAL_GPIO_WritePin(sensorPort[pin], sensorList[pin], GPIO_PIN_RESET); // スレーブの選択を解除する

		unsigned int sensorVal = ((rxBuf[0] & 0x03) << 8) + rxBuf[1]; // デジタル値のセンサの値を取得する（10bit）
		// 初めに取得した白(3.3V)と黒(0V)の値を使ってセンサの値をスケールする
		double scaledSensorVal;
		if (sensorVal >= sensorWhiteList[pin]) {
			scaledSensorVal = sensorWhiteList[pin];
		} else if (sensorVal <= sensorBlackList[pin]) {
			scaledSensorVal = sensorBlackList[pin];
		} else {
			scaledSensorVal = (sensorVal - sensorBlackList[pin]) * 1024 / (sensorWhiteList[pin] - sensorBlackList[pin]);
		}

		// ローバスフィルタを適応する
		filterdSensorVal[pin] = moving_average_filter_update(bufList[pin], (double) scaledSensorVal);

		// デバッグ用の出力を書くところ
	}

	// 横ずれを中央４つのセンサを使って検出する
	/*
	 * diff = (右の二つのセンサの値の和) - (左の二つのセンサの値の和)
	 * if diff > 0 then ロボットがラインに対して右にずれている
	 * if diff < 0 then ロボットがラインに対して左にずれている
	 *
	 */
	float horizontalOffset = (filterdSensorVal[2 - 1] + filterdSensorVal[6 - 1]) - (filterdSensorVal[3 - 1] + filterdSensorVal[7 - 1]);

	// 角度のずれを中央4つのセンサを使って検出する
	/*
	 *　いいやり方が見つからない
	 * */

	// 水平ライン検出
	/*
	 * 上の4つのセンサと下の四つのセンサを使って水平ラインを検出する
	 * 多分, 8つのセンサのうちそれぞれの側（左右）で一つづつセンサが反応すればOKかな
	 * 後で実装する
	 */
	uint8_t verticalLineDetector = 0;

	// CAN通信で上位の基盤に送る
	// 横ずれ
	uint8_t horizontalOffsetTxData[4];
	memcpy(horizontalOffsetTxData, &horizontalOffset, sizeof(float));
	SendMessageOnCAN(0x00, FDCAN_DLC_BYTES_4, horizontalOffsetTxData);

	// 水平ライン検出
	uint8_t verticalLineDetectorTxData[1] = { // 意味はないけど, 今後のバグ防止
			verticalLineDetector
	};
	SendMessageOnCAN(0x01, FDCAN_DLC_BYTES_1, verticalLineDetectorTxData);
	return;
}

void SendMessageOnCAN(uint32_t Identifier, uint32_t DataLength, uint8_t TxData[3]) {
	FDCAN_TxHeaderTypeDef TxHeader;
//	HAL_StatusTypeDef HAL_ret;
	TxHeader.Identifier = Identifier;                 // ID
	TxHeader.IdType = FDCAN_STANDARD_ID;         // 標準ID
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;     // データフレーム
	TxHeader.DataLength = DataLength;     // 3バイトデータ
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK){
			Error_Handler();
	}

	// FDCANメッセージの送信
//	HAL_ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);

	return;
}
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
  MX_SPI1_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  /*センサの個体差を吸収するため, 初期値を取得する. 後で実装する*/
  /*白*/
  unsigned int sensorWhiteList[8] = {
		  1024, // センサ1
		  1024, // センサ2
		  1024, // センサ3
		  1024, // センサ4
		  1024, // センサ5
		  1024, // センサ6
		  1024, // センサ7
		  1024  // センサ8
  };
  /*黒*/
  unsigned int sensorBlackList[8] = {
		  0, // センサ1
		  0, // センサ2
		  0, // センサ3
		  0, // センサ4
		  0, // センサ5
		  0, // センサ6
		  0, // センサ7
		  0  // センサ8
  };

  /*それぞれのセンサのバッファを初期化*
   *初めに読み取った値で初期化するのが良さそう*/
  NHK2024_Filter_Buffer* bufList[8] = {
		  moving_average_filter_init(0.0, 10), // センサ1
		  moving_average_filter_init(0.0, 10), // センサ2
		  moving_average_filter_init(0.0, 10), // センサ3
		  moving_average_filter_init(0.0, 10), // センサ4
		  moving_average_filter_init(0.0, 10), // センサ5
		  moving_average_filter_init(0.0, 10), // センサ6
		  moving_average_filter_init(0.0, 10), // センサ7
		  moving_average_filter_init(0.0, 10)  // センサ8
  };
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	ReadADCCChannel(bufList, sensorWhiteList, sensorBlackList);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 17;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Sensor8_Pin|Sensor7_Pin|Sensor1_Pin|Sensor6_Pin
                          |Sensor5_Pin|Sensor4_Pin|Sensor3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Sensor2_GPIO_Port, Sensor2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Sensor8_Pin Sensor7_Pin Sensor1_Pin Sensor6_Pin
                           Sensor5_Pin Sensor4_Pin Sensor3_Pin Sensor2_Pin */
  GPIO_InitStruct.Pin = Sensor8_Pin|Sensor7_Pin|Sensor1_Pin|Sensor6_Pin
                          |Sensor5_Pin|Sensor4_Pin|Sensor3_Pin|Sensor2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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