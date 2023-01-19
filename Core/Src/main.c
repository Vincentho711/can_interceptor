/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// CAN Transmit header structure
CAN_TxHeaderTypeDef TxHeader;
// CAN Receive header structure
CAN_RxHeaderTypeDef RxHeader;

// Array to store Tx,Rx data
uint8_t TxData[8];
uint8_t RxData[8];

// Variable for TxMailbox
uint32_t TxMailbox;

// Buffer for printing out strings using UART
char uart_buf[50];
int uart_buf_len;

// A flag for whether the correct Rx message has been received
int datacheck = 0;

// Ticker to count elapsed time since program starts
uint32_t ticker = 0;

// Handle received messages on the CAN bus
// When this controller receives message, a message pending callback will be called
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	// Get the Rx message in the Rx FIFO
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	// Check the data length of the received message
	// Check if it is from 446
	if (RxHeader.StdId == 0x446)
	{
		// Check the data length of the received message
		if (RxHeader.DLC == 2)
		{
			// Set flag
			datacheck = 1;
		}

	}
	ticker = HAL_GetTick();
	// Print the received message
	uart_buf_len = sprintf(uart_buf,
			"Elapsed Time: %d ms \r\n",(unsigned int)ticker);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

	uart_buf_len = sprintf(uart_buf,
			"CAN ID: 0x%02x \r\n",(unsigned int)RxHeader.StdId);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
	uart_buf_len = sprintf(uart_buf,
			"---- \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
	for (int i = 0; i < RxHeader.DLC; i++)
	{
		uart_buf_len = sprintf(uart_buf,
				"0x%02x \r\n",(unsigned int)RxData[i]);
		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
	}
	uart_buf_len = sprintf(uart_buf,
			" \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Print Start message
  strcpy((char*)uart_buf,"Start of debug session \r\n");
  HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf),50);
  strcpy((char*)uart_buf,"=============================== \r\n");
  HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf),50);

  // Start CAN
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
	  Error_Handler();
  }

  // Activate the notification for data pending in the Rx FIFO
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }

  // Load data in the TxHeader
  // Sending 2 data bytes
  TxHeader.DLC = 2;	// data length
  TxHeader.IDE = CAN_ID_STD; // Type of identifier for the message that will be transmitted
  TxHeader.RTR = CAN_RTR_DATA; // Type of frame that will be transmitted
  TxHeader.StdId = 0x103; // Standard Identifier, ID of sender

  // The first byte contains the delay for the LED on 2nd board
  // The second byte will act as the number of times the LED will blink
  TxData[0] = 50; //ms delay
  TxData[1] = 20; // loop rep

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // If the data length flag has been set after Rx, blink blue LED at PB7
	  if (datacheck)
	  {
		  for (int i =0;i<RxData[1];i++)
		  {
			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
			  HAL_Delay(RxData[0]);
		  }

		  // Reset the flag
		  datacheck = 0;
		  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  // Configure the CAN filter on this board
  // Establish filter config object
  CAN_FilterTypeDef canfilterconfig;
  // Enable the CAN filter
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  // How many filter banks to assign to the CAN1
  canfilterconfig.SlaveStartFilterBank = 14;
  // In case of single instance of CAN peripheral, 14 filter banks are available
  // to filter CAN1 messages. Every filter bank has two 32-bit registers.
  // We can assign any filter 0-13 to any FIF1 for CAN1.
  canfilterconfig.FilterBank = 0;
  // Assign a FIFO0 to the CAN filter
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;


  // https://www.youtube.com/watch?v=NS3eeZPUDns for a detail explanation
  // In "Mask Mode", the mask register controls which bit of the CAN ID will be checked.
  // A CAN ID is 11 bit long,
  // so if mask register = 111 0000 0000, then only the 3 MSB will be check.
  // If we only want messages with CAN ID = 001 XXXX XXXX to pass through,
  // then set the 3 MSB of id register = 001 0000 0000, using mask mode allows us to
  // let a range of CAN ID pass.

  // "ID Mode" is better if we just want to accept particular CAN ID exclusively.
  // E.g. If we just want to accept CAN ID = 0x65D and 0x651. We set mode to "ID Mode" and
  // both 32-bits register will be used as id register. Now we just store those 2 IDs in the registers.

  // In the STM32F4, each 32-bit id register and mask register is formed by 2 16-bit register.
  // FilterMaskIdHigh sets the 16 MSBs of the mask register while FilterMaskIdLow sets the
  // 16 LSBs of the mask register. The standard ID occupies the 11 MSBs of the message.
  // Same goes with id register.

  // E.g. The sender has CAN ID = 001 0000 0011 = 0x103
  // Using "Mask Mode", the 16 MSBs of the mask register should be 0010 0000 0110 0000 = 0x103<<5
  // to set up the relevant bit to be checked. 16 LSBs of the mask register can be set to all 0s.
  // To allow one ID to pass in "Mask Mode", those bits that are checked has to be the same as the
  // CAN ID, so 16 MSBs of id register = 0x103<<5. 16 LSBs = 0x000.

  // Using "ID Mode", just set 16 MSBs of both register to 0x446 and 16 LSBs to 0x000 to receive from F446.

  // Filter banks can be in either "Mask Mode" or "ID Mode"
  // We use "ID Mode" in this case
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  // FilterScale sets the number of bits for filter registers, we will use 32 bits
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  // Set filter bank so that only a single CAN ID = 0x446 pass through
  canfilterconfig.FilterMaskIdHigh = 0x000<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterIdHigh = 0x000<<5;
  canfilterconfig.FilterIdLow = 0x0000;

  // Finally, we initialise the filter object
  if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig)!=HAL_OK)
  {
	  Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

