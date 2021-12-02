/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	if (HAL_I2C_IsDeviceReady(&hi2c1, 0xD0, 10, HAL_MAX_DELAY) == HAL_OK)
	{
		for (int i = 1; i<=10;i++) // indicator of ready device
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
			HAL_Delay(250);
		}
	}
//	if (HAL_I2C_IsDeviceReady(&hi2c3, 0xD0, 10, HAL_MAX_DELAY) == HAL_OK)
//	{
//		for (int i = 1; i<=30;i++) // indicator of ready device
//		{
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
//			HAL_Delay(250);
//		}
//	}
	uint8_t secbuffer[2], minbuffer[2], hourbuffer[2], daybuffer[2], datebuffer[2];
// seconds
	secbuffer[0] = 0x00; //register address
	secbuffer[1] = 0x00; //data to put in register --> 00 sec
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, secbuffer, 2, 10);
// minutes
	minbuffer[0] = 0x01; //register address
	minbuffer[1] = 0x40; //data to put in register --> 40 min
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, minbuffer, 2, 10);
// hours
	hourbuffer[0] = 0x02; //register address
	hourbuffer[1] = 0x11; //data to put in register 01001001 --> 11 am
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, hourbuffer, 2, 10);
	
	daybuffer[0] = 0x03;
	daybuffer[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, daybuffer, 2, 10);

	datebuffer[0] = 0x04;
	datebuffer[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, datebuffer, 2, 10);

// Transmit via I2C to set alarm 2
	uint8_t status_reg[2], ctrl_reg[2];
	uint8_t alarm_minbuffer[2], alarm_hourbuffer[2], alarm_daybuffer[2], alarm_datebuffer[2];
	int count = 1;
	//char t[1];
	//while (HAL_UART_Receive(&huart1, (uint8_t *)t, 1, HAL_MAX_DELAY) != HAL_OK){} 
	//count = atoi(t);
	//while (HAL_UART_Transmit(&huart1, (uint8_t *)count, 1, HAL_MAX_DELAY) != HAL_OK){}	
	//HAL_UART_Transmit(&huart1, (uint8_t *)count, 1, HAL_MAX_DELAY);
// status register
	status_reg[0] = 0x0F;
	status_reg[1] = 0x88;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, status_reg, 2, 10);
// control register
	ctrl_reg[0] = 0x0E;
	ctrl_reg[1] = 0x1E; 
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, ctrl_reg, 2, 10);
// minutes
	alarm_minbuffer[0] = 0x0B;
	alarm_minbuffer[1] = 0x41;
	alarm_minbuffer[1] += count;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, alarm_minbuffer, 2, 10);
// hours
	alarm_hourbuffer[0] = 0x0C;
	alarm_hourbuffer[1] = 0x11;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, alarm_hourbuffer, 2, 10);
	
	alarm_daybuffer[0] = 0x0D;
	alarm_daybuffer[1] = 0xC0;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, alarm_daybuffer, 2, 10);
	
	alarm_datebuffer[0] = 0x0D;
	alarm_datebuffer[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, alarm_datebuffer, 2, 10);

//Receive via I2C and forward to UART
	uint8_t h1,h2,m1,m2,s1,s2;
	char uartBuf [100] = {0};
	uint8_t a2f;

	uint8_t yes[] = {'Y', 'E', 'S', '\n', '\r'};
	uint8_t no[] = {'N', 'O', '\n', '\r'};
	
	uint8_t mode_reg[2];
	uint8_t fifo_data[2];
	uint8_t result[4];
	fifo_data[0] = 0x05;
	mode_reg[0] = 0x06;
	mode_reg[1] = 0x42;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, mode_reg, 2, 10);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
//		HAL_I2C_Master_Transmit(&hi2c1, 0xD0, fifo_data, 1, 10);
//		for (int i = 0; i < 4; i++)
//		{
//			HAL_I2C_Master_Receive(&hi2c1, 0xD1, fifo_data+1, 1, 10);
//			result[i] = fifo_data[1];
//		}
//		HAL_UART_Transmit(&huart1, result, 4, HAL_MAX_DELAY);
		
		//send seconds register address 00h to read from
		HAL_I2C_Master_Transmit(&hi2c1, 0xD0, secbuffer, 1, 10);
		//read data of register 00h to secbuffer[1]
		HAL_I2C_Master_Receive(&hi2c1, 0xD1, secbuffer+1, 1, 10);
		//prepare UART output
		s1 = secbuffer[1] >> 4;
		s2 = secbuffer[1] & 0x0F;
		HAL_I2C_Master_Transmit(&hi2c1, 0xD0, minbuffer, 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, 0xD1, minbuffer+1, 1, 10);
		m1 = minbuffer[1] >> 4;
		m2 = minbuffer[1] & 0x0F;
		HAL_I2C_Master_Transmit(&hi2c1, 0xD0, hourbuffer, 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, 0xD1, hourbuffer+1, 1, 10);
		h1 = (hourbuffer[1] >> 4) & 1;
		h2 = hourbuffer[1] & 0x0F;
		// transmit time to UART
		sprintf(uartBuf, "%d%d:%d%d:%d%d\r\n",h1,h2,m1,m2,s1,s2);
		HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, sizeof(uartBuf), 10);
		HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &status_reg[0], 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, 0xD1, &a2f, 1, 10);
		
		if (a2f & 0x02)
		{
			alarm_minbuffer[1] += count;	
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, alarm_minbuffer, 2, 10);
		}
		
		HAL_I2C_Master_Transmit(&hi2c1, 0xD0, status_reg, 2, 10);
		
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 0)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
				HAL_UART_Transmit(&huart1, yes, 5, 100);
			}
		else 
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
				HAL_UART_Transmit(&huart1, no, 4, 100);
			}
		
		HAL_Delay(1000);
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
  RCC_OscInitStruct.PLL.PLLN = 40;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
