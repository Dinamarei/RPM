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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "math.h"
#include "semphr.h"
#include "task.h"
#include "max30102.h"

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
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SOS */
osThreadId_t SOSHandle;
const osThreadAttr_t SOS_attributes = {
  .name = "SOS",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for HR */
osThreadId_t HRHandle;
const osThreadAttr_t HR_attributes = {
  .name = "HR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RTC */
osThreadId_t RTCHandle;
const osThreadAttr_t RTC_attributes = {
  .name = "RTC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myUartQueue */
osMessageQueueId_t myUartQueueHandle;
const osMessageQueueAttr_t myUartQueue_attributes = {
  .name = "myUartQueue"
};
/* Definitions for sem1 */
osSemaphoreId_t sem1Handle;
const osSemaphoreAttr_t sem1_attributes = {
  .name = "sem1"
};
/* Definitions for sem2 */
osSemaphoreId_t sem2Handle;
const osSemaphoreAttr_t sem2_attributes = {
  .name = "sem2"
};
/* USER CODE BEGIN PV */
	uint8_t secbuffer[2], minbuffer[2], hourbuffer[2], daybuffer[2], datebuffer[2];
  uint8_t h1,h2,m1,m2,s1,s2;
	uint8_t alarm_minbuffer[2], alarm_hourbuffer[2], alarm_daybuffer[2], alarm_datebuffer[2];
	int count = 1;
	uint8_t a2f;
	uint8_t sos[] = {'S', 'O', 'S', '\n', '\r'};
	uint8_t fff[] = {'0','0','0'};
	int xyz =0;
	int secs = 30;
	uint8_t nl[] = {'\n'};
	char hrt[12];
	uint8_t status_reg[2], ctrl_reg[2];
	int first_time=1; 
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
void StartDefaultTask(void *argument);
void startSOS(void *argument);
void getHR(void *argument);
void clkcount(void *argument);

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
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  initialize();
 
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sem1 */
  sem1Handle = osSemaphoreNew(1, 1, &sem1_attributes);

  /* creation of sem2 */
  sem2Handle = osSemaphoreNew(1, 1, &sem2_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myUartQueue */
  myUartQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &myUartQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SOS */
  SOSHandle = osThreadNew(startSOS, NULL, &SOS_attributes);

  /* creation of HR */
  HRHandle = osThreadNew(getHR, NULL, &HR_attributes);

  /* creation of RTC */
  RTCHandle = osThreadNew(clkcount, NULL, &RTC_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10909CEC;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startSOS */
/**
* @brief Function implementing the SOS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startSOS */
void startSOS(void *argument)
{
  /* USER CODE BEGIN startSOS */
  /* Infinite loop */
  for(;;)
	{
		if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0) )
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
			}
		else 
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
				if (xSemaphoreTakeFromISR(sem1Handle, NULL))
				{
					taskENTER_CRITICAL();
					HAL_UART_Transmit(&huart2, sos, 4, 100);
					taskENTER_CRITICAL();
					xSemaphoreGive(sem1Handle);
				}
			}	
  /* USER CODE END startSOS */
	}
}

/* USER CODE BEGIN Header_getHR */
/**
* @brief Function implementing the HR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_getHR */
void getHR(void *argument)
{
  /* USER CODE BEGIN getHR */
  /* Infinite loop */
	
  for(;;)
  {
		if (secs == 0)
		{
			xyz= read_sequential();
			fff[0] = xyz<100?0:xyz / 100 + 48;
			fff[1] = xyz<10?0:(xyz / 10) % 10 + 48;
			fff[2] = xyz % 10 + 48;
			sprintf(hrt, "Heart rate: ");
			if (xSemaphoreTake(sem1Handle, 10) == pdTRUE) 
			{
				taskENTER_CRITICAL();
				HAL_UART_Transmit(&huart2, (uint8_t *)hrt, 12, 1000);
				HAL_UART_Transmit(&huart2, fff, sizeof(fff), 1000);
				HAL_UART_Transmit(&huart2, nl, 1, 1000);
				taskEXIT_CRITICAL();
				xSemaphoreGive(sem1Handle);
			}
			secs = 30;
		}
		else secs --;
		HAL_Delay(1000);
		osDelay(1);
		}

  }
	
  /* USER CODE END getHR */


/* USER CODE BEGIN Header_clkcount */
/**
* @brief Function implementing the RTC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_clkcount */
void clkcount(void *argument)
{
  /* USER CODE BEGIN clkcount */
	if (first_time)
	{
		secbuffer[0] = 0x00; //register address
		secbuffer[1] = 0x00; //data to put in register
		HAL_I2C_Master_Transmit(&hi2c3, 0xD0, secbuffer, 2, 10);
		// minutes
		minbuffer[0] = 0x01; //register address
		minbuffer[1] = 0x40; //data to put in register
		HAL_I2C_Master_Transmit(&hi2c3, 0xD0, minbuffer, 2, 10);
		
		// hours
		hourbuffer[0] = 0x02; //register address
		hourbuffer[1] = 0x11; //data to put in register 
		HAL_I2C_Master_Transmit(&hi2c3, 0xD0, hourbuffer, 2, 10);
	 
		daybuffer[0] = 0x03;
		daybuffer[0] = 0x00;
		HAL_I2C_Master_Transmit(&hi2c3, 0xD0, daybuffer, 2, 10);

		datebuffer[0] = 0x04;
		datebuffer[0] = 0x00;
		HAL_I2C_Master_Transmit(&hi2c3, 0xD0, datebuffer, 2, 10);
		
		status_reg[0] = 0x0F;
		status_reg[1] = 0x88;
		HAL_I2C_Master_Transmit(&hi2c3, 0xD0, status_reg, 2, 10);
	// control register
		ctrl_reg[0] = 0x0E;
		ctrl_reg[1] = 0x1E; 
		HAL_I2C_Master_Transmit(&hi2c3, 0xD0, ctrl_reg, 2, 10);
	// minutes
		alarm_minbuffer[0] = 0x0B;
		alarm_minbuffer[1] = 0x41;
		//alarm_minbuffer[1] += count;
		HAL_I2C_Master_Transmit(&hi2c3, 0xD0, alarm_minbuffer, 2, 10);
	// hours
		alarm_hourbuffer[0] = 0x0C;
		alarm_hourbuffer[1] = 0x11;
		HAL_I2C_Master_Transmit(&hi2c3, 0xD0, alarm_hourbuffer, 2, 10);
		
		alarm_daybuffer[0] = 0x0D;
		alarm_daybuffer[1] = 0xC0;
		HAL_I2C_Master_Transmit(&hi2c3, 0xD0, alarm_daybuffer, 2, 10);
		
		alarm_datebuffer[0] = 0x0D;
		alarm_datebuffer[1] = 0x80;
		HAL_I2C_Master_Transmit(&hi2c3, 0xD0, alarm_datebuffer, 2, 10);
		
		first_time = 0;
}
  /* Infinite loop */
  for(;;)
  {
		HAL_I2C_Master_Transmit(&hi2c3, 0xD0, &status_reg[0], 1, 10);
		HAL_I2C_Master_Receive(&hi2c3, 0xD1, &a2f, 1, 10);
		
		if (a2f & 0x02)
		{
			alarm_minbuffer[1] += count;	
			HAL_I2C_Master_Transmit(&hi2c3, 0xD0, alarm_minbuffer, 2, 10);			
			HAL_I2C_Master_Transmit(&hi2c3, 0xD0, status_reg, 2, 10);
		}
    osDelay(1);
  }
  /* USER CODE END clkcount */
}

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
