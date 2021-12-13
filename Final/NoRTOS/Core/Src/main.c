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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "math.h"

int xyz  = 0;

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t fff[] = {'0','0','0'}; //array for heartrate
uint8_t a2f;
uint8_t status_reg[2], ctrl_reg[2];

uint8_t nl[] = {'\r', '\n', '\r'};
char hrt[12];

void displayuart()
{
	HAL_UART_Transmit(&huart2, (uint8_t *)hrt, 12, 1000);
	HAL_UART_Transmit(&huart2, fff, sizeof(fff), 1000);
	HAL_UART_Transmit(&huart2, nl, 3, 1000);
}



int heart_rate_array[4];
int previous_reading = 0;
int heart_rate = 0;
int ir[15] = {0};
uint8_t reg_data;
uint8_t data;
uint32_t read_fifo_red, read_fifo_ir;
uint8_t dat[6] = {0,0,0,0,0,0};
uint32_t read_seq_red[100];
int read_seq_ir[100];
uint8_t n=100;

int findsamples(int h[], int size, int t, int max)
{
    int i = 0;
    int peaks = 0;
    while (i < size - 1)
    {
        if (h[i] > t && h[i] > h[i - 1])
        {
            int width = 1;
            while (i + width < size - 1 && h[i] == h[i + width]) 
                width++;
            if (h[i] > h[i + width] && peaks < max) 
            {
                ir[peaks] = i;
                int k = peaks - 1;
                while (k >= 0 && i > ir[k]) 
                {
                    int temp = ir[k];
                    ir[k] = i;						
                    ir[k + 1] = temp;
                    k--;
                }
                peaks++; 
                i += width + 1; 
            }
            else
                i += width;		
        }
        else
            i++;
    }
    return peaks;
}
int remove_fluct(int peaks, int rd[], int min)
{
    int i = -1;
    while (i < peaks)
    {
        int old_peaks = peaks;    	
        peaks = i + 1;
        int j = i + 1;
        while (j < old_peaks) 
        {
            int n_dist = i != -1 ? (ir[j] - ir[i]) : (ir[j] + 1);
            if (n_dist > min || n_dist < -1 * min)
            {
                ir[peaks] = ir[j];
                peaks++;										
            }
            j++;
        }
        i++;
    }
    for (int i = 0; i < peaks - 1; i++)			
        for (int j = i + 1; j < peaks; j++)
            if (ir[i] > ir[j])
            {
                int temp = ir[i];				
                ir[i] = ir[j]; 
                ir[j] = temp;						
            }
		return peaks;
}
int calculate(int rd[])
{
    int sum = 0;
    for (int i = 0; i < n; i++) 
        sum += rd[i];
    int mean = sum / n;
    for (int i = 0; i < n; i++) 
        rd[i] = (rd[i] - mean) * -1;
    for (int i = 0; i < n-4; i++)
        rd[i] = (rd[i] + rd[i + 1] + rd[i + 2] + rd[i + 3]) / 4; 
    sum = 0;
    for (int i = 0; i < n; i++)
        sum += rd[i]; 
    mean = sum / n;
    int t = mean < 30 ? 30 : mean > 60 ? 60 : mean;
    int peaks = findsamples(rd, 100, t, 15); 
    int peaks2 = remove_fluct(peaks, rd, 4); 
    peaks = peaks < peaks2 ? peaks : peaks2; 
    int peaks_interval = 0;
    if (peaks >= 2) 
    {
        for (int i = 1; i < peaks; i++)
            peaks_interval += (ir[i] - ir[i - 1]); 
            peaks_interval = peaks_interval / (peaks - 1);
            heart_rate = 25 * 60 / peaks_interval; 
			previous_reading = heart_rate>30 && heart_rate<170?heart_rate:previous_reading; 
			heart_rate = heart_rate>30 && heart_rate<170?heart_rate:-999;
    }
    else
        heart_rate = -999;
    return heart_rate;
}
void init(){

	    if(HAL_I2C_IsDeviceReady(&hi2c1, 0xAE, 1, 100)!=HAL_OK){HAL_Delay(1000);}
				
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x09, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);	

        HAL_Delay(10);
        
        HAL_I2C_Mem_Read(&hi2c1, 0xAF, 0x00, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
        data = 0xc0;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x02, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x03, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x04, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x05, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x06, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x4f;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x08, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x03;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x09, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x27;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x0A, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x24;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x0C, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x24;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x0D, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x7f;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x10, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
}

void read_fifo(){
    uint8_t reg_INTR1, reg_INTR2;
    HAL_I2C_Mem_Read(&hi2c1, 0xAE, 0x00, I2C_MEMADD_SIZE_8BIT, &reg_INTR1, 1, 10);
    HAL_I2C_Mem_Read(&hi2c1, 0xAE, 0x01, I2C_MEMADD_SIZE_8BIT, &reg_INTR2, 1, 10);
    HAL_I2C_Mem_Read(&hi2c1, 0xAE, 0x07, I2C_MEMADD_SIZE_8BIT, dat, 6 , 25);
    read_fifo_red = (dat[0] << 16 | dat[1] << 8 | dat[2]) & 0x03FFFF;
    read_fifo_ir = (dat[3] << 16 | dat[4] << 8 | dat[5]) & 0x03FFFF;
}

int read_sequential(){
    int i;
		int c999 = 0;
		for(int ii = 0; ii<4; ii++){
    for(i=0;i<n;i++){
        while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1){} 
        read_fifo();
        read_seq_red[i] = read_fifo_red; 
        read_seq_ir[i] = read_fifo_ir;
    }
		heart_rate_array[ii] = calculate(read_seq_ir);
		if(heart_rate_array[ii] == -999)
			c999++;
	}
		return (heart_rate_array[0]+heart_rate_array[1]+heart_rate_array[2]+heart_rate_array[3]+(999*c999))/(4-c999); 
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  init();
  uint8_t secbuffer[2], minbuffer[2], hourbuffer[2], daybuffer[2], datebuffer[2];
  uint8_t h1,h2,m1,m2,s1,s2;
  char uartBuf [100] = {0};
  uint8_t alarm_minbuffer[2], alarm_hourbuffer[2], alarm_daybuffer[2], alarm_datebuffer[2];
  int count = 1;
  // seconds
  secbuffer[0] = 0x00; //register address
  secbuffer[1] = 0x00; //data to put in register --> 0 sec
  HAL_I2C_Master_Transmit(&hi2c3, 0xD0, secbuffer, 2, 10);
  // minutes
  minbuffer[0] = 0x01; //register address
  minbuffer[1] = 0x40; //data to put in register --> 15 min
  HAL_I2C_Master_Transmit(&hi2c3, 0xD0, minbuffer, 2, 10);
	
  // hours
  hourbuffer[0] = 0x02; //register address
  hourbuffer[1] = 0x11; //data to put in register 01001001 --> 7 am
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

	int flag =0;
	
	uint8_t sos[] = {'S', 'O', 'S', '\n', '\r'};
	//uint8_t no[] = {'N', 'O', '\n', '\r'};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  int count =1;
	  //int count2 =0; 	
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

		if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0) )
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
				//HAL_UART_Transmit(&huart2, no, 5, 100);
			}
		else 
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
				HAL_UART_Transmit(&huart2, sos, 4, 100);
			}	
    if (flag ==0)
    {
      HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, sizeof(uartBuf), 10);
      flag =1;
    }
		
    HAL_I2C_Master_Transmit(&hi2c3, 0xD0, secbuffer, 1, 10);
    //read data of register 00h to secbuffer[1]
    HAL_I2C_Master_Receive(&hi2c3, 0xD1, secbuffer+1, 1, 10);
    //prepare UART output
    s1 = secbuffer[1] >> 4;
    s2 = secbuffer[1] & 0x0F;
    HAL_I2C_Master_Transmit(&hi2c3, 0xD0, minbuffer, 1, 10);
    HAL_I2C_Master_Receive(&hi2c3, 0xD1, minbuffer+1, 1, 10);
    m1 = minbuffer[1] >> 4;
    m2 = minbuffer[1] & 0x0F;
    HAL_I2C_Master_Transmit(&hi2c3, 0xD0, hourbuffer, 1, 10);
    HAL_I2C_Master_Receive(&hi2c3, 0xD1, hourbuffer+1, 1, 10);
    h1 = (hourbuffer[1] >> 4) & 1;
    h2 = hourbuffer[1] & 0x0F;
    
    // transmit time to UART
    //sprintf(uartBuf, "%d%d:%d%d:%d%d\r\n",h1,h2,m1,m2,s1,s2);
    //HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, sizeof(uartBuf), 10);
  //	HAL_UART_Transmit(&huart2, (uint8_t *)hrt, 12, 1000);
  //	HAL_UART_Transmit(&huart2, fff, sizeof(fff), 1000);
  //	HAL_UART_Transmit(&huart2, nl, 3, 1000);

      HAL_I2C_Master_Transmit(&hi2c3, 0xD0, &status_reg[0], 1, 10);
      HAL_I2C_Master_Receive(&hi2c3, 0xD1, &a2f, 1, 10);
      
      if (a2f & 0x02)
      {
        char myuartBuf [100] = {0};
				char nl [1]= {'\n'};
        alarm_minbuffer[1] += count;	
        HAL_I2C_Master_Transmit(&hi2c3, 0xD0, alarm_minbuffer, 2, 10);
        sprintf(myuartBuf, "MR:%d%d:%d%d:%d%d \r \n",h1,h2,m1,m2,s1,s2);
        HAL_UART_Transmit(&huart2, (uint8_t *)myuartBuf, sizeof(myuartBuf), 10);
				HAL_UART_Transmit(&huart2, (uint8_t *)nl, 1, 10);
        
        HAL_I2C_Master_Transmit(&hi2c3, 0xD0, status_reg, 2, 10);
    
        xyz = read_sequential();
        fff[0] = xyz<100?0:xyz / 100 + 48;
        fff[1] = xyz<10?0:(xyz / 10) % 10 + 48;
        fff[2] = xyz % 10 + 48;
        sprintf(hrt, "Heart rate: ");
        displayuart();

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
  huart2.Init.BaudRate = 9600;
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
