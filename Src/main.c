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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLES 1000
#define WINDOW_SIZE 2
#define PC_RX_BUFFER 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t spi_rx_buffer[SAMPLES];
volatile int spi_rx_first_half_ready = 0;
volatile int spi_rx_second_half_ready = 0;


//Moving Average Stuff
volatile uint16_t window[WINDOW_SIZE];
volatile uint16_t processing_buffer_1[SAMPLES/2];
volatile uint16_t processed_buffer_1[SAMPLES/2];
volatile uint16_t processing_buffer_2[SAMPLES/2];
volatile uint16_t processed_buffer_2[SAMPLES/2];
volatile uint32_t sum = 0;
volatile int window_index = 0;

// Buffers for downsampled data
volatile uint8_t downsampled_buffer_1[SAMPLES/2];
volatile uint8_t downsampled_buffer_2[SAMPLES/2];

// Ultrasonic
uint16_t count_1 = 0;
uint16_t count_2 = 0;
float min_distance = 10; // unit: cm
float distance = 0; // unit: cm
int flag = 0;
int spi_tx_flag = 0;

uint8_t UART2_rxBuffer[PC_RX_BUFFER] = {0};
volatile int ultrasonic_mode = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void delay_uS(uint16_t delay);
void HCSR04_Read();
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  // Initialize SPI with DMA

  //Timer to count to 10ms
  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim7);

  HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)spi_rx_buffer, SAMPLES);
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, UART2_rxBuffer, PC_RX_BUFFER);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (spi_rx_first_half_ready == 1) {
      // First, reset the flag

      spi_rx_first_half_ready = 0;
      sum = 0;
      window_index = 0;
      // Copy the data first
      memcpy(processing_buffer_1, spi_rx_buffer, (SAMPLES / 2) * sizeof(uint16_t));

      // Processing steps should be done here
      // Initial window fill
      // Reference for moving average


      for (int i = 0; i < WINDOW_SIZE; i++) {
        window[i] = processing_buffer_1[i];
        sum += window[i];
        processed_buffer_1[i] = sum / (i + 1);
      }

      // Process remaining samples
      for (int i = WINDOW_SIZE; i < SAMPLES/2; i++) {
        // Get current sample
        uint16_t current_sample = processing_buffer_1[i];

        // Calculate mean of current window
        uint16_t window_mean = sum / WINDOW_SIZE;

        // Check if current sample is an outlier (using threshold)
        uint16_t threshold = (window_mean * 85) / 100; // Adjust as needed

        if (abs(current_sample - window_mean) > threshold) {
          // Replace outlier with the current window mean
          current_sample = window_mean;
        }

        sum -= window[window_index];
        window[window_index] = current_sample;
        sum += window[window_index];
        processed_buffer_1[i] = sum / WINDOW_SIZE;
        window_index = (window_index + 1 == WINDOW_SIZE) ? 0 : window_index + 1;
      }

      // DOWNSAMPLING
      for (int i = 0; i < SAMPLES/2; i++) {
        // Shift the data by 4 bits to the right to conserve MSB and get rid of the LSB
        downsampled_buffer_1[i] = (uint8_t)(processed_buffer_1[i] >> 4);
      }



      // Transmit the processed data via UART2
      if (ultrasonic_mode == 0 || (ultrasonic_mode == 1 && spi_tx_flag == 1)) {
        HAL_UART_Transmit_IT(&huart2, (uint8_t*)&downsampled_buffer_1, (SAMPLES / 2));
      }
      // Consider if you need to wait for transmission to complete before the next SPI reception
    }

    if (spi_rx_second_half_ready == 1) {
      // First, reset the flag
      spi_rx_second_half_ready = 0;
      sum = 0;
      window_index = 0;

      // Copy over the second half of the data
      memcpy(processing_buffer_2, &spi_rx_buffer[SAMPLES / 2], (SAMPLES / 2) * sizeof(uint16_t));

      // Processing steps should be done here
      // Initial window fill
      for (int i = 0; i < WINDOW_SIZE; i++) {
        window[i] = processing_buffer_2[i];
        sum += window[i];
        processed_buffer_2[i] = sum / (i + 1);
      }
      // Process remaining samples
      for (int i = WINDOW_SIZE; i < SAMPLES/2; i++) {
        // Get current sample
        uint16_t current_sample = processing_buffer_2[i];

        // Calculate mean of current window
        uint16_t window_mean = sum / WINDOW_SIZE;

        // Check if current sample is an outlier (using threshold)
        uint16_t threshold = (window_mean * 85) / 100; // Adjust as needed

        if (abs(current_sample - window_mean) > threshold) {
          // Replace outlier with the current window mean
          current_sample = window_mean;
        }

        sum -= window[window_index];
        window[window_index] = current_sample;
        sum += window[window_index];
        processed_buffer_2[i] = sum / WINDOW_SIZE;
        window_index = (window_index + 1 == WINDOW_SIZE) ? 0 : window_index + 1;
      }


      // DOWNSAMPLING
      for (int i = 0; i < SAMPLES/2; i++) {
        // Shift the data by 4 bits to the right to conserve MSB and get rid of the LSB
        downsampled_buffer_2[i] = (uint8_t)(processed_buffer_2[i] >> 4);
      }


      //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
      // Transmit the processed data via UART2-++
      if (ultrasonic_mode == 0 || (ultrasonic_mode == 1 && spi_tx_flag == 1)) {
        HAL_UART_Transmit_IT(&huart2, (uint8_t*)&downsampled_buffer_2, (SAMPLES / 2));
      }


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

/* USER CODE BEGIN 4 */

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  // First half of rx buffer filled, flag will be set to 1 so data can be processed further.
  // Data processing is not done here to keep ISR as short as possible
  spi_rx_first_half_ready = 1;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  // Second half of rx buffer filled, flag will be set to 1 so data can be processed further.
  // Data processing is not done here to keep ISR as short as possible
  spi_rx_second_half_ready = 1;
}


// ULTRASONIC

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim == &htim7) {
    HCSR04_Read();
    if (distance < min_distance) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
      spi_tx_flag = 1;
    } else {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
      spi_tx_flag = 0;
    }
  }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

  if ((htim == &htim1) && (htim->Channel == 1)) {
    if (flag == 0) {
      count_1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
      flag = 1;
    }
    else {
      count_2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
      __HAL_TIM_SET_COUNTER(htim, 0);
      distance = (count_2 - count_1) / 58.0;
      flag = 0;
    }
  }
}

void delay_uS(uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim6, 0);
  while(__HAL_TIM_GET_COUNTER(&htim6) < delay) {

  }
}

void HCSR04_Read() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
  delay_uS(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2) {
    if (Size > 0) {
      if (UART2_rxBuffer[0] == '1' && UART2_rxBuffer[1] == 'U' && UART2_rxBuffer[2] == ' ') {
        int value = 0;
        for (int i = 3; i < Size && UART2_rxBuffer[i] >= '0' && UART2_rxBuffer[i] <= '9'; i++) {
          value = value * 10 + (UART2_rxBuffer[i] - '0'); // This is just an easy way to convert the ascii into an actual integer
        }
        if (value > 0) {
          min_distance = (float)value;
          ultrasonic_mode = 1;
        }
      }
      if (UART2_rxBuffer[0] == '2' && UART2_rxBuffer[1] == 'U') {
        ultrasonic_mode = 0;
      }
    }
    HAL_UARTEx_ReceiveToIdle_IT(huart, UART2_rxBuffer, PC_RX_BUFFER);
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
