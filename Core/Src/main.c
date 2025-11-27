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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM3
#include "arm_math.h"
#include <stdio.h>  // For sprintf
#include <string.h> // For memcpy
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define USE_DMA 1 // 1: DMA (Double Buffering), 0: Polling (Data Loss)
#define VISUALIZER_MODE                                                        \
  1 // 1: Binary Output for Python Visualizer (230400bps), 0: Text Output
    // (115200bps)

#define MAX_FFT_LEN                                                            \
  512 // Reduced from 1024 to prevent RAM overflow (2 channels * 2 buffers)
uint16_t current_fft_len = 256;        // Default to Normal Mode
uint32_t current_sampling_rate = 2048; // Default to 2kHz

#if USE_DMA
// Double Buffered, 2 Channels (CH0: Signal, CH1: Temp)
// Size = FFT_LEN * 2 (Ping-Pong) * 2 (Channels)
uint16_t adc_buffer[MAX_FFT_LEN * 4];
#else
uint16_t adc_buffer[MAX_FFT_LEN * 2]; // Single Buffer for Polling (2 Channels)
#endif

float32_t fft_input_buffer[MAX_FFT_LEN];
float32_t fft_output_buffer[MAX_FFT_LEN];
float32_t fft_magnitude[MAX_FFT_LEN / 2];
volatile uint8_t fft_data_ready = 0;
arm_rfft_fast_instance_f32 S;
FeaturePacket g_feat;                   // Global Feature Packet
uint8_t current_mode = CMD_MODE_NORMAL; // Default to Normal Mode
uint8_t spi_rx_buf[22];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void Update_Sampling_Rate(uint8_t command);
void Reinit_FFT(uint16_t new_len);
void Configure_ADC_Timer(uint32_t sampling_rate);
void Calculate_Features(float32_t *fft_mag, float32_t *time_sig,
                        uint16_t fft_len, uint32_t sampling_rate,
                        FeaturePacket *feat);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  arm_rfft_fast_init_f32(&S, MAX_FFT_LEN);
  Reinit_FFT(current_fft_len);
  Configure_ADC_Timer(current_sampling_rate);
  HAL_TIM_Base_Start(&htim3);

#if USE_DMA
  // Start ADC with DMA (Circular Mode)
  // Length = FFT_LEN * 4 (2 Channels * 2 Buffers)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, current_fft_len * 4);
#else
  HAL_ADC_Start(
      &hadc1); // Start ADC in Interrupt/Polling mode (Triggered by Timer)
#endif

  // Debug: Confirm UART is working at startup
  HAL_UART_Transmit(&huart2, (uint8_t *)"UART Initialized v4 (ADC Fixed)\r\n",
                    33, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if USE_DMA
    // =========================================================================
    // DMA Mode (Double Buffering) - No Data Loss
    // =========================================================================
    if (fft_data_ready) {
      // Critical Section: Safely read and clear the flag to prevent race
      // conditions
      __disable_irq();
      uint8_t state = fft_data_ready;
      fft_data_ready = 0;
      __enable_irq();

      // If state was 0 (unlikely but possible if interrupt happened before
      // check), skip
      if (state == 0)
        continue;

      // state 1: First half (0 to 2*FFT_LEN - 1) -> Contains FFT_LEN samples
      // for 2 channels state 2: Second half (2*FFT_LEN to 4*FFT_LEN - 1)
      uint16_t *src_buffer =
          (state == 1) ? &adc_buffer[0] : &adc_buffer[current_fft_len * 2];
#else
    // =========================================================================
    // Polling Mode - Data Loss Occurs during Processing
    // =========================================================================
    // 1. Collect Data (Blocking)
    // In this loop, CPU is busy waiting for ADC.
    // We need to collect FFT_LEN samples for 2 channels = FFT_LEN * 2
    for (int i = 0; i < current_fft_len * 2; i++) {
      // Wait for conversion (Triggered by TIM3)
      HAL_ADC_PollForConversion(&hadc1, 100);
      adc_buffer[i] = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }

    // Set src_buffer to the single buffer
    uint16_t *src_buffer = adc_buffer;

    // Fake the block to reuse processing code below
    {
#endif

      // 1. Start Timing Measurement (Toggle LD2 High)
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

      // 2. Convert ADC data (uint16_t) to float and Extract Temperature
      // DMA stores data as: [CH0, CH1, CH0, CH1, ...]
      // CH0 = Signal, CH1 = Temperature
      uint32_t temp_sum = 0;

      for (int i = 0; i < current_fft_len; i++) {
        // Even indices (0, 2, 4...) -> Signal (CH0)
        fft_input_buffer[i] = (float32_t)src_buffer[2 * i];

        // Odd indices (1, 3, 5...) -> Temperature (CH1)
        temp_sum += src_buffer[2 * i + 1];
      }

      // Calculate Average Temperature
      float32_t temp_adc_avg = (float32_t)temp_sum / (float32_t)current_fft_len;
      float32_t temp_volts = temp_adc_avg * 3.3f / 4095.0f;
      float32_t temperature =
          temp_volts * 100.0f; // Assuming LM35 (10mV/C) or similar

      // 3. Perform RFFT
      arm_rfft_fast_f32(&S, fft_input_buffer, fft_output_buffer, 0);

      // 4. Calculate Magnitude
      arm_cmplx_mag_f32(fft_output_buffer, fft_magnitude, current_fft_len / 2);

      // 5. Feature Extraction (F1~F5)
      Calculate_Features(fft_magnitude, fft_input_buffer, current_fft_len,
                         current_sampling_rate, &g_feat);

      // 6. Send Features to STM32_B via SPI2
      // Prepare SPI Packet with Header
      // Requested Structure: Header(2) + F1_peak + F1_index + F2_sideband +
      // F2_index + F3_harmonic + F4_rms + F5_thd + Temp Total = 2 + (8 * 4) =
      // 34 bytes
      uint8_t spi_packet[34];
      spi_packet[0] = 0xAA;
      spi_packet[1] = 0xBB;
      memcpy(&spi_packet[2], &g_feat, sizeof(FeaturePacket));

      // Small delay for stability (approx 10-20us)
      for (volatile int k = 0; k < 100; k++) {
        __NOP();
      }

      // Manually copy specific fields
      float *pSpiData = (float *)&spi_packet[2];
      pSpiData[0] = g_feat.F1_peak;
      pSpiData[1] = g_feat.F1_index; // Added F1_index
      pSpiData[2] = g_feat.F2_sideband;
      pSpiData[3] = g_feat.F2_index;
      pSpiData[4] = g_feat.F3_harmonic;
      pSpiData[5] = g_feat.F4_rms;
      pSpiData[6] = g_feat.F5_thd;
      pSpiData[7] = temperature;

      // Transmit Data (NSS handled by Hardware)
      HAL_SPI_Transmit(&hspi2, spi_packet, sizeof(spi_packet), 100);

#if VISUALIZER_MODE
      // =========================================================================
      // Visualizer Mode: Send Binary Data (Header + Float Array)
      // =========================================================================
      // 1. Send Header (0xFFFFFFFF) to sync
      uint32_t header = 0xFFFFFFFF;
      HAL_UART_Transmit(&huart2, (uint8_t *)&header, 4, 100);

      // 2. Send FFT Magnitude Data (FFT_LEN/2 floats)
      if (current_fft_len < MAX_FFT_LEN) {
        memset(&fft_magnitude[current_fft_len / 2], 0,
               ((MAX_FFT_LEN - current_fft_len) / 2) * sizeof(float32_t));
      }
      HAL_UART_Transmit(&huart2, (uint8_t *)fft_magnitude,
                        (current_fft_len / 2) * sizeof(float32_t), 1000);

      // 3. Send Temperature Data (1 float)
      HAL_UART_Transmit(&huart2, (uint8_t *)&temperature, sizeof(float32_t),
                        100);

      // 4. Send Feature Packet (9 floats - Exclude F1_index for Python
      // compatibility) Python expects: F1_mag, F1_freq, F2_mag, F2_idx,
      // F2_freq, F3_mag, F3_freq, F4, F5
      float uart_feat[9];
      uart_feat[0] = g_feat.F1_peak;
      uart_feat[1] = g_feat.F1_freq;
      uart_feat[2] = g_feat.F2_sideband;
      uart_feat[3] = g_feat.F2_index;
      uart_feat[4] = g_feat.F2_freq;
      uart_feat[5] = g_feat.F3_harmonic;
      uart_feat[6] = g_feat.F3_freq;
      uart_feat[7] = g_feat.F4_rms;
      uart_feat[8] = g_feat.F5_thd;

      HAL_UART_Transmit(&huart2, (uint8_t *)uart_feat, sizeof(uart_feat), 100);
#else
      // =========================================================================
      // Text Mode: Send Peak Frequency Info + Temperature + Features
      // =========================================================================
      // 5. Find Peak Frequency (UART Debug)
      float32_t max_value;
      uint32_t max_index;
      // Skip DC component (Index 0)
      arm_max_f32(&fft_magnitude[1], (current_fft_len / 2) - 1, &max_value,
                  &max_index);

      // Adjust index because we started from 1
      max_index += 1;

      // Calculate Frequency: Freq = Index * Fs / FFT_LEN
      // Fs = 4096 Hz (Calculated from TIM3 settings)
      float32_t freq = (float32_t)max_index * (float32_t)current_sampling_rate /
                       (float32_t)current_fft_len;

      char uart_buf[200];
      int len = sprintf(
          uart_buf,
          "Mode: %s, Peak: %d.%d Hz, Mag: %d, Temp: %d.%d C\r\n"
          "F1: %d, F2: %d, F3: %d, F4: %d, F5: %d\r\n",
          (current_mode == CMD_MODE_HIGH_RES) ? "HIGH" : "NORM", (int)freq,
          (int)((freq - (int)freq) * 10), (int)max_value, (int)temperature,
          (int)((temperature - (int)temperature) * 10), (int)g_feat.F1_peak,
          (int)g_feat.F2_sideband, (int)g_feat.F3_harmonic, (int)g_feat.F4_rms,
          (int)(g_feat.F5_thd *
                100)); // THD as percentage * 100 for int display
      HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, len, 1000);
#endif

      // 6. End Timing Measurement (Toggle LD2 Low)
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }
#if !USE_DMA
    // End of fake block for Polling mode
  }
#endif
}
/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 124;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 124;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400; // Lowered to 230400 for stability
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Update_Sampling_Rate(uint8_t command) {
  if (command == current_mode) {
    return; // No change
  }

  if (command == CMD_MODE_HIGH_RES) {
    // High Resolution Mode: Increase Sampling Rate
    // Target: 4096 Hz (Original Standard)
    // PSC=124, ARR=124 -> 4096 Hz
    __HAL_TIM_SET_AUTORELOAD(&htim3, 124);
    current_sampling_rate = 4096;
    current_mode = command;
  } else if (command == CMD_MODE_NORMAL) {
    // Normal Mode: Reduce Sampling Rate (Power Saving)
    // Target: 2048 Hz (Half)
    // PSC=124, ARR=249 (approx 250 for half freq)
    // 64MHz / (125 * 250) = 2048 Hz
    __HAL_TIM_SET_AUTORELOAD(&htim3, 249);
    current_sampling_rate = 2048;
    current_mode = command;
  }
  // Reset Counter to avoid glitches if CNT > new ARR
  __HAL_TIM_SET_COUNTER(&htim3, 0);
}

#if USE_DMA
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    fft_data_ready = 1; // First half ready
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    fft_data_ready = 2; // Second half ready
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
  // Toggle LD2 fast to indicate error
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}
#endif

void Calculate_Features(float32_t *fft_mag, float32_t *time_sig,
                        uint16_t fft_len, uint32_t sampling_rate,
                        FeaturePacket *feat) {
  // F1: Peak Frequency Magnitude (Global Peak)
  // Skip DC (index 0)
  float32_t max_val;
  uint32_t max_idx;
  arm_max_f32(&fft_mag[1], (fft_len / 2) - 1, &max_val, &max_idx);
  max_idx += 1; // Adjust for offset because we started search from index 1

  feat->F1_peak = max_val;
  feat->F1_index = (float32_t)max_idx; // Store Index
  feat->F1_freq =
      (float32_t)max_idx * (float32_t)sampling_rate / (float32_t)fft_len;

  // F2: Sideband Magnitude (Targeted Search using 1/7 Rule)
  // 1/7 Rule: Mechanical Frequency (fr) = Electrical Frequency (fs) / 7
  // Sidebands appear at fs +/- k * fr (where k = 1, 3, 5...)
  // We search around these candidate positions relative to F1.

  float32_t sb_max = 0.0f;
  uint32_t sb_idx = 0;

  // Calculate Machine Frequency Index (fr_idx)
  // max_idx is F1 (fs). fr_idx = fs / 7
  uint32_t fr_idx = max_idx / 7;

  // Candidate multipliers: 1 (Shaft/Unbalance), 3 (Outer Race), 5 (Inner Race)
  int multipliers[] = {1, 3, 5};

  for (int k = 0; k < 3; k++) {
    int dist_idx = fr_idx * multipliers[k]; // Distance from F1

    // Check both Left (-) and Right (+) sidebands
    int signs[] = {-1, 1};
    for (int s = 0; s < 2; s++) {
      int center_idx = (int)max_idx + (dist_idx * signs[s]);

      // Search +/- 5 index around the candidate center to catch offset peaks
      for (int offset = -5; offset <= 5; offset++) {
        int target_idx = center_idx + offset;

        // Boundary Check
        if (target_idx > 0 && target_idx < (fft_len / 2)) {
          // Skip F1 itself (just in case)
          if (target_idx == max_idx)
            continue;

          if (fft_mag[target_idx] > sb_max) {
            sb_max = fft_mag[target_idx];
            sb_idx = target_idx;
          }
        }
      }
    }
  }

  feat->F2_sideband = sb_max;
  feat->F2_index = (float32_t)sb_idx; // Store Index for SPI

  // Calculate F2 Frequency
  if (sb_max > 0.0f) {
    feat->F2_freq =
        (float32_t)sb_idx * (float32_t)sampling_rate / (float32_t)fft_len;
  } else {
    feat->F2_freq = 0.0f;
  }

  // F3: 3rd Harmonic Magnitude
  // Harmonic Index = 3 * Fundamental Index
  int h3_idx = max_idx * 3;
  if (h3_idx < (fft_len / 2)) {
    feat->F3_harmonic = fft_mag[h3_idx];
    feat->F3_freq =
        (float32_t)h3_idx * (float32_t)sampling_rate / (float32_t)fft_len;
  } else {
    feat->F3_harmonic = 0.0f;
    feat->F3_freq = 0.0f;
  }

  // F4: RMS (Time Domain)
  // sqrt( sum(x^2) / N )
  float32_t sum_sq = 0.0f;
  for (int i = 0; i < fft_len; i++) {
    sum_sq += time_sig[i] * time_sig[i];
  }
  float32_t mean_sq = sum_sq / (float32_t)fft_len;
  arm_sqrt_f32(mean_sq, &feat->F4_rms);

  // F5: THD (Total Harmonic Distortion)
  // sqrt( sum(Harmonic_Mag^2) ) / Fundamental_Mag
  // Harmonics: 2nd to 5th
  float32_t harm_energy = 0.0f;
  for (int h = 2; h <= 5; h++) {
    int h_idx = max_idx * h;
    if (h_idx < (fft_len / 2)) {
      harm_energy += fft_mag[h_idx] * fft_mag[h_idx];
    }
  }

  float32_t harm_rms;
  arm_sqrt_f32(harm_energy, &harm_rms);

  if (feat->F1_peak > 0.0001f) {
    feat->F5_thd = harm_rms / feat->F1_peak;
  } else {
    feat->F5_thd = 0.0f;
  }
}

void Configure_ADC_Timer(uint32_t sampling_rate) {
  // TIM3 Clock = PCLK1 * 2 = 8MHz * 2 = 16MHz (Assuming HSI 8MHz, PLLx2 ->
  // 16MHz... wait, SystemClock_Config says PLLMUL16 -> 64MHz?) Let's check
  // SystemClock_Config. HSI(8MHz) / 2 * 16 = 64MHz SYSCLK. APB1 = HCLK/2 =
  // 32MHz. TIM3 CLK = APB1 * 2 = 64MHz.

  // Target Frequency = sampling_rate
  // Update Event Freq = TIM_CLK / ((PSC + 1) * (ARR + 1))
  // We want Update Event Freq = sampling_rate

  // Let's fix PSC and calculate ARR.
  // 64,000,000 / (sampling_rate) = Total Counts
  // If sampling_rate = 2000 -> 32000 counts. Fits in 16-bit ARR (65535).
  // If sampling_rate = 8000 -> 8000 counts. Fits.

  // Let's use PSC = 63 (Divide by 64) -> 1MHz Timer Clock
  // Then ARR = (1,000,000 / sampling_rate) - 1

  uint32_t psc = 63;
  uint32_t arr = (1000000 / sampling_rate) - 1;

  __HAL_TIM_SET_PRESCALER(&htim3, psc);
  __HAL_TIM_SET_AUTORELOAD(&htim3, arr);

  // Reset Counter to avoid immediate trigger if current count > new ARR
  __HAL_TIM_SET_COUNTER(&htim3, 0);
}

void Reinit_FFT(uint16_t new_len) {
  // Re-initialize FFT structure
  arm_rfft_fast_init_f32(&S, new_len);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
