/* USER CODE BEGIN Header */
/** ADC 보내는 기능 추가
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
#include <stdio.h> // For sprintf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float F1_peak;
    float F2_sideband;
    float F3_harmonic;
    float F4_rms;
    float F5_thd;
} FeaturePacket;
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

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
SPI_HandleTypeDef hspi1;   

/* USER CODE BEGIN PV */
#define USE_DMA 1 // 1: DMA (Double Buffering), 0: Polling (Data Loss)
#define VISUALIZER_MODE 1 // 1: Binary Output for Python Visualizer (460800bps), 0: Text Output (115200bps)
#define FFT_LEN  512 // Reduced from 1024 to prevent RAM overflow (2 channels * 2 buffers)
volatile uint8_t g_cmd_from_B = 0;  // B가 보내준 상태값 저장 변수
uint8_t spi_rx_buf[1];              // 1바이트 수신 버퍼

#if USE_DMA
// Double Buffered, 2 Channels (CH0: Signal, CH1: Temp)
// Size = FFT_LEN * 2 (Ping-Pong) * 2 (Channels)
uint16_t adc_buffer[FFT_LEN * 4]; 
#else
uint16_t adc_buffer[FFT_LEN * 2]; // Single Buffer for Polling (2 Channels)
#endif

float32_t fft_input_buffer[FFT_LEN];
float32_t fft_output_buffer[FFT_LEN];
float32_t fft_magnitude[FFT_LEN / 2];
volatile uint8_t fft_data_ready = 0;
arm_rfft_fast_instance_f32 S;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);   

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();   

  /* USER CODE BEGIN 2 */
  arm_rfft_fast_init_f32(&S, FFT_LEN);
  HAL_TIM_Base_Start(&htim3);
  HAL_SPI_Receive_IT(&hspi1, spi_rx_buf, 1);

#if USE_DMA
  // Start ADC with DMA (Circular Mode)
  // Length = FFT_LEN * 4 (2 Channels * 2 Buffers)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, FFT_LEN * 4);
#else
  HAL_ADC_Start(&hadc1); // Start ADC in Interrupt/Polling mode (Triggered by Timer)
#endif

  // Debug: Confirm UART is working at startup
  HAL_UART_Transmit(&huart2, (uint8_t*)"UART Initialized v4 (ADC Fixed)\r\n", 33, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if USE_DMA
    // =========================================================================
    // DMA Mode (Double Buffering) - No Data Loss
    // =========================================================================
    if (fft_data_ready) {
      // Critical Section: Safely read and clear the flag to prevent race conditions
      __disable_irq();
      uint8_t state = fft_data_ready;
      fft_data_ready = 0;
      __enable_irq();

      // If state was 0 (unlikely but possible if interrupt happened before check), skip
      if (state == 0) continue;

      // state 1: First half (0 to 2*FFT_LEN - 1) -> Contains FFT_LEN samples for 2 channels
      // state 2: Second half (2*FFT_LEN to 4*FFT_LEN - 1)
      uint16_t* src_buffer = (state == 1) ? &adc_buffer[0] : &adc_buffer[FFT_LEN * 2];
#else
    // =========================================================================
    // Polling Mode - Data Loss Occurs during Processing
    // =========================================================================
    // 1. Collect Data (Blocking)
    // In this loop, CPU is busy waiting for ADC.
    // We need to collect FFT_LEN samples for 2 channels = FFT_LEN * 2
    for (int i = 0; i < FFT_LEN * 2; i++) {
        // Wait for conversion (Triggered by TIM3)
        HAL_ADC_PollForConversion(&hadc1, 100); 
        adc_buffer[i] = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    
    // Set src_buffer to the single buffer
    uint16_t* src_buffer = adc_buffer;
    
    // Fake the block to reuse processing code below
    {
#endif

      // 1. Start Timing Measurement (Toggle LD2 High)
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

      // 2. Convert ADC data (uint16_t) to float and Extract Temperature
      // DMA stores data as: [CH0, CH1, CH0, CH1, ...]
      // CH0 = Signal, CH1 = Temperature
      uint32_t temp_sum = 0;
      
      for (int i = 0; i < FFT_LEN; i++) {
        // Even indices (0, 2, 4...) -> Signal (CH0)
        fft_input_buffer[i] = (float32_t)src_buffer[2 * i];
        
        // Odd indices (1, 3, 5...) -> Temperature (CH1)
        temp_sum += src_buffer[2 * i + 1];
      }
      
      // Calculate Average Temperature
      float32_t temp_adc_avg = (float32_t)temp_sum / (float32_t)FFT_LEN;
      float32_t temp_volts = temp_adc_avg * 3.3f / 4095.0f;
      float32_t temperature = temp_volts * 100.0f; // Assuming LM35 (10mV/C) or similar
      // ========================================
      // B에서 받은 상태값(g_cmd_from_B)에 따라 A 동작 변경
      // ========================================

      if (g_cmd_from_B == 1) {
          // ★ 고해상도 / 더 정교한 분석 모드 (원하면 기능 넣기)
      }
      else if (g_cmd_from_B == 2) {
          // ★ 부하 변화 모드 처리
      }
      else if (g_cmd_from_B == 3) {
          // ★ FAULT 모드 → FFT 수행하지 않고 스킵
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
          continue;
      }
      // 0 또는 기본 상태라면 NORMAL 모드로 그대로 FFT 수행

      
      // 3. Perform RFFT
      arm_rfft_fast_f32(&S, fft_input_buffer, fft_output_buffer, 0);
      
      
      // 4. Calculate Magnitude
      arm_cmplx_mag_f32(fft_output_buffer, fft_magnitude, FFT_LEN / 2);
      
#if VISUALIZER_MODE
      // =========================================================================
      // Visualizer Mode: Send Binary Data (Header + Float Array)
      // =========================================================================
      // 1. Send Header (0xFFFFFFFF) to sync
      uint32_t header = 0xFFFFFFFF;
      HAL_UART_Transmit(&huart2, (uint8_t*)&header, 4, 10);

      // 2. Send FFT Magnitude Data (FFT_LEN/2 floats)
      HAL_UART_Transmit(&huart2, (uint8_t*)fft_magnitude, (FFT_LEN / 2) * sizeof(float32_t), 100);

      // 3. Send Temperature Data (1 float)
      HAL_UART_Transmit(&huart2, (uint8_t*)&temperature, sizeof(float32_t), 10);
      // ======================================================
      // 추가: Feature 계산 + SPI 전송
      // ======================================================
      FeaturePacket pkt;
      compute_features(&pkt);

      // SPI로 20바이트 패킷 전송
      HAL_SPI_Transmit(&hspi1, (uint8_t*)&pkt, sizeof(pkt), 10);

#else
      // =========================================================================
      // Text Mode: Send Peak Frequency Info + Temperature
      // =========================================================================
      // 5. Find Peak Frequency (UART Debug)
      float32_t max_value;
      uint32_t max_index;
      // Skip DC component (Index 0)
      arm_max_f32(&fft_magnitude[1], (FFT_LEN / 2) - 1, &max_value, &max_index);
      
      // Adjust index because we started from 1
      max_index += 1;
      
      // Calculate Frequency: Freq = Index * Fs / FFT_LEN
      // Fs = 4096 Hz (Calculated from TIM3 settings)
      float32_t freq = (float32_t)max_index * 4096.0f / (float32_t)FFT_LEN;
      
      char uart_buf[100];
      int len = sprintf(uart_buf, "Peak: %d.%d Hz, Mag: %d, Temp: %d.%d C\r\n", 
                        (int)freq, (int)((freq - (int)freq) * 10), 
                        (int)max_value, 
                        (int)temperature, (int)((temperature - (int)temperature) * 10));
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, 100);
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
void SystemClock_Config(void)
{
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE; // Enable Scan for multiple channels
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE; // Disable Discontinuous to convert all channels on one trigger
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2; // 2 Channels
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
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
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

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
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;

  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
      Error_Handler();
  }
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
#if VISUALIZER_MODE
  huart2.Init.BaudRate = 460800;
#else
  huart2.Init.BaudRate = 115200;
#endif
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

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
static void MX_GPIO_Init(void)
{
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
// =========================
// B → A SPI 수신 콜백 추가
// =========================
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        g_cmd_from_B = spi_rx_buf[0];  // 상태값 저장

        // 다음 수신 준비
        HAL_SPI_Receive_IT(&hspi1, spi_rx_buf, 1);
    }
}
// 주파수를 FFT bin으로 변환
static inline uint32_t freq_to_bin(float freq_hz) {
    float Fs = 4096.0f;
    float df = Fs / FFT_LEN;
    int k = (int)((freq_hz / df) + 0.5f);
    if (k < 0) k = 0;
    if (k >= FFT_LEN/2) k = FFT_LEN/2 - 1;
    return (uint32_t)k;
}

// Feature 5개 계산 함수
void compute_features(FeaturePacket* out)
{
    float f1_target = 60.0f;

    // F1: Fundamental Peak
    uint32_t k1 = freq_to_bin(f1_target);
    float F1 = 0.0f;
    for (int dk = -1; dk <= 1; dk++) {
        int k = (int)k1 + dk;
        if (k > 0 && k < FFT_LEN/2) {
            if (fft_magnitude[k] > F1) F1 = fft_magnitude[k];
        }
    }

    // F2: Sideband Energy
    float band_width = 40.0f;
    uint32_t k_low  = freq_to_bin(f1_target - band_width);
    uint32_t k_high = freq_to_bin(f1_target + band_width);
    float side = 0.0f;

    for (uint32_t k = k_low; k <= k_high; k++) {
        if (k >= k1 - 2 && k <= k1 + 2) continue;
        float m = fft_magnitude[k];
        side += m * m;
    }

    // F3: 2nd Harmonic
    float f2 = 2.0f * f1_target;
    uint32_t k2 = freq_to_bin(f2);
    float F3 = 0.0f;
    for (int dk = -1; dk <= 1; dk++) {
        int k = (int)k2 + dk;
        if (k > 0 && k < FFT_LEN/2) {
            if (fft_magnitude[k] > F3) F3 = fft_magnitude[k];
        }
    }

    // F4: RMS (Time domain)
    float sum_sq = 0.0f;
    for (int i = 0; i < FFT_LEN; i++) {
        float x = fft_input_buffer[i];
        sum_sq += x * x;
    }
    float F4 = sqrtf(sum_sq / FFT_LEN);

    // F5: THD
    float harm_energy = 0.0f;
    for (int n = 2; n <= 5; n++) {
        float fn = n * f1_target;
        uint32_t kn = freq_to_bin(fn);

        float peak = 0.0f;
        for (int dk = -1; dk <= 1; dk++) {
            int k = (int)kn + dk;
            if (k > 0 && k < FFT_LEN/2) {
                if (fft_magnitude[k] > peak) peak = fft_magnitude[k];
            }
        }
        harm_energy += peak * peak;
    }

    float F5 = 0.0f;
    if (F1 > 1e-6f) {
        F5 = sqrtf(harm_energy) / F1;
    }

    // 최종 출력 구조체에 저장
    out->F1_peak      = F1;
    out->F2_sideband  = side;
    out->F3_harmonic  = F3;
    out->F4_rms       = F4;
    out->F5_thd       = F5;
}

#if USE_DMA
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        fft_data_ready = 1; // First half ready
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        fft_data_ready = 2; // Second half ready
    }
}
#endif
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
