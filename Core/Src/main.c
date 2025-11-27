/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Register Level)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM3
#include "arm_math.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_LEN  512
#define ADC_BUF_LEN (FFT_LEN * 4) // 2 Channels * 2 Buffers (Ping-Pong)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// ADC Buffer (DMA Circular)
uint16_t adc_buffer[ADC_BUF_LEN];

// FFT Buffers
float32_t fft_input_buffer[FFT_LEN];
float32_t fft_output_buffer[FFT_LEN];
float32_t fft_magnitude[FFT_LEN / 2];

volatile uint8_t fft_data_ready = 0;
arm_rfft_fast_instance_f32 S;
FeaturePacket g_feat;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
void Calculate_Features(float32_t* fft_mag, float32_t* time_sig, FeaturePacket* feat);

// Helper Functions
void UART_Send(uint8_t *data, uint16_t len);
void SPI_Send(uint8_t *data, uint16_t len);

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
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  // HAL_Init(); // Removed HAL_Init

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
  arm_rfft_fast_init_f32(&S, FFT_LEN);

  // Start TIM3 to trigger ADC
  TIM3->CR1 |= TIM_CR1_CEN;

  // Debug: Confirm UART is working
  char *msg = "UART Initialized (Register Mode)\r\n";
  UART_Send((uint8_t*)msg, strlen(msg));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (fft_data_ready) {
      // Critical Section
      __disable_irq();
      uint8_t state = fft_data_ready;
      fft_data_ready = 0;
      __enable_irq();

      if (state == 0) continue;

      // Determine buffer pointer
      // state 1: First half (0 to 2*FFT_LEN - 1)
      // state 2: Second half (2*FFT_LEN to 4*FFT_LEN - 1)
      uint16_t* src_buffer = (state == 1) ? &adc_buffer[0] : &adc_buffer[FFT_LEN * 2];

      // 1. Start Timing Measurement (Toggle LD2 High)
      GPIOA->BSRR = GPIO_BSRR_BS5; // Set PA5

      // 2. Convert ADC data and Extract Temperature
      uint32_t temp_sum = 0;
      
      for (int i = 0; i < FFT_LEN; i++) {
        // Even indices (0, 2, 4...) -> Signal (CH0)
        fft_input_buffer[i] = (float32_t)src_buffer[2 * i];
        
        // Odd indices (1, 3, 5...) -> Temperature (CH1)
        temp_sum += src_buffer[2 * i + 1];
      }
      
      float32_t temp_adc_avg = (float32_t)temp_sum / (float32_t)FFT_LEN;
      float32_t temp_volts = temp_adc_avg * 3.3f / 4095.0f;
      float32_t temperature = temp_volts * 100.0f;

      // 3. Perform RFFT
      arm_rfft_fast_f32(&S, fft_input_buffer, fft_output_buffer, 0);

      // 4. Calculate Magnitude
      arm_cmplx_mag_f32(fft_output_buffer, fft_magnitude, FFT_LEN / 2);
      
      // 5. Feature Extraction
      Calculate_Features(fft_magnitude, fft_input_buffer, &g_feat);

      // 6. Send Features to STM32_B via SPI2
      uint8_t spi_packet[30];
      spi_packet[0] = 0xAA;
      spi_packet[1] = 0xBB;
      
      float* pSpiData = (float*)&spi_packet[2];
      pSpiData[0] = g_feat.F1_peak;
      pSpiData[1] = g_feat.F2_sideband;
      pSpiData[2] = g_feat.F2_index;
      pSpiData[3] = g_feat.F3_harmonic;
      pSpiData[4] = g_feat.F4_rms;
      pSpiData[5] = g_feat.F5_thd;
      pSpiData[6] = temperature;

      SPI_Send(spi_packet, sizeof(spi_packet));
      
      // 7. Send Data to Visualizer via UART
      // Header (0xABCD1234)
      uint32_t header = 0xABCD1234;
      UART_Send((uint8_t*)&header, 4);

      // FFT Magnitude
      UART_Send((uint8_t*)fft_magnitude, (FFT_LEN / 2) * sizeof(float32_t));

      // Temperature
      UART_Send((uint8_t*)&temperature, sizeof(float32_t));

      // Feature Packet
      UART_Send((uint8_t*)&g_feat, sizeof(FeaturePacket));

      // Footer (0x5555AAAA)
      uint32_t footer = 0x5555AAAA;
      UART_Send((uint8_t*)&footer, 4);

      // 8. End Timing Measurement (Toggle LD2 Low)
      GPIOA->BSRR = GPIO_BSRR_BR5; // Reset PA5
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  // 1. Enable HSI
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY));

  // 2. Configure PLL
  // PLL Source = HSI / 2
  // PLL Mul = 16 -> 8MHz / 2 * 16 = 64MHz
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
  RCC->CFGR |= (RCC_CFGR_PLLMULL16); // HSI/2 is default source

  // 3. Enable PLL
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY));

  // 4. Flash Latency (2 Wait States for 64MHz)
  FLASH->ACR |= FLASH_ACR_LATENCY_2;

  // 5. Configure Bus Prescalers
  // Clear Prescaler Bits first
  RCC->CFGR &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_ADCPRE);
  // AHB = 1 (64MHz)
  // APB1 = 2 (32MHz) -> Max 36MHz
  // APB2 = 1 (64MHz)
  // ADC = 8 (8MHz) -> Max 14MHz
  RCC->CFGR |= (RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_ADCPRE_DIV8);

  // 6. Switch System Clock to PLL
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  // Enable ADC1 Clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  // Reset ADC1
  RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;
  RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;

  // 1. Set Prescaler (Already done in SystemClock_Config: /8 -> 8MHz)

  // 2. Configure ADC1 CR1
  // Scan Mode Enable
  ADC1->CR1 |= ADC_CR1_SCAN;

  // 3. Configure ADC1 CR2
  // DMA Enable, External Trigger Enable (TIM3 TRGO), Align Right
  ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_EXTTRIG;
  
  // Select External Trigger: TIM3 TRGO (0100) -> Bit 19:17 = 100
  ADC1->CR2 &= ~ADC_CR2_EXTSEL;
  ADC1->CR2 |= ADC_CR2_EXTSEL_2; 

  // 4. Configure Sample Time
  // Channel 0: 71.5 Cycles (SMP0 = 110)
  // Channel 1: 71.5 Cycles (SMP1 = 110)
  ADC1->SMPR2 &= ~(ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1);
  ADC1->SMPR2 |= (ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP0_1); // 110
  ADC1->SMPR2 |= (ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_1); // 110

  // 5. Configure Sequence
  // Length = 2 conversions (L = 1 -> 0001)
  ADC1->SQR1 &= ~ADC_SQR1_L;
  ADC1->SQR1 |= (1 << 20); // L=1 means 2 conversions

  // SQ1 = Channel 0
  ADC1->SQR3 &= ~ADC_SQR3_SQ1;
  ADC1->SQR3 |= (0 << 0);

  // SQ2 = Channel 1
  ADC1->SQR3 &= ~ADC_SQR3_SQ2;
  ADC1->SQR3 |= (1 << 5);

  // 6. Enable ADC
  ADC1->CR2 |= ADC_CR2_ADON;
  
  // Wait for ADC stabilization (approx 1us)
  // At 64MHz, 1us = 64 cycles. A simple loop is sufficient.
  for(volatile int i = 0; i < 100; i++); 
  
  // Calibration
  // Reset Calibration
  ADC1->CR2 |= ADC_CR2_RSTCAL;
  while (ADC1->CR2 & ADC_CR2_RSTCAL);
  
  // Start Calibration
  ADC1->CR2 |= ADC_CR2_CAL;
  while (ADC1->CR2 & ADC_CR2_CAL);
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  // Enable SPI2 Clock
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

  // Configure SPI2
  // Master Mode, Baud Rate /2 (000), CPOL Low, CPHA 1 Edge, 8-bit, MSB First
  // SSM=1, SSI=1 (Software Slave Management, Internal Slave Select High)
  SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;

  // Enable SPI2
  SPI2->CR1 |= SPI_CR1_SPE;
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  // Enable TIM3 Clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  // Configure Timer
  // PSC = 124, ARR = 124
  // Clock = 64MHz / 1 = 64MHz (APB1 x2)
  // Freq = 64MHz / ((124+1) * (124+1)) = 64,000,000 / 15625 = 4096 Hz
  TIM3->PSC = 124;
  TIM3->ARR = 124;

  // Configure TRGO (Trigger Output) on Update Event
  // MMS = 010 (Update)
  TIM3->CR2 &= ~TIM_CR2_MMS;
  TIM3->CR2 |= TIM_CR2_MMS_1;
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  // Enable USART2 Clock
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

  // Configure Baud Rate
  // APB1 Clock = 32MHz
  // Baud = 230400
  // USARTDIV = 32,000,000 / (16 * 230400) = 8.6805
  // Mantissa = 8
  // Fraction = 0.6805 * 16 = 10.88 -> 11 (0xB)
  // BRR = 0x8B
  USART2->BRR = 0x8B;

  // Configure Control Register
  // TE (Transmitter Enable), RE (Receiver Enable), UE (USART Enable)
  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/**
  * @brief DMA Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA_Init(void)
{
  // Enable DMA1 Clock
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  // DMA1 Channel 1 (ADC1) Configuration
  // Peripheral Address: &ADC1->DR
  DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
  
  // Memory Address: adc_buffer
  DMA1_Channel1->CMAR = (uint32_t)adc_buffer;
  
  // Number of Data: ADC_BUF_LEN
  DMA1_Channel1->CNDTR = ADC_BUF_LEN;

  // Configuration
  // Priority High (PL=10)
  // Memory Size 16-bit (MSIZE=01)
  // Peripheral Size 16-bit (PSIZE=01)
  // Memory Increment (MINC=1)
  // Circular Mode (CIRC=1)
  // Transfer Error Interrupt (TEIE=1)
  // Half Transfer Interrupt (HTIE=1)
  // Transfer Complete Interrupt (TCIE=1)
  // Enable DMA (EN=1) - Enable later? No, enable now.
  DMA1_Channel1->CCR = DMA_CCR_PL_1 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | 
                       DMA_CCR_MINC | DMA_CCR_CIRC | 
                       DMA_CCR_TEIE | DMA_CCR_HTIE | DMA_CCR_TCIE;
  
  DMA1_Channel1->CCR |= DMA_CCR_EN;

  // Enable DMA1 Channel 1 Interrupt in NVIC
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  // Enable GPIOA, GPIOC Clocks
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;

  // PA5 (LD2) -> Output Push-Pull, 2MHz
  // CNF5=00, MODE5=10
  GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
  GPIOA->CRL |= GPIO_CRL_MODE5_1;

  // PA2 (USART2_TX) -> Alternate Function Push-Pull, 10MHz
  // CNF2=10, MODE2=01
  GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2);
  GPIOA->CRL |= (GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2_0);

  // PA3 (USART2_RX) -> Input Floating (Default) or Pull-up
  // CNF3=01 (Floating), MODE3=00 (Input)
  GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
  GPIOA->CRL |= GPIO_CRL_CNF3_0;

  // PA0, PA1 (ADC) -> Analog Input
  // CNF0=00, MODE0=00
  GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
  GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);

  // PC13 (Button) -> Input Floating
  GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
  GPIOC->CRH |= GPIO_CRH_CNF13_0;
}

// DMA Interrupt Handler
void DMA1_Channel1_IRQHandler(void)
{
  if (DMA1->ISR & DMA_ISR_HTIF1) {
    DMA1->IFCR = DMA_IFCR_CHTIF1;
    fft_data_ready = 1; // First Half Ready
  }
  if (DMA1->ISR & DMA_ISR_TCIF1) {
    DMA1->IFCR = DMA_IFCR_CTCIF1;
    fft_data_ready = 2; // Second Half Ready
  }
}

void UART_Send(uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = data[i];
  }
}

void SPI_Send(uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++) {
    while (!(SPI2->SR & SPI_SR_TXE));
    SPI2->DR = data[i];
  }
}

void Calculate_Features(float32_t* fft_mag, float32_t* time_sig, FeaturePacket* feat) {
    // F1: Peak Frequency Magnitude (Global Peak)
    // Skip DC (index 0)
    float32_t max_val;
    uint32_t max_idx;
    arm_max_f32(&fft_mag[1], (FFT_LEN / 2) - 1, &max_val, &max_idx);
    max_idx += 1; // Adjust for offset because we started search from index 1
    
    feat->F1_peak = max_val;
    feat->F1_freq = (float32_t)max_idx * 4096.0f / (float32_t)FFT_LEN;
    
    // F2: Sideband Magnitude (Targeted Search using 1/7 Rule)
    float32_t sb_max = 0.0f;
    uint32_t sb_idx = 0;
    
    uint32_t fr_idx = max_idx / 7;
    int multipliers[] = {1, 3, 5};
    
    for (int k = 0; k < 3; k++) {
        int dist_idx = fr_idx * multipliers[k];
        int signs[] = {-1, 1};
        for (int s = 0; s < 2; s++) {
            int center_idx = (int)max_idx + (dist_idx * signs[s]);
            
            // Search +/- 5 index around the candidate center
            for (int offset = -5; offset <= 5; offset++) {
                int target_idx = center_idx + offset;
                
                if (target_idx > 0 && target_idx < (FFT_LEN / 2)) {
                    if (target_idx == max_idx) continue;
                    
                    if (fft_mag[target_idx] > sb_max) {
                        sb_max = fft_mag[target_idx];
                        sb_idx = target_idx;
                    }
                }
            }
        }
    }
    
    feat->F2_sideband = sb_max;
    feat->F2_index = (float32_t)sb_idx;
    
    if (sb_max > 0.0f) {
        feat->F2_freq = (float32_t)sb_idx * 4096.0f / (float32_t)FFT_LEN;
    } else {
        feat->F2_freq = 0.0f;
    }

    // F3: 3rd Harmonic Magnitude
    int h3_idx = max_idx * 3;
    if (h3_idx < (FFT_LEN / 2)) {
        feat->F3_harmonic = fft_mag[h3_idx];
        feat->F3_freq = (float32_t)h3_idx * 4096.0f / (float32_t)FFT_LEN;
    } else {
        feat->F3_harmonic = 0.0f;
        feat->F3_freq = 0.0f;
    }

    // F4: RMS (Time Domain)
    float32_t sum_sq = 0.0f;
    for (int i = 0; i < FFT_LEN; i++) {
        sum_sq += time_sig[i] * time_sig[i];
    }
    float32_t mean_sq = sum_sq / (float32_t)FFT_LEN;
    arm_sqrt_f32(mean_sq, &feat->F4_rms);

    // F5: THD
    float32_t harm_energy = 0.0f;
    for (int h = 2; h <= 5; h++) {
        int h_idx = max_idx * h;
        if (h_idx < (FFT_LEN / 2)) {
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

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
