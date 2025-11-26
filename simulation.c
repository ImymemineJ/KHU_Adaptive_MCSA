#include <stdint.h>
#include <stdio.h>
#include <string.h>

// ============================================================================
// Mocks for STM32 HAL & Types
// ============================================================================
typedef float float32_t;
typedef struct {
  uint32_t Instance;
} ADC_HandleTypeDef;
typedef struct {
  uint32_t Instance;
} SPI_HandleTypeDef;
typedef struct {
  uint32_t Instance;
} UART_HandleTypeDef;
typedef struct {
  uint32_t Instance;
} TIM_HandleTypeDef;

ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim3;

#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define GPIOB 0

// Feature Packet
typedef struct {
  float F1_peak;
  float F2_sideband;
  float F3_harmonic;
  float F4_rms;
  float F5_thd;
} FeaturePacket;

// Commands
#define CMD_MODE_NORMAL 0x01
#define CMD_MODE_HIGH_RES 0x02

// Global Variables from main.c
#define MAX_FFT_LEN 1024
uint16_t current_fft_len = 256;
uint32_t current_sampling_rate = 2000;
uint8_t current_mode = CMD_MODE_NORMAL;
uint8_t spi_rx_buf[sizeof(FeaturePacket)];
FeaturePacket g_feat;

// ============================================================================
// Mock Functions
// ============================================================================
void HAL_GPIO_WritePin(int port, int pin, int state) {
  // printf("GPIO Write: %d\n", state);
}

void HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc) {
  printf("[HAL] ADC Stopped\n");
}

void HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData,
                       uint32_t Length) {
  printf("[HAL] ADC Started (DMA) with Length: %d (Buffer Size)\n", Length);
}

void Reinit_FFT(uint16_t new_len) {
  printf("[DSP] FFT Re-initialized with Length: %d\n", new_len);
}

void Configure_ADC_Timer(uint32_t sampling_rate) {
  printf("[TIM] Timer Configured for Sampling Rate: %d Hz\n", sampling_rate);
}

void Calculate_Features(float32_t *fft_mag, float32_t *time_sig,
                        FeaturePacket *feat, uint16_t fft_len) {
  // Dummy calculation
  feat->F1_peak = 100.0f;
  // printf("[APP] Features Calculated\n");
}

// Simulation Helper to inject commands
uint8_t next_command_to_inject = 0;

void HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData,
                             uint8_t *pRxData, uint16_t Size,
                             uint32_t Timeout) {
  // Simulate receiving a command
  pRxData[0] = next_command_to_inject;
  // printf("[SPI] Transmitted Features, Received Command: 0x%02X\n",
  // next_command_to_inject);

  // Clear injection after one read (simulate master sending it once)
  // In real life, master might send it continuously until ack, but here we
  // assume pulse.
  next_command_to_inject = 0;
}

// ============================================================================
// Main Simulation Loop
// ============================================================================
int main() {
  printf("=== STM32 Adaptive FFT Simulation ===\n");

  // Initial Setup
  Reinit_FFT(current_fft_len);
  Configure_ADC_Timer(current_sampling_rate);
  HAL_ADC_Start_DMA(&hadc1, NULL, current_fft_len * 4);

  printf("\n--- Starting Main Loop ---\n");

  // Simulate 5 iterations
  for (int i = 1; i <= 5; i++) {
    printf("\n[Cycle %d] Mode: %s, Len: %d, Rate: %d\n", i,
           (current_mode == CMD_MODE_NORMAL) ? "NORMAL" : "HIGH_RES",
           current_fft_len, current_sampling_rate);

    // 1. Simulate Data Ready & Processing
    Calculate_Features(NULL, NULL, &g_feat, current_fft_len);

    // 2. Inject Command at Cycle 2 (Switch to High Res)
    if (i == 2) {
      printf(">>> Injecting Command: SWITCH TO HIGH RES\n");
      next_command_to_inject = CMD_MODE_HIGH_RES;
    }
    // 3. Inject Command at Cycle 4 (Switch to Normal)
    if (i == 4) {
      printf(">>> Injecting Command: SWITCH TO NORMAL\n");
      next_command_to_inject = CMD_MODE_NORMAL;
    }

    // 4. SPI Communication (Transmit Features, Receive Command)
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&g_feat, spi_rx_buf,
                            sizeof(FeaturePacket), 10);

    // 5. Check for Mode Change Command (Logic copied from main.c)
    uint8_t received_cmd = spi_rx_buf[0];
    if (received_cmd != 0 && received_cmd != current_mode) {
      printf("[LOGIC] Command Received: 0x%02X\n", received_cmd);

      if (received_cmd == CMD_MODE_HIGH_RES) {
        current_mode = CMD_MODE_HIGH_RES;
        current_fft_len = 1024;
        current_sampling_rate = 8000;
      } else if (received_cmd == CMD_MODE_NORMAL) {
        current_mode = CMD_MODE_NORMAL;
        current_fft_len = 256;
        current_sampling_rate = 2000;
      }

      // Apply Changes
      HAL_ADC_Stop_DMA(&hadc1);
      Reinit_FFT(current_fft_len);
      Configure_ADC_Timer(current_sampling_rate);
      HAL_ADC_Start_DMA(&hadc1, NULL, current_fft_len * 4);
    }
  }

  printf("\n=== Simulation Finished ===\n");
  return 0;
}
