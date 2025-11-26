# 적응형 FFT 및 샘플링 레이트 구현 계획

## 목표 설명
STM32_B 보드로부터 SPI Full Duplex 통신을 통해 피드백을 받아, STM32_A 보드가 FFT 해상도(샘플 수)와 샘플링 레이트를 동적으로 조절하는 적응형 모니터링 시스템을 구현합니다. 이를 통해 평상시에는 저전력/저부하로 모니터링하고, 잠재적인 결함이 의심될 때는 정밀 분석을 수행할 수 있습니다.

## 사용자 검토 필요 사항
> [!IMPORTANT]
> **메모리 할당**: "Low Res" 모드에서도 힙 단편화를 방지하기 위해 **최대** 지원 FFT 길이(예: 1024 또는 2048)에 맞춰 버퍼를 정적으로 할당해야 합니다.
> **SPI 프로토콜**: STM32_A는 마스터(Master)입니다. *현재 주기의 특징값*을 전송하는 동안 *다음 주기를 위한 명령*을 수신합니다.

## 변경 제안 사항 (STM32_A)

### Core/Src
#### [MODIFY] [main.c](file:///Users/alice/Desktop/project2/STM32_A-FFT/Core/Src/main.c)
- **전역 변수**:
    - `FFT_LEN` 매크로를 `current_fft_len` 변수로 변경합니다.
    - 버퍼 크기 결정을 위한 `MAX_FFT_LEN`을 정의합니다.
    - `current_sampling_rate` 변수를 추가합니다.
- **SPI 통신**:
    - `HAL_SPI_Transmit`을 `HAL_SPI_TransmitReceive`로 교체합니다.
    - STM32_B로부터 명령을 수신할 `spi_rx_buf` 수신 버퍼를 추가합니다.
- **새로운 함수**:
    - `void Configure_ADC_Timer(uint32_t sampling_rate)`: TIM3의 ARR/PSC 레지스터를 조정합니다.
    - `void Reinit_FFT(uint16_t new_len)`: `arm_rfft_fast_instance_f32` 구조체를 재초기화합니다.
- **메인 루프**:
    - 전송 후 `spi_rx_buf`를 파싱합니다.
    - 수신된 명령이 현재 상태와 다를 경우, 재설정 함수들을 호출합니다.
    - "히스테리시스"(잦은 모드 변경 방지) 로직은 STM32_B에서 처리한다고 가정하지만, STM32_A는 명령을 즉시 수행해야 합니다.

### Core/Inc
#### [MODIFY] [main.h](file:///Users/alice/Desktop/project2/STM32_A-FFT/Core/Inc/main.h)
- SPI 명령 정의를 추가합니다 (예: `CMD_MODE_NORMAL`, `CMD_MODE_HIGH_RES`).

---

## STM32_B 구현 가이드 (수신부)

STM32_B는 **SPI Slave**로서 STM32_A로부터 데이터를 수신하고, 분석 결과에 따라 다음 주기의 동작 모드를 결정하여 명령을 보냅니다.

### 1. SPI Slave 설정
- **Mode**: SPI Slave
- **Data Size**: 8bit
- **CPOL/CPHA**: STM32_A와 동일하게 설정 (Low/1Edge)
- **NSS**: Hardware NSS Input 또는 Software NSS (STM32_A가 CS를 제어함)

### 2. 통신 로직 (Full Duplex)
STM32_B는 **"이번에 받은 데이터"**를 분석하여 **"다음에 보낼 명령"**을 결정합니다.
따라서 `HAL_SPI_TransmitReceive`를 호출할 때, `pTxData`에는 **이미 결정된 다음 명령**이 들어있어야 합니다.

```c
// 예시 코드 구조
uint8_t rx_buffer[sizeof(FeaturePacket)];
uint8_t next_command = CMD_MODE_NORMAL; // 초기값

void SPI_Slave_Process() {
    // 1. 데이터 수신 및 명령 전송 (동시)
    // 주의: Slave는 Master가 클럭을 주기 전까지 대기하므로, 인터럽트나 DMA 방식 권장
    if (HAL_SPI_TransmitReceive(&hspi1, &next_command, rx_buffer, sizeof(FeaturePacket), 100) == HAL_OK) {
        
        // 2. 수신된 데이터 파싱
        FeaturePacket* feat = (FeaturePacket*)rx_buffer;
        
        // 3. 상태 판단 로직 (Threshold Check)
        // 예: F1(Peak)가 1000을 넘거나, THD가 10% 이상이면 위험으로 간주
        if (feat->F1_peak > 1000.0f || feat->F5_thd > 0.1f) {
            // 위험 감지 -> High Res 모드 요청
            next_command = CMD_MODE_HIGH_RES;
            hysteresis_counter = 0; // 카운터 리셋
        } else {
            // 정상 상태
            // 히스테리시스: 즉시 낮추지 않고, 일정 시간(예: 50사이클) 동안 정상이 유지되면 복귀
            hysteresis_counter++;
            if (hysteresis_counter > 50) {
                next_command = CMD_MODE_NORMAL;
            }
        }
    }
}
```

### 3. 히스테리시스 (Hysteresis) 적용
- **목적**: 경계값 근처에서 모드가 빈번하게 바뀌는(Chattering) 현상 방지.
- **로직**:
    - **Normal -> High Res**: 즉시 전환 (위험은 빨리 감지해야 함).
    - **High Res -> Normal**: 일정 시간(Count) 동안 정상 상태가 유지될 때만 전환.

## 검증 계획
### 수동 검증
- **SPI 루프백 (시뮬레이션)**: STM32_B가 없으므로, N초 후 모드를 변경하거나 버튼 입력을 통해 명령 수신을 시뮬레이션합니다.
- **UART 출력**: UART 로그를 통해 다음 사항을 확인합니다:
    - FFT 길이 변경 (예: 256 -> 1024).
    - 샘플링 레이트 변경 (예: 2kHz -> 8kHz).
    - 모드 전환 후에도 피크 주파수 감지가 정확한지 확인.
