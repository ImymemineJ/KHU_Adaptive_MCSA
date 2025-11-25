# SPI Communication Troubleshooting Log 🛠️

This document records the issues encountered during the implementation of SPI communication between STM32_A (Master) and STM32_B (Slave), and the solutions applied.

## 1. NSS Pin Configuration Issue
*   **Symptom**: STM32_B received no data or inconsistent signals.
*   **Cause**: The NSS pin (PB12) on STM32_A was not initialized as a GPIO Output in `main.c`, leaving it floating or in an undefined state.
*   **Solution**:
    *   Explicitly initialized PB12 as `GPIO_MODE_OUTPUT_PP` in `MX_GPIO_Init`.
    *   Set initial state to High (`GPIO_PIN_SET`).

## 2. SPI Overrun Error (OVR)
*   **Symptom**: STM32_B reported SPI Error Code 4 (OVR).
*   **Cause**: STM32_A was transmitting data faster than STM32_B could process and clear its RX buffer.
*   **Solution**:
    *   Lowered STM32_A SPI BaudRatePrescaler from `2` to `256` in `MX_SPI2_Init`.
    *   This significantly reduced the clock speed, giving the Slave ample time to process interrupts/DMA.

## 3. Mode Fault Error (MODF) & Timing
*   **Symptom**: STM32_B reported SPI Error Code 32 (MODF) or data was corrupted.
*   **Cause**:
    *   NSS signal timing was too tight. NSS was pulled Low and data transmission started immediately, or NSS was released too quickly.
    *   Potential noise on the NSS line causing the Master to think it lost bus control.
*   **Solution**:
    *   Added a software delay loop (`for(volatile int k=0; k<100; k++) { __NOP(); }`) after pulling NSS Low and before calling `HAL_SPI_Transmit`.
    *   Ensured NSS is pulled High only after transmission completes.

## 4. Data Synchronization (Bit Shift)
*   **Symptom**: Received data values were incorrect (e.g., extremely large or zero floats), but no hardware error flags were raised. Hex dump showed shifted bits.
*   **Cause**: Lack of a synchronization mechanism. If the Slave missed a bit or started reading mid-stream, all subsequent bits would be misaligned.
*   **Solution**:
    *   **Protocol Update**: Added a 2-byte Header (`0xAA`, `0xBB`) to the start of every packet.
    *   **Packet Structure**: `[0xAA] [0xBB] [F1] [F2] [F3] [F4] [F5]` (Total 22 Bytes).
    *   **Slave Logic**: STM32_B now scans for `0xAA`, `0xBB` to identify the start of a valid packet.

## 5. Signal Integrity (Ringing) - **CRITICAL**
*   **Symptom**: Data corruption persisted despite protocol fixes. Signals looked noisy on the oscilloscope.
*   **Cause**: GPIO Output Speed for SPI pins (SCK, MOSI) was set to `GPIO_SPEED_FREQ_HIGH` (50MHz). In a jumper wire environment, this high slew rate caused "Ringing" (signal oscillation), leading to false clock edges or bit errors.
*   **Solution**:
    *   Modified `stm32f1xx_hal_msp.c` to change GPIO Speed from `HIGH` to `LOW`.
    *   **Result**: Smoother signal edges, eliminated ringing, and stable communication.

---

## Final Configuration Summary (STM32_A)

| Parameter | Value | Note |
| :--- | :--- | :--- |
| **SPI Mode** | Master | |
| **Prescaler** | 256 | Slow speed for stability |
| **CPOL/CPHA** | Low / 1Edge | Mode 0 |
| **NSS** | Software (PB12) | Manual control with delay |
| **GPIO Speed** | **LOW** | **Crucial for signal integrity** |
| **Packet Size** | 22 Bytes | Header(2) + Data(20) |
