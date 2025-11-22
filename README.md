# STM32 Real-Time FFT Analyzer 📊

![STM32](https://img.shields.io/badge/STM32-F103RB-blue.svg) ![License](https://img.shields.io/badge/License-MIT-yellow.svg)

A high-performance, real-time Fast Fourier Transform (FFT) implementation on the STM32F103 microcontroller. This project demonstrates how to capture analog signals using ADC with DMA and process them using the CMSIS-DSP library to analyze frequency components efficiently.

## 🚀 Features

*   **Real-Time Processing**: Utilizes **Double Buffering (Ping-Pong)** with DMA to ensure zero data loss during continuous sampling.
*   **DMA vs Polling Comparison**: Includes a compile-time switch to toggle between DMA and Polling modes, demonstrating the impact of "Blind Time" on signal analysis.
*   **High Precision**: Implements 512-point Floating-Point RFFT (Real FFT) using ARM CMSIS-DSP.
*   **Dual Channel Monitoring**: Simultaneously captures **Signal (CH0)** and **Temperature (CH1)** using ADC Scan Mode.
*   **Live Monitoring**: Outputs Peak Frequency, Magnitude, and Temperature via UART for real-time analysis on a PC.
*   **Performance Verified**: Optimized to run within the sampling period (Processing time < 110ms @ 4.6kHz Sampling).
*   **Safety First**: Includes critical section handling to prevent race conditions between DMA interrupts and main loop processing.

## 🛠 Hardware Requirements

*   **MCU**: STM32F103RB (Nucleo-F103RB or equivalent)
*   **Input**: Analog Signal Generator (Sine/Square wave)
*   **Debug**: USB-to-UART Converter (or ST-Link VCOM)
*   **Measurement**: Oscilloscope (for timing verification)

### Pin Configuration
| Pin | Function | Description |
| :--- | :--- | :--- |
| **PA0** | ADC_IN0 | Analog Signal Input (0V ~ 3.3V) |
| **PA1** | ADC_IN1 | Temperature Sensor Input (e.g., LM35) |
| **PA2** | USART2_TX | UART Debug Output (115200 bps / 460800 bps) |
| **PA3** | USART2_RX | UART Debug Input |
| **PA5** | LD2 (LED) | Timing Measurement / Status LED |

> [!WARNING]
> **Input Voltage Warning**: The ADC input range is **0V to 3.3V**. Do NOT apply negative voltage or voltage exceeding 3.3V. Ensure your signal has a proper DC offset (e.g., 1.65V).

## 💻 Software Requirements

*   **IDE**: STM32CubeIDE (v1.10.0 or later recommended)
*   **Library**: ARM CMSIS-DSP Library (Pre-compiled lib required)

## ⚙️ Installation & Build

1.  **Clone the Repository**
    ```bash
    git clone https://github.com/Adaptive-MCSA-Redundancy-System/STM32_A-FFT.git
    ```

2.  **Import to STM32CubeIDE**
    *   `File` -> `Import` -> `Existing Projects into Workspace` -> Select the project directory.

3.  **Link CMSIS-DSP Library**
    *   Ensure `libarm_cortexM3l_math.a` is linked.
    *   Go to `Project Properties` -> `C/C++ Build` -> `Settings` -> `MCU GCC Linker` -> `Libraries`.
    *   Add `arm_cortexM3l_math` to Libraries (-l).
    *   Add the library path to Search Path (-L).

4.  **Build & Flash**
    *   Click the **Hammer** icon to build.
    *   Click the **Run/Debug** icon to flash the firmware to the STM32 board.

## 📖 Usage

1.  **Connect Hardware**: Connect your signal generator to **PA0** and **GND**.
2.  **Open Terminal**: Connect the board to PC and open a serial terminal (Tera Term, PuTTY).
    *   Baud Rate: **115200**
    *   Data: 8 bit, Parity: None, Stop: 1 bit
3.  **Run**: Reset the board. You should see the peak frequency output:
    ```text
    Peak: 1000.0 Hz, Mag: 45000, Temp: 25.5 C
    Peak: 1000.0 Hz, Mag: 44950, Temp: 25.6 C
    ...
    ```
4.  **Verify Timing**: Probe **PA5** with an oscilloscope. The High pulse width represents the FFT calculation time.

## 🔬 Experiment: DMA vs Polling

You can switch between DMA and Polling modes to observe the importance of DMA in real-time systems.

1.  Open `Core/Src/main.c`.
2.  Modify the `USE_DMA` macro:
    ```c
    #define USE_DMA 1 // 1: DMA Mode (No Data Loss), 0: Polling Mode (Data Loss)
    ```
3.  **DMA Mode (`1`)**:
    *   ADC writes to buffer via DMA in the background.
    *   CPU processes the other half of the buffer simultaneously.
    *   **Result**: Continuous monitoring, no blind spots.
4.  **Polling Mode (`0`)**:
    *   CPU is busy waiting for ADC conversion.
    *   ADC stops capturing while CPU is processing FFT (Blind Time ~42ms).
    *   **Result**: Signal updates are slower (~264ms period) and data during processing is lost.

### 4. Experimental Results (Observed)
| Feature | DMA On (`USE_DMA 1`) | DMA Off (`USE_DMA 0`) |
| :--- | :--- | :--- |
| **FPS (Update Rate)** | **Stable ~4.0 FPS** (Max 5.7) | **Unstable 1.9 ~ 4.0 FPS** |
| **Signal Continuity** | Smooth, continuous movement | Jerky, "teleporting" peaks |
| **Blind Time** | **Zero** (0ms) | **~50ms** per frame |

> **Analysis**:
> Even though the maximum FPS is limited by the physical sampling time (~220ms), **DMA Mode** maintains a stable frame rate because data acquisition never stops.
> **Polling Mode** suffers from significant frame drops (down to 1.9 FPS) because the CPU stops capturing data while processing the FFT, leading to "Blind Time" where signal changes are completely missed.

## 🧠 Technical Details

### Double Buffering (Ping-Pong)
To achieve real-time performance, we use a buffer size of `2 * FFT_LEN`.
*   **Half-Complete Interrupt**: Triggered when the first half is full. CPU processes the first half while DMA fills the second half.
*   **Full-Complete Interrupt**: Triggered when the second half is full. CPU processes the second half while DMA fills the first half (Circular Mode).

### Signal Processing Chain
1.  **ADC Capture**: 12-bit samples @ ~4.1kHz (4096Hz).
    *   **Scan Mode**: Captures CH0 (Signal) and CH1 (Temperature) sequentially.
    *   **FFT Length**: 512 samples (Reduced from 1024 to accommodate dual-channel buffering in RAM).
    *   **Resolution**: ~8 Hz per bin.
2.  **Int-to-Float**: Convert `uint16_t` to `float32_t`.
3.  **RFFT**: Compute Real FFT using `arm_rfft_fast_f32`.
4.  **Magnitude**: Calculate `sqrt(Re^2 + Im^2)`.
5.  **Peak Search**: Find the bin with maximum magnitude (ignoring DC).