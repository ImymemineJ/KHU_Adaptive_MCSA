# STM32 Real-Time FFT Analyzer 📊

![STM32](https://img.shields.io/badge/STM32-F103RB-blue.svg) ![License](https://img.shields.io/badge/License-MIT-yellow.svg)

A high-performance, real-time Fast Fourier Transform (FFT) implementation on the STM32F103 microcontroller. This project demonstrates how to capture analog signals using ADC with DMA and process them using the CMSIS-DSP library to analyze frequency components efficiently.

## 🚀 Features

*   **Real-Time Processing**: Utilizes **Double Buffering (Ping-Pong)** with DMA to ensure zero data loss during continuous sampling.
*   **DMA vs Polling Comparison**: Includes a compile-time switch to toggle between DMA and Polling modes, demonstrating the impact of "Blind Time" on signal analysis.
*   **High Precision**: Implements 1024-point Floating-Point RFFT (Real FFT) using ARM CMSIS-DSP.
*   **Live Monitoring**: Outputs Peak Frequency and Magnitude via UART for real-time analysis on a PC.
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
| **PA2** | USART2_TX | UART Debug Output (115200 bps) |
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
    git clone https://github.com/yourusername/STM32_A-FFT.git
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
*   **Full-Complete Interrupt**: Triggered when the second half is full. CPU processes the second half while DMA fills the first half (Circular Mode).

### Signal Processing Chain
1.  **ADC Capture**: 12-bit samples @ ~4.6kHz.
2.  **Int-to-Float**: Convert `uint16_t` to `float32_t`.
3.  **RFFT**: Compute Real FFT using `arm_rfft_fast_f32`.
4.  **Magnitude**: Calculate `sqrt(Re^2 + Im^2)`.
5.  **Peak Search**: Find the bin with maximum magnitude (ignoring DC).

## 🤝 Contributing
Contributions are welcome! Please fork the repository and submit a Pull Request.

## 📄 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
