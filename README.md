# Tiva-C FreeRTOS Dashboard + HMI Demo

A comprehensive **real-time embedded system** built for the **EK-TM4C1294XL** LaunchPad featuring a full-color touchscreen HMI, sensor integration, motor control, and fault simulation using FreeRTOS.

![Tiva-C Dashboard](https://via.placeholder.com/800x400?text=Dashboard+Screenshot)  
*(Add screenshots here)*

## ✨ Features

- **Multi-tasking Real-Time OS** using FreeRTOS
- **Interactive Graphical User Interface** with Kentec 320x240 touchscreen
- **Real-time Light Sensing** using OPT3001 sensor with moving average filtering
- **Live Plotting** of raw and filtered sensor data
- **Motor State Machine** (Idle → Starting → Running → Fault)
- **Fault Simulation & Safety System** with e-brake, overcurrent, overtemperature, etc.
- **Live Dashboard** with speed, temperature, time, and fault indicators
- **Multiple Panels** (Dashboard, Engine, Light Monitor, etc.)
- **Touch Controls**: Sliders, buttons, checkboxes, and navigation
- **Button Interrupt Handling** with semaphore synchronization
- **UART Debug Output** with thread-safe printing

## 🛠️ Hardware

- **MCU**: TM4C1294NCPDT (EK-TM4C1294XL LaunchPad)
- **Display**: Kentec 320x240 16-bit Color TFT with Touchscreen
- **Sensor**: OPT3001 Ambient Light Sensor (I2C)
- **Input**: On-board SW1 & SW2 buttons
- **Output**: 4 User LEDs

## 🧰 Software Stack

- **RTOS**: FreeRTOS
- **Graphics**: Texas Instruments Graphics Library (GrLib)
- **Widgets**: Custom widget-based HMI framework
- **DriverLib**: TI Peripheral Driver Library
- **Build System**: PlatformIO (recommended) or Keil/IAR/Code Composer Studio

## 📊 System Architecture

The application is built around **6 main FreeRTOS tasks**:

| Task                    | Priority | Role |
|------------------------|----------|------|
| `prvOptTask`           | 2        | Reads OPT3001 sensor every 50ms + filtering |
| `readerTask`           | 2        | Queue consumer, plot buffer management |
| `prvDisplayTask`       | 3        | GUI rendering and touch handling |
| `prvMotorTask`         | 2        | Motor state machine |
| `prvFaultSimulatorTask`| 2        | Random fault generation & sensor simulation |
| `prvLEDTasksw1`        | 2        | Button handling via semaphore |

**Communication:**
- Queue for sensor data (latest value only)
- Event Group for system events (thresholds, faults, mode changes)
- Mutex-protected UART output

## 🚀 Getting Started

### Prerequisites
- EK-TM4C1294XL LaunchPad
- Kentec 320x240 display connected via BoosterPack headers
- PlatformIO or TI Code Composer Studio

### Build & Flash
```bash
# Using PlatformIO
platformio run -t upload
