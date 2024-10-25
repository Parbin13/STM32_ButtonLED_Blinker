# STM32_ButtonLED_Blinker

This project is an STM32 microcontroller application that controls an LED using a button. The LED blinks at various frequencies based on button presses, demonstrating GPIO control, interrupt handling, and SysTick timer functionality on the STM32 platform.

## Features
- **LED Control**: Blinks the onboard LED (PA.5) at different frequencies.
- **Button Input**: Uses a button (PC.13) to cycle through blink frequencies or reset.
- **Interrupts and SysTick**: Demonstrates usage of external interrupts and SysTick for timing.

## Hardware Requirements
- **Microcontroller**: STM32L4 series (tested on STM32L476RG).
- **Board**: STM32 Nucleo-L476RG (or equivalent).
- **Components**: Onboard LED (PA.5), onboard button (PC.13).

## How It Works
1. **Button Press**: Short presses of the button cycle the LED through four blinking frequencies (500ms, 1000ms, 1500ms, and 2000ms).
2. **Button Hold**: Holding the button for more than 1 second resets the LED and stops blinking.

## Getting Started

### Prerequisites
- **STM32CubeIDE** or **Keil uVision** for development and flashing.
- Basic familiarity with STM32 GPIO, SysTick, and interrupt configuration.

### Project Setup
1. Clone the repository:
   ```bash
   git clone https://github.com/Parbin13/STM32_ButtonLED_Blinker.git
