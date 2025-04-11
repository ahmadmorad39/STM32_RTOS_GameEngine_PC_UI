# STM32 + FreeRTOS Game Engine with Python GUI

This project showcases a FreeRTOS-based game engine running on the STM32F429I-DISC1 board, which offloads rendering, sound, and logging to a PC-based Python GUI.

## Features

- Real-time multitasking with FreeRTOS
- UART-based game state streaming
- Python GUI (Pygame/PyQt5) displays game in real time
- Optional: logging, sound, score plotting

## Technologies

- STM32 HAL + FreeRTOS
- UART (115200 baud)
- Python 3 + PySerial + PyGame/PyQt5

## Setup

- Flash STM32 firmware from `stm32-firmware/`
- Run GUI via `python3 python-ui/main.py`
- Connect USB UART (check COM port)
