#ifndef BOARD_H
#define BOARD_H

#include "stm32f7xx_hal.h"              // Keil::Device:STM32Cube HAL:Common

void Board_Initialize(void);

void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

void LED2_On(void);
void LED2_Off(void);
void LED2_Toggle(void);

#endif
