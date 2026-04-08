#ifndef I2C_OLED_H
#define I2C_OLED_H

#include "stm32f4xx.h"

// OLED I2C Address (0x3C shifted left by 1 = 0x78)
#define OLED_ADDR 0x78 

// Function Prototypes
void I2C1_Init(void);
void OLED_WriteCmd(uint8_t cmd);
void OLED_WriteData(uint8_t data);
void OLED_Init(void);
void OLED_SetCursor(uint8_t row, uint8_t col);
void OLED_Clear(void);
void OLED_PrintChar(char c);
void OLED_PrintString(char* str);

#endif
