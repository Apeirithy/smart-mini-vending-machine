#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdint.h>

void LCD_Init(uint8_t addr);
void LCD_Clear(void);
void LCD_Print(const char *str);
void LCD_Goto(uint8_t col, uint8_t row);

#endif 
