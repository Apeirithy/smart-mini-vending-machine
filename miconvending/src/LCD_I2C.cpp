#include "LCD_I2C.h"
#include "i2c.h"
#include <util/delay.h>
#include <stdint.h>
static uint8_t LCD_ADDR;
#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_RS        0x01  

static void lcd_pulse(uint8_t b)
{
    i2c_start((LCD_ADDR << 1) | 0); 
    i2c_write(b | LCD_ENABLE);
    _delay_us(1);
    i2c_write(b & ~LCD_ENABLE);
    i2c_stop();
}

static void lcd_write4(uint8_t nibble, uint8_t mode)
{
    uint8_t upper = (nibble & 0xF0) | LCD_BACKLIGHT;
    if (mode) upper |= LCD_RS;
    lcd_pulse(upper);
}

void LCD_Send(uint8_t data, uint8_t mode)
{
    uint8_t high = data & 0xF0;
    uint8_t low  = (data << 4) & 0xF0;

    lcd_write4(high, mode);
    lcd_write4(low, mode);
    _delay_us(50);
}

void LCD_Command(uint8_t cmd)
{
    LCD_Send(cmd, 0x00);
    _delay_ms(2);
}

void LCD_Char(char c)
{
    LCD_Send((uint8_t)c, 0x01);
}

void LCD_Init(uint8_t addr)
{
    LCD_ADDR = addr;

    _delay_ms(50);

    i2c_start((LCD_ADDR << 1) | 0);
    i2c_write(0x00);
    i2c_stop();
    _delay_ms(50);

    lcd_write4(0x30, 0); _delay_ms(5);
    lcd_write4(0x30, 0); _delay_ms(5);
    lcd_write4(0x20, 0); _delay_ms(5); 

    LCD_Command(0x28); 
    LCD_Command(0x0C); 
    LCD_Command(0x06); 
    LCD_Clear();
}

void LCD_Clear(void)
{
    LCD_Command(0x01);
    _delay_ms(2);
}

void LCD_Goto(uint8_t col, uint8_t row)
{
    uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_Command(addr);
}

void LCD_Print(const char *str)
{
    while (*str) {
        LCD_Char(*str++);
    }
}
