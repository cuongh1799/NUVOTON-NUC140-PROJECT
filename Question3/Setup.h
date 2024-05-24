#ifndef SETUP_FILE
#define SETUP_FILE
#include <stdio.h>
#include <stdint.h>
void SPI3_config(void); // for the LCD
void System_config(void);
void LCD_start(void);
void LCD_command(unsigned char temp);
void LCD_data(unsigned char temp);
void LCD_clear(void);
void LCD_SetAddress(uint8_t PageAddr, uint8_t ColumnAddr);
void SPI2_TX(unsigned char temp);
void clr_segment(void);
void show_segment(unsigned char n, unsigned char number);
#endif