//------------------------------------------- main.c CODE STARTS ---------------------------------------------------------------------------
#include <stdio.h>
#include "NUC100Series.h"
#include "LCD.h"

#define HXT_STATUS 1<<0
#define PLL_STATUS 1<<2

void System_Config(void);
void LCD_start(void);
void LCD_command(unsigned char temp);
void LCD_data(unsigned char temp);
void LCD_clear(void);
void LCD_SetAddress(uint8_t PageAddr, uint8_t ColumnAddr);

int main(void)
{
	//System initialization
	System_Config();

	//--------------------------------
	//SPI3 initialization
	//--------------------------------
	SYS->GPD_MFP |= 1 << 11; //1: PD11 is configured for alternative function
	SYS->GPD_MFP |= 1 << 9; //1: PD9 is configured for alternative function
	SYS->GPD_MFP |= 1 << 8; //1: PD8 is configured for alternative function

	SPI3->CNTRL &= ~(1 << 23); //0: disable variable clock feature
	SPI3->CNTRL &= ~(1 << 22); //0: disable two bits transfer mode
	SPI3->CNTRL &= ~(1 << 18); //0: select Master mode
	SPI3->CNTRL &= ~(1 << 17); //0: disable SPI interrupt

	SPI3->CNTRL |= 1 << 11; //1: SPI clock idle high
	SPI3->CNTRL &= ~(1 << 10); //0: MSB is sent first
	SPI3->CNTRL &= ~(0b11 << 8); //00: one transmit/receive word will be executed in one data transfer
	SPI3->CNTRL &= ~(0b11111 << 3);
	SPI3->CNTRL |= 9 << 3; //9 bits/word
	SPI3->CNTRL |= (1 << 2);  //1: Transmit at negative edge of SPI CLK

	SPI3->DIVIDER = 24; // SPI clock divider. SPI clock = HCLK / ((DIVIDER+1)*2)

	LCD_start(); //start the LCD display
	LCD_clear(); //clear current LCD content

				 //Turn on some pixels at four corners of the LCD
	LCD_SetAddress(0, 2); // move to (page,column)=(0,2)
	LCD_data(0xF0); // write data to that (0,2) position
	LCD_SetAddress(0, 129);
	LCD_data(0x0F);
	LCD_SetAddress(7, 129);
	LCD_data(0x33);
	LCD_SetAddress(7, 2);
	LCD_data(0x55);
}

void System_Config(void) {
	SYS_UnlockReg(); // Unlock protected registers
	CLK->PWRCON |= 1 << 0;
	while (!(CLK->CLKSTATUS & HXT_STATUS));
	//PLL configuration starts
	CLK->PLLCON &= ~(1 << 19); //0: PLL input is HXT
	CLK->PLLCON &= ~(1 << 16); //PLL in normal mode
	CLK->PLLCON &= (~(0x01FF << 0));
	CLK->PLLCON |= 8;
	CLK->PLLCON &= ~(1 << 18);
	while (!(CLK->CLKSTATUS & PLL_STATUS));
	//PLL configuration ends
	//clock source selection
	CLK->CLKSEL0 &= (~(0x07 << 0));
	CLK->CLKSEL0 |= (0x02 << 0);
	//clock frequency division
	CLK->CLKDIV &= (~0x0F << 0);
	//enable clock of SPI3
	CLK->APBCLK |= 1 << 15;		//Enable clock for SPI3		
	SYS_LockReg();  // Lock protected registers		
}

void LCD_start(void)
{
	LCD_command(0xE2);
	LCD_command(0xA1);
	LCD_command(0xEB);

	LCD_command(0x81);
	LCD_command(0xA0);
	LCD_command(0xC0);
	LCD_command(0xAF);
}

void LCD_command(unsigned char temp)
{
	SPI3->SSR |= 1 << 0;
	SPI3->TX[0] = temp;
	SPI3->CNTRL |= 1 << 0;
	while (SPI3->CNTRL & (1 << 0));
	SPI3->SSR &= ~(1 << 0);
}

void LCD_data(unsigned char temp)
{
	SPI3->SSR |= 1 << 0;
	SPI3->TX[0] = 0x0100 + temp;
	SPI3->CNTRL |= 1 << 0;
	while (SPI3->CNTRL & (1 << 0));
	SPI3->SSR &= ~(1 << 0);
}

void LCD_clear(void)
{
	int16_t i;
	LCD_SetAddress(0x0, 0x0);
	for (i = 0; i < 132 * 8; i++)
	{
		LCD_data(0x00);
	}
}

void LCD_SetAddress(uint8_t PageAddr, uint8_t ColumnAddr)
{
	LCD_command(0xB0 | PageAddr);
	LCD_command(0x10 | (ColumnAddr >> 4) & 0xF);
	LCD_command(0x00 | (ColumnAddr & 0xF));
}
//------------------------------------------- main.c CODE ENDS ---------------------------------------------------------------------------
