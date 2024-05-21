
#include <stdio.h>
#include "NUC100Series.h"
#include "LCD.h"
#include <stdint.h>
#include <math.h>

#define HXT_STATUS 1<<0
#define PLL_STATUS 1<<2

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
void TMR0_init(void);
	
int main(){

	System_config();
	TMR0_init();
	SPI3_config();
	
	LCD_start();
	LCD_clear();
	
	PB->PMD &= (~(0x03<<30)); // enable GPB15 (the button)
	
	PC->DOUT |= (1<<7);     //Logic 1 to turn on the digit
	PC->DOUT &= ~(1<<6);		//SC3
	PC->DOUT &= ~(1<<5);		//SC2
	PC->DOUT &= ~(1<<4);		//SC1
	
	while(1) { 
		if(!(PB->PIN & (1<<15))) //Check if button pressed, show 3
			{
				clr_segment();
				show_segment(0,6);
				show_segment(1,9);
				//PE->DOUT |= (1<<2);		//segment f
			}
			else // If not pressed, show 9
			{
				clr_segment();
				show_segment(1,9);
				show_segment(0,6);
				/*
				PE->DOUT &= ~(1<<7);		//segment g
				PE->DOUT &= ~(1<<5);		//segment d
				PE->DOUT &= ~(1<<4);	//segment b
				PE->DOUT &= ~(1<<3);		//segment a
				PE->DOUT &= ~(1<<2);		//segment f
				PE->DOUT &= ~(1<<1);	//DOT
				PE->DOUT &= ~(1<<0);		// segment c
				*/
				
			}				
	}
}

void System_config(void){
	SYS_UnlockReg(); // Unlock protected registers
	
	CLK->PWRCON |= (1 << 0);
  while(!(CLK->CLKSTATUS & HXT_STATUS));
	//PLL configuration starts
	CLK->PLLCON &= ~(1 << 19); //0: PLL input is HXT
	CLK->PLLCON &= ~(1 << 16); //PLL in normal mode
	CLK->PLLCON &= (~(0x01FF << 0));
	CLK->PLLCON |= 48;
	CLK->PLLCON &= ~(1 << 18); //0: enable PLLOUT
	while(!(CLK->CLKSTATUS & PLL_STATUS));
	//PLL configuration ends
	
	//clock source selection
	CLK->CLKSEL0 &= (~(0x07 << 0));
	CLK->CLKSEL0 |= (0x02 << 0);
	//clock frequency division
	CLK->CLKDIV &= (~0x0F << 0);
	
	// SPI3 clock enable
	CLK->APBCLK |= 1 << 15;
	
	// SPI2 clock enable
	CLK->APBCLK |= 1 << 14;
	
	//ADC Clock selection and configuration
	CLK->CLKSEL1 &= ~(0x03 << 2); // ADC clock source is 12 MHz
	CLK->CLKDIV &= ~(0x0FF << 16);
	CLK->CLKDIV |= (0x0B << 16); // ADC clock divider is (11+1) --> ADC clock is 12/12 = 1 MHz
	CLK->APBCLK |= (0x01 << 28); // enable ADC clock
	
	SYS_LockReg();  // Lock protected registers 
}

void TMR0_init(void){
		SYS_UnlockReg();
		CLK->CLKSEL1 &= ~(0b111 << 8); // 12 MHZ
		CLK->APBCLK |= (1 << 2);
		TIMER0->TCSR &= ~(0xFF << 0); // Prescaler
		//reset Timer 0
		TIMER0->TCSR |= (1 << 26);
		//define Timer 0 operation mode
		TIMER0->TCSR &= ~(0b11 << 27);
		TIMER0->TCSR |= (0b01 << 27);
		TIMER0->TCSR &= ~(1 << 24);
		//Enable interrupt in the Control Resigter of Timer istsle
		TIMER0->TCSR |= (1 << 29);		
		//TimeOut = 0.5s 
		TIMER0->TCMPR = 16384-1;
		TIMER0->TCSR |= (1 << 30);
		//Setup Timer0 interrupt
		NVIC->ISER[0] |= (1 << 8);
		NVIC->IP[2] &= (~(3 << 6));
		SYS_LockReg();  // Lock protected registers
}

void SPI3_config(void){

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
}


void LCD_start(void)
{
	LCD_command(0xE2); // Set system reset
	LCD_command(0xA1); // Set Frame rate 100 fps  
	LCD_command(0xEB); // Set LCD bias ratio E8~EB for 6~9 (min~max)  
	LCD_command(0x81); // Set V BIAS potentiometer
	LCD_command(0xA0); // Set V BIAS potentiometer: A0 ()        	
	LCD_command(0xC0);
	LCD_command(0xAF); // Set Display Enable
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

 
void clr_segment(void)
{
	unsigned char i;
	for(i=0; i<4; i++)
	{
		PC->DOUT &= ~(1<<(4+i));
	}
}
 
void show_segment(unsigned char n, unsigned char number)
{
	unsigned char i;
	unsigned char number__[10] = {0x82 ,0xEE, 0x07, 0x46, 0x6A, 0x52, 0x12, 0xE6, 0x02, 0x62};
	for(i=0; i<8; i++)
	{
		if((number__[number]&0x01)==0x01)
		PE->DOUT |= (1<<i);
		else
		PE->DOUT &= ~(1<<i);
		number__[number]=number__[number]>>1;
	}
	PC->DOUT |= (1<<(4+n));
}



