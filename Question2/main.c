
#include <stdio.h>
#include "NUC100Series.h"
#include "LCD.h"
#include <stdint.h>
#include <math.h>

#define HXT_STATUS 1<<0
#define PLL_STATUS 1<<2

void SPI2_config(void);
void SPI3_config(void);
void System_config(void);
void ADC_config(void);
void LCD_start(void);
void LCD_command(unsigned char temp);
void LCD_data(unsigned char temp);
void LCD_clear(void);
void LCD_SetAddress(uint8_t PageAddr, uint8_t ColumnAddr);
void SPI2_TX(unsigned char temp);

int main(){
	uint32_t adc7_val;
	double const Vref = 3.3;
	double const bitRange = pow(2, 12) - 1;
	double voltage_input_val;
	
	char adc7_val_s[4] = "0000";
	char voltage_input_val_s[4] = "0000";
	
	System_config();
	SPI3_config();
	SPI2_config();
	ADC_config();
	
	LCD_start();
	LCD_clear();

	printS_5x7(2, 0, "meow meow meow");
	printS_5x7(2, 8, "ADC7 conversion test");
	printS_5x7(2, 16, "Reference voltage: 3.3 V");
	printS_5x7(2, 24, "A/D resolution: 0.73 mV"); // <------------------------ remember to change this
	printS_5x7(2, 40, "A/D value:");
	printS_5x7(2, 48, "Voltage value:");

	ADC->ADCR |= (1 << 11); // start ADC channel 7 conversion

	while (1) {
		
		while (!(ADC->ADSR & (1 << 0))); // wait until conversion is completed (ADF=1)
		ADC->ADSR |= (1 << 0); // write 1 to clear ADF
		adc7_val = ADC->ADDR[7] & 0x0000FFFF;
		
		// Convert `adc7_val` to char[]
		sprintf(adc7_val_s, "%d", adc7_val);
		printS_5x7(4 + 5 * 10, 40, "    ");
		printS_5x7(4 + 5 * 10, 40, adc7_val_s);
		
		/*
		Vref is 3.3V
		Resolution is 3.3 / ( 2^12 - 1) = 8.0586x10^-4 V
		Now, we know the ADC by adjusting the Potentiometer, therefore to find the VIn,
		we need to take in the `adc7_val` * 8.0586x10^-4 V (Formula in ADC lecture pg.16)
		Need to cast `uint32_t` because sprintf requires `int` parameter
		*/
		voltage_input_val = adc7_val * Vref / bitRange;
		sprintf(voltage_input_val_s, "%d", (uint32_t)voltage_input_val);
		printS_5x7(35 + 5 * 10, 48, "    ");
		printS_5x7(35 + 5 * 10, 48, voltage_input_val_s);
		
		if(voltage_input_val > 2){
			printS_5x7(2, 56, "hehe");
			
			// put AD at 2585 for best quality hehe
			SPI2_TX(0x48); // H
			SPI2_TX(0x45); // E
			SPI2_TX(0x48); // H
			SPI2_TX(0x45); // E
			
			//SPI2_TX(0xE2);
			CLK_SysTickDelay(200);
			
		}
		else {
			printS_5x7(2, 56, "     ");
		}
		 
		//CLK_SysTickDelay(2000000);
	}

}

void ADC_config(void){
	
	/* 
	SETTING UP ADC CHANNEL 7
	
	So basically we're setting GPA7 to ouput mode, then change the function of the pin to ADC7
	To do that we need GPA_MFP[7] to 1, EBI_EN (ALT_MFP[11]) to 0
	*/
	
	PA->PMD &= ~(0b11 << 14);
	PA->PMD |= (0b01 << 14); 
	PA->OFFD |= (0x01 << 7); // PA.7 digital input path is disabled
	
	SYS->GPA_MFP |= 1 << 7; // Change GPA7 to ADC7 page.68
	SYS->ALT_MFP &= ~(1 << 11); // External Bus Interface page.83
	
	ADC->ADCR |= (0b11 << 2); // Continuous scan mode
	ADC->ADCR &= ~(1 << 1); // ADC interrupt is disabled
	
	ADC->ADCR |= 1 << 0; // ADC enable
	
	ADC->ADCHER &= ~(0b11 << 8); 
	ADC->ADCHER |= (1 << 7); // Analog Input Channel 7 enable , page.482

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

void SPI2_config(void){
	
	SYS->GPD_MFP |= 1 << 3; //1: PD3 is configured for alternative function
	SYS->GPD_MFP |= 1 << 1; //1: PD1 is configured for alternative function
	SYS->GPD_MFP |= 1 << 0; //1: PD0 is configured for alternative function
	
	SPI2->CNTRL &= ~(1 << 23); //0: disable variable clock feature
	SPI2->CNTRL &= ~(1 << 22); //0: disable two bits transfer mode
	SPI2->CNTRL &= ~(1 << 18); //0: select Master mode
	SPI2->CNTRL &= ~(1 << 17); //0: disable SPI interrupt
	SPI2->CNTRL |= 1 << 11; //1: SPI clock idle high
	SPI2->CNTRL &= ~(1 << 2); // Disable transmit at negative edge
	//SPI2->CNTRL |= 1 << 1; // test later hehe idk if its needed
	SPI2->CNTRL &= ~(1 << 10); 
	SPI2->CNTRL |= 1 << 10; //1: LSB is sent first
	SPI2->CNTRL |= 8 << 3; // 1 byte/word
	
	SPI2->DIVIDER = 24; // SPI clock divider. SPI clock = HCLK / ((DIVIDER+1)*2)
	
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

void SPI2_TX(unsigned char temp)
{
	SPI2->SSR |= 1 << 0;
	SPI2->TX[0] = temp;
	SPI2->CNTRL |= 1 << 0;
	while (SPI2->CNTRL & (1 << 0));
	SPI2->SSR &= ~(1 << 0);
}



