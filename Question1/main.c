#include <stdio.h>
#include "NUC100Series.h"
#include "MCU_init.h"
#include "SYS_init.h"

void UART02_IRQHandler(void);
void UART0_Init(void);

int main(void) {
		PD->PMD &= ~(0x03 << 6);
		PD->PMD |= 1 << 6;
    // Initialize system clock and multi-function I/O
    SYS_Init();
    
    // Initialize UART0
    UART0_Init();
    
    while (1) {
			
    }
}

void UART0_Init(void) {
	// UART0 pin configuration. PB.1 pin is for UART0 TX
	PB->PMD &= ~(0b11 << 2);
	PB->PMD |= (0b01 << 2); // PB.1 is output pin
	SYS->GPB_MFP |= (1 << 1); // GPB_MFP[1] = 1 -> PB.1 is UART0 TX pin
	
	SYS->GPB_MFP |= (1 << 0); // GPB_MFP[0] = 1 -> PB.0 is UART0 RX pin	
	PB->PMD &= ~(0b11 << 0);	// Set Pin Mode for GPB.0(RX - Input)

	// UART0 operation configuration
	UART0->LCR |= (0b11 << 0); // 8 data bit
	UART0->LCR &= ~(1 << 2); // one stop bit	
	UART0->LCR &= ~(1 << 3); // no parity bit
	UART0->FCR |= (1 << 1); // clear RX FIFO
	UART0->FCR |= (1 << 2); // clear TX FIFO
	UART0->FCR &= ~(0xF << 16); // FIFO Trigger Level is 1 byte]
	
	//Baud rate config: BRD/A = 1, DIV_X_EN=0
	//--> Mode 0, Baud rate = UART_CLK/[16*(A+2)] = 22.1184 MHz/[16*(1+2)]= 115200 bps
	UART0->BAUD &= ~(0b11 << 28); // mode 0	
	UART0->BAUD &= ~(0xFFFF << 0);
	UART0->BAUD |= 10;
    
    // Enable UART0 receive interrupt
    UART0->IER |= 1 << 0;
    
    // Enable NVIC UART0 IRQ
    //NVIC_EnableIRQ(UART02_IRQn);
		NVIC->ISER[0] |= 1 << 12;
}

void UART02_IRQHandler(void) {
    uint8_t receivedData;
    
    // Check if interrupt is caused by received data available
    if (UART0->ISR & 1 << 8) {
			
        // Get data from RBR (Receive Buffer Register)
        receivedData = UART0->RBR;
        
        // Process the received data (for example, echo back)
			
        UART0->THR = receivedData;
			
			/*
				UART0->THR = UART0->RBR;
			*/
    }
    // Clear the interrupt flags
    UART0->ISR = ~(1 << 8);
}

void SYS_Init(void) {
    SYS_UnlockReg(); // Unlock protected registers

		// enable clock sources
		CLK->PWRCON |= (1 << 0);
		while(!(CLK->CLKSTATUS & (1 << 0)));

		//PLL configuration starts
		CLK->PLLCON &= ~(1<<19); //0: PLL input is HXT
		CLK->PLLCON &= ~(1<<16); //PLL in normal mode
		CLK->PLLCON &= (~(0x01FF << 0));
		CLK->PLLCON |= 48;
		CLK->PLLCON &= ~(1<<18); //0: enable PLLOUT
		while(!(CLK->CLKSTATUS & (1 << 2)));
		//PLL configuration ends

		// CPU clock source selection
		CLK->CLKSEL0 &= (~(0x07 << 0));
		CLK->CLKSEL0 |= (0x02 << 0);    
		//clock frequency division
		CLK->CLKDIV &= (~0x0F << 0);

		//UART0 Clock selection and configuration
		CLK->CLKSEL1 |= (0b11 << 24); // UART0 clock source is 22.1184 MHz
		CLK->CLKDIV &= ~(0xF << 8); // clock divider is 1
		CLK->APBCLK |= (1 << 16); // enable UART0 clock

		SYS_LockReg();  // Lock protected registers
}