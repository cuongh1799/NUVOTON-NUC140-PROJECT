
#include <stdio.h>
#include "NUC100Series.h"
#include "LCD.h"
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "MCU_init.h"
#include "SYS_init.h"

#define TMR0_COUNTS 100000
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
void UART02_IRQHandler(void);
void UART0_Init(void);
int byteToInt(unsigned char byte);

void clr_segment(void);
void show_segment(unsigned char n, unsigned char number);
void TIMER0_Config(void);
static int TimePassed = 0;			// for tmr interrupt
void GPIO_Config(void);

void checkState(int state);
int getMatrixKey(void);
void ScanAndDisplay(void);
void hitScan(void); // scan if hit
void appendChar(char *str, char c);


int countShipSpot(void);
static int WinCon = 0;
static char WinCon_ch[4] = "0000"; 

extern unsigned char amogus[32*32] = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
0xE0,0xF0,0xF0,0xF8,0xF8,0xF8,0xF8,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF3,0xF3,0xE3,0xE3,0xE1,0xE1,0xE1,0xE3,0xE3,0xE3,0xE3,0xE2,0xF4,0xBC,0x18,
0x07,0x1F,0x1F,0x1F,0x1F,0x1F,0x3F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x3F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x1F,0x03,0x00,
0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x01,0x01,0x01,0x01,0x01,0x00};

static int shots = 15;
static char shots_ch[4] = "0000";

/*
----------------------------------------------------------------------------------------------------------------------------------------------------------------
-----------------------MAP VARIABLE-----------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------------------------------
`map_display` is the map that shows on the LCDs
`map_ship` is the map that is use for scanning the hits
when hits, change the hit spot to 1 in the `map_display`
*/

	
static int map_display[8][8]={
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0}
};

// Both of this is use for the UART transfering... things
// `uploadIndexIndex` should be Y, `uploadIndex` should be X
static int uploadIndex= 0;
static int uploadIndexIndex = 0;

static int map_ship[8][8]={
{1,0,0,0,0,0,0,0},
{1,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0}
};

static int score = 0;
static char score_ch[1] = "0";

/*
0 = menu
1 = map (playing)
2 = end
*/
static int state = 0;
static char state_ch[4] = "0000";
static int mapRow = 2;
static int mapCol = 0;

/*
BECAUSE IN THE GAMEPLAY, WE SEE THE COLUMN AND ROW STARTS WITH `1`
HOWEVER, IN THE SYSTEM ARRAY `map_ship` AND `map_display` THE COLUMN AND ROW STARTS WITH 0
THEREFORE WHEN HITSCAN THE COORDINATE, WE NEED TO -1 BOTH OF THE COORDINATE, HENCE THE STARTING POSITION IS 1 INSTEAD 0
*/

static int currentX = 1; // To show the user current x or y
static int currentY = 1;
static char currentX_ch[1] = "1";
static char currentY_ch[1] = "1";

static int bullets = 0;

/*
----------------------------------------------------------------------------------------------------------------------------------------------------------------
-----------------------KEYPRESS MATRIX-----------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

static int keypress; // to store the matrix 3x3 key presses
static char keypress_ch[4] = "0000";
static int coordinateMode = 0; // x for 0, y for 1
static char coordinateModeName[2][2] = {"X", "Y"};
static int coordinateFlag = 3; // represent the 7segment position

/*
----------------------------------------------------------------------------------------------------------------------------------------------------------------
-----------------------TIMER------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

static int segmentBit = 0;

/*
----------------------------------------------------------------------------------------------------------------------------------------------------------------
-----------------------MAIN-------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
int main(){
	
	System_config();
	SPI3_config();
	
	LCD_start();
	LCD_clear();
	GPIO_Config();
	UART0_Init();
	
	PB->PMD &= (~(0x03<<30)); // enable GPB15 (the button)
	
	PC->DOUT |= (1<<7);     
	PC->DOUT &= ~(1<<6);		
	PC->DOUT &= ~(1<<5);		
	PC->DOUT &= ~(1<<4);	

	WinCon = countShipSpot();
	sprintf(WinCon_ch, "%d", countShipSpot());	
	
	TIMER0_Config();
	

	while(1){		
		ScanAndDisplay();
		checkState(state);
		CLK_SysTickDelay(1000);
	}
}

/*
----------------------------------------------------------------------------------------------------------------------------------------------------------------
-----------------------CLOCK CONFIG-----------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

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
	
	/*
	//ADC Clock selection and configuration
	CLK->CLKSEL1 &= ~(0x03 << 2); // ADC clock source is 12 MHz
	CLK->CLKDIV &= ~(0x0FF << 16);
	CLK->CLKDIV |= (0x0B << 16); // ADC clock divider is (11+1) --> ADC clock is 12/12 = 1 MHz
	CLK->APBCLK |= (0x01 << 28); // enable ADC clock
	*/
	
	//UART0 Clock selection and configuration
	CLK->CLKSEL1 &= ~(0b11 << 24);
	CLK->CLKSEL1 |= (0b11 << 24); // 22. something
	//CLK->CLKSEL1 |= (0b01 << 24); // UART0 clock source is PLLsource
	CLK->CLKDIV &= ~(0xF << 8); // clock divider is 1
	CLK->APBCLK |= (1 << 16); // enable UART0 clock
	
	SYS_LockReg();  // Lock protected registers 
}

/*
----------------------------------------------------------------------------------------------------------------------------------------------------------------
-----------------------UART0 config-----------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

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

/*
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
-------------------------Timer0 configuration-----------------------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

void TIMER0_Config(void){
	CLK->CLKSEL1 &= ~(0x111 << 8);	
	CLK->CLKSEL1 |= (0b010 << 8);			// 12MHz
	CLK->APBCLK |= (1 << 2);					// enable timer0
	// Set prescaler
	TIMER0->TCSR &= ~(0xFF << 0);			// reset timer0
	// Opearting mode
	TIMER0->TCSR |= (1 << 26);				// Reset Timer0
	TIMER0->TCSR &= ~(0b11 << 27);		// one-shot mode
	TIMER0->TCSR |= (0b01 << 27);			// timer operate at periodic mode
	TIMER0->TCSR &= ~(1 << 24);			  // no effect
	//TIMER0->TCSR |= (1 << 16);				// TDR is updated continuously while counter is counting
	// Value
	TIMER0->TCMPR = TMR0_COUNTS;			// 
	TIMER0->TCSR |= (1 << 29);				// Enable timer interrupt flag TIF	
	TIMER0->TCSR |= (1 << 30);				// Start counting
	
	//Set Timer0 in NVIC Set-Enable Control Register (NVIC_ISER)
	NVIC->ISER[0] |= 1 << 8;
	// CONFIG CLOCK, OPERATING MODE, AND VALUE FOR TIMER0 ENDS-------
}

// Timer0 interrupt - for time transition each 7Segment LED to avoid bouncing
void TMR0_IRQHandler(void){

	clr_segment(); 
	TIMER0->TISR |= (1<<0);		// generate interrupt

	switch(segmentBit){
		case 0:
			if(bullets < 10){
				show_segment(0, bullets);
			}
			else if(bullets >= 10 && bullets < 16){
				show_segment(0, bullets % 10);
			}
			segmentBit++;
			break;
		case 1:
			clr_segment();
			if(bullets >= 10){
				show_segment(1, 1);
			}
			segmentBit++;
			break;
		case 2:
			clr_segment();
			if(coordinateMode == 0){
				show_segment(coordinateFlag ,currentX);
			}
			else if(coordinateMode == 1){
				PE->DOUT |= (1<<7);		//segment g
				PE->DOUT |= (1<<5);		//segment d
				PE->DOUT |= (1<<4);	//segment b
				PE->DOUT |= (1<<3);		//segment a
				PE->DOUT |= (1<<2);		//segment f
				PE->DOUT |= (1<<1);	//DOT
				PE->DOUT |= (1<<0);		// segment c
			}
			segmentBit++;
			break;
		case 3:
			clr_segment();
			if(coordinateMode == 1){
				show_segment(coordinateFlag ,currentY);
			}
			else if(coordinateMode == 0){
				PE->DOUT |= (1<<7);		//segment g
				PE->DOUT |= (1<<5);		//segment d
				PE->DOUT |= (1<<4);	//segment b
				PE->DOUT |= (1<<3);		//segment a
				PE->DOUT |= (1<<2);		//segment f
				PE->DOUT |= (1<<1);	//DOT
				PE->DOUT |= (1<<0);		// segment c
			}
			segmentBit = 0;
			break;
	}
	
	TIMER0->TISR |= (1<<0);		// Clear the flag by writing 1 to it
}


/*
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
----------------------------SPI3--------------------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

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

/*
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
-----------------------------------------LCD---------------------------------------------------------------------------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

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

/*
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
----------------------------7SEGMENT-----------------------------------------------------------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
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

void ScanAndDisplay(void){
		keypress = getMatrixKey();
		
		if(keypress == 9){
			/*
			0 = x
			1 = y
			*/
			if(coordinateMode == 0){
				coordinateMode = 1;
				coordinateFlag = 2;
				clr_segment();
				CLK_SysTickDelay(4500);
				//show_segment(coordinateFlag ,0);
			}
			else if(coordinateMode == 1){
				coordinateMode = 0;
				coordinateFlag = 3;
				clr_segment();
				CLK_SysTickDelay(4500);
				//show_segment(coordinateFlag ,0);
			}
		}
		if(keypress != 0 && keypress != 9){
			//keypress = lastkeypress;
			clr_segment();
			//show_segment(coordinateFlag ,keypress);
			if(coordinateMode == 0){
				currentX = keypress;
				sprintf(currentX_ch, "%d", currentX);
			}
			else if(coordinateMode == 1){
				currentY = keypress;
				sprintf(currentY_ch, "%d", currentY);
			}
		}
}

/*
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
-------------------------------GPB15 AND KEYMATRIX------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void GPIO_Config(void)	{
	// SET PA 0-2 TO INPUT PULL UP (IMPORTANT), 3-5 TO OUTPUT
	/*
	PA->PMD &= (~(0x03 << 0)); 
	PA->PMD &= (~(0x03 << 2)); 
	PA->PMD &= (~(0x03 << 4)); 
	
	PA->PMD &= (~(0x03 << 6)); // 3
	PA->PMD &= (~(0x03 << 8)); // 4
	PA->PMD &= (~(0x03 << 10)); // 5
	PA->PMD &= ((0x01 << 6));
	PA->PMD &= ((0x01 << 8));
	PA->PMD &= ((0x01 << 10));
	*/
	
	GPIO_SetMode(PA, BIT0, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT1, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT2, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT3, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT4, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT5, GPIO_MODE_QUASI);
	
	PC->PMD &= (~(0x03 << 12)); // LED hit
	PC->PMD |= (0x01 << 12); // LED hit
	
	PB->PMD &= (~(0x03 << 30)); // Input
	PB->IMD &= (~(1 << 15)); // Detect edge-trigger interrupt 
	PB->IEN |= (1 << 15); // falling edge-trigger
	
	NVIC->ISER[0] |= 1 << 3; // Turn on interrupt
	NVIC->IP[0] &= (~(3 << 30)); // Set priority
	
	// Debounce configuration
	PB->DBEN |= (1<<15); // Initial Debounce
	PA->DBEN |= (1<<0); // Initial Debounce
	PA->DBEN |= (1<<2); // Initial Debounce
	PA->DBEN |= (1<<4); // Initial Debounce
	PA->DBEN |= (1<<6); // Initial Debounce
	PA->DBEN |= (1<<8); // Initial Debounce
	PA->DBEN |= (1<<10); // Initial Debounce
	GPIO->DBNCECON &= ~(0xF << 0);
	GPIO->DBNCECON |= (0b0111 << 0); // Sample interrupt input once per 128 clocks: 12MHz/128 = 93.75kHz
	GPIO->DBNCECON |= (1<<4); // Debounce counter clock source is the internal 10kHz low speed oscillator 
	
}

int getMatrixKey(void){
	/*
	The idea of this is to scan the rows and column
	*/
		PA0 = 1; PA1 = 1; PA2 = 0; PA3 = 1; PA4 = 1; PA5 = 1;
	if (PA3 == 0) return 1;
	if (PA4 == 0) return 4;
	if (PA5 == 0) return 7;
	PA0 = 1; PA1 = 0; PA2 = 1; PA3 = 1; PA4 = 1; PA5 = 1;
	if (PA3 == 0) return 2;
	if (PA4 == 0) return 5;
	if (PA5 == 0) return 8;
	PA0 = 0; PA1 = 1; PA2 = 1; PA3 = 1; PA4 = 1; PA5 = 1;
	if (PA3 == 0) return 3;
	if (PA4 == 0) return 6;
	if (PA5 == 0) return 9;
	return 0;
}
/*
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------INTERRUPT & SHOOTING----------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

void EINT1_IRQHandler(void) {

	switch(state){
		case 0:
			state = 1;
			LCD_clear();
			break;
		case 1:
			if(bullets >= 16){
				state = 3;
				break;
			}
			if(bullets < 16){
				hitScan();
				bullets++;
				if(score == WinCon){
					state = 2;
				}
				
				break;
			}
			
	}
	
	PB->ISRC |= (1 << 15);
}

void hitScan(void){
	if(map_ship[currentY-1][currentX-1] == 1){
		// Check if the position on `map_display` already changed/shot before
		if(map_display[currentY-1][currentX-1] != 1){
			map_display[currentY-1][currentX-1] = 1;
			score++;
			sprintf(score_ch, "%d", score);
			for(int i = 0; i < 6; i++){
				CLK_SysTickDelay(10000);
				PC->DOUT ^= 1 << 12;
			}
		}
	}
}

void UART02_IRQHandler(void) {
    uint8_t receivedData;
    
    // Check if interrupt is caused by received data available
    if (UART0->ISR & 1 << 8) {
		
			// Get data from RBR (Receive Buffer Register)
			receivedData = UART0->RBR;		
			UART0->THR = receivedData;
    }
    // Clear the interrupt flags
    UART0->ISR = ~(1 << 8);
}

/*
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
-------------------------------GAMELOGIC----------------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
int countShipSpot(void){
	int counting = 0;
	for (int i=0; i<8;i++){
		for (int j=0; j<8;j++){
			if(map_ship[i][j] == 1){
				counting++;
			}
		}
	}
	return counting;
}

void checkState(int state){
	if(state == 0){
		goto menu;
	}
	else if(state == 1){
		goto map;
	}
	else if(state == 2){
		goto win;
	}
	else if(state == 3){
		goto lose;
	}

	menu:
				printS_5x7(4, 10, "Battleship");
				printS_5x7(4, 18, "By hehe");
				//printS_5x7(100, 26, state_ch);
				draw_Bmp32x32(4, 30, 1, 0, amogus); 
				CLK_SysTickDelay(1000000);
				return;
	
	map:
		//CLK_SysTickDelay(500);	
		//printS_5x7(67, 12, "Objects: ");
		//printS_5x7(120, 12, WinCon_ch);
		//printS_5x7(67, 12, strcat("Objects: ", WinCon_ch));
	
		printS_5x7(67, 20, "Score:");
		printS_5x7(100, 20, score_ch);
	
		printS_5x7(67, 32, "Current x:");
		printS_5x7(120, 32, currentX_ch);
	
		printS_5x7(67, 40, "Current y:");
		printS_5x7(120, 40, currentY_ch);
	
		for (int i=0; i<8;i++){
			for (int j=0; j<8;j++)
			{
				if (map_display[i][j] == 0) {
					printS_5x7(mapRow, mapCol, "-");	// Display "0" = "-"
				}
				else if(map_display[i][j] == 1){
					printS_5x7(mapRow, mapCol, "x");	// Display "1" = "X"
				}
				mapRow = mapRow+8;
			}
			mapCol = mapCol+8;
			mapRow = 2;
		}
		return;
	win:
		CLK_SysTickDelay(10000);
		clear_LCD();
		clr_segment();
		printS_5x7(20, 32, "gg you win");
		return;
	lose:
		CLK_SysTickDelay(10000);
		clear_LCD();
		clr_segment();
		printS_5x7(20, 32, "gg you lost");
		return;
}

int byteToInt(unsigned char byte) {
    return (int)byte; // Typecast byte to int
}

void appendChar(char *str, char c) {
    int len = strlen(str); // Find the length of the string
    str[len] = c; // Append the character
    str[len + 1] = '\0'; // Add the null terminator
}