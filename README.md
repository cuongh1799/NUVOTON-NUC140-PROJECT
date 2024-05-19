# NUVOTON NUC140 PROJECT
 
# Question 1:
Using a USB-UART Adapter as a bridge between the PC's Terminal and NUC140 Board, adjust so that
- Clock Frequency at 50 MHz
- UART clock source is 22.1184 MHz
- UART0 channel is used to transmit data and receive data.
- Data packet: 1 start bit + 8 data bit + no parity bit + 1 stop bit
- Baud rate: 115200 bps
The program should able to allow the Terminal to send messages to the NUC140 through the Terminal, thus allowing the NUC140 to receive and resend the message to the Terminal.

Implementation:
- We're going to use interrupt to detect any changes in the RX.
- If the RX is not empty, this means there's a pending message.
- What we're going to do is receive that message through the `RBR` register, and send through `THR` register.

# Question 2:
Using the NUC140 and its AD converter, create a program that allow the NUC140 to send messages only when its higher / lower than specific Voltage
- Clock for the CPU is at 50 MHz
- ADC channel 7 (12-bit A/D converter with reference voltage, Vref = 3.3 V ) is used to continuously sample analog voltage at GPIO port A pin 7 and convert it into a corresponding digital value
- ADC Clock Frequency at 50 MHz, continuous scan mode
- SPI Serial Clock at 1 MHz, Idle high state, data transmit at rising edge, sending LSB first, one byte between transmition
- Using pin D0, D1, D3.

Implementation:
- We're going to calculate the Voltage input as we already know the Reference Voltage (3.3V), Resolution.
- From this we're going to send data through SPI3 to display on the LCD
