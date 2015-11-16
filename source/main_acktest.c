/* 
* This file is licensed under BSD. It is originally copyright Texas Instruments, 
* but has been adapted by Lars Kristian Roland
*/

/*
* Put an LED between P1.4 and GND (or VCC). Press the button on the other board,
* and your LED should turn on and off. This demo implements an ACK packet that
* goes back to the recipient. (Both devices use the same address). 
*/

//#define interrupt(x) void __attribute__((interrupt (x)))
#define ACKLEN 4
#define MSGLEN 12


#include "include.h"
#include "uart.h"
#include "utils.h"

extern char paTable[];
extern char paTableLen;
volatile char sendDataFlag; // flag to determine when a new data pack should be send.

char txBuffer[MSGLEN];
char ackBuffer[ACKLEN] = {ACKLEN-1, 0x01, 0xFF, 0x00}; // acknowledgement message
char rxBuffer[MSGLEN];
unsigned int i = 0;

// Build packet
/*ackBuffer[0] = 3;                        	// Packet length not including this bit
ackBuffer[1] = 0x01;                     	// Packet address
ackBuffer[2] = 0xFF;						// acknowledgement
ackBuffer[3] = 0x00;						// termination*/


void main (void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  // 5ms delay to compensate for time to startup between MSP430 and CC1100/2500
  __delay_cycles(5000);
  
  uartInit();
  
  TI_CC_SPISetup();                         // Initialize SPI port

  TI_CC_PowerupResetCCxxxx();               // Reset CCxxxx
  writeRFSettings();                        // Write RF settings to config reg
  TI_CC_SPIWriteBurstReg(TI_CCxxx0_PATABLE, paTable, paTableLen);//Write PATABLE

  // Configure ports -- switch inputs, LEDs, GDO0 to RX packet info from CCxxxx
  TI_CC_SW_PxREN |= TI_CC_SW1;               // Enable Pull up resistor
  TI_CC_SW_PxOUT |= TI_CC_SW1;               // Enable pull up resistor
  TI_CC_SW_PxIES |= TI_CC_SW1;               // Int on falling edge
  TI_CC_SW_PxIFG &= ~(TI_CC_SW1);           // Clr flags
  TI_CC_SW_PxIE |= TI_CC_SW1;                // Activate interrupt enables
  TI_CC_LED_PxOUT &= ~(TI_CC_LED1); // Outputs = 0
  TI_CC_LED_PxDIR |= TI_CC_LED1;// LED Direction to Outputs

  BCSCTL3 |= LFXT1S_2; 					// source ACKL from VLO (12KHz
  TACCTL0 = CCIE;
  TACCR0 = 16400; // 1 second delay @ ACLK
  TACTL = TASSEL_1 + MC1;


  TI_CC_GDO0_PxIES |= TI_CC_GDO0_PIN;       // Int on falling edge (end of pkt)
  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // Clear flag
  TI_CC_GDO0_PxIE |= TI_CC_GDO0_PIN;        // Enable int on end of packet

  TI_CC_SPIStrobe(TI_CCxxx0_SRX);           // Initialize CCxxxx in RX mode.
                                            // When a pkt is received, it will
                                            // signal on GDO0 and wake CPU

  __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3, interrupts enabled

  	  while(1) {
  		  if (sendDataFlag) {
			txBuffer[0] = MSGLEN-1;                        // Packet length
		    txBuffer[1] = 0x01;                     // Packet address - If this is 0xFF, it's an ack and not data.
		    txBuffer[2] = TI_CC_LED1;				// data to toggle LED1.
		    txBuffer[3] = 0x32;
		    txBuffer[4] = 0x33;
		    txBuffer[5] = 0x34;
		    txBuffer[6] = 0x35;
		    txBuffer[7] = 0x36;
		    txBuffer[8] = 0x37;
		    txBuffer[9] = 0x38;
		    txBuffer[10] = 0x39;
		    txBuffer[11] = 0x00;
			RFSendPacket(txBuffer, 12);
			sendDataFlag = 0; // clear the flag
			__bis_SR_register(LPM3_bits + GIE); // re-enter LPM3 + interrupts
  		  }
  	  }
}

// Interrupt handler for TimerA0
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMERA0_ISR(void)
{
	_BIC_SR(LPM3_EXIT); // exit low power mode when we get an interrupt.
	sendDataFlag = 1; // set flag to send new data
}

// The ISR assumes the interrupt came from a pressed button
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR()
{
  
  // If Switch was pressed
  if(TI_CC_SW_PxIFG & TI_CC_SW1)
  {
  
    // Build packet
    txBuffer[0] = 11;                        // Packet length
    txBuffer[1] = 0x01;                     // Packet address - If this is 0xFF, it's an ack and not data.
    txBuffer[2] = TI_CC_LED1;				// data to toggle LED1.
    txBuffer[3] = 0x32;
    txBuffer[4] = 0x33;
    txBuffer[5] = 0x34;
    txBuffer[6] = 0x35;
    txBuffer[7] = 0x36;
    txBuffer[8] = 0x37;
    txBuffer[9] = 0x38;
    txBuffer[10] = 0x39;
    txBuffer[11] = 0x00;					// terimate
    
    // Using printf isn't very efficient, although if you configure your compiler,
    // http://www.43oh.com/forum/viewtopic.php?f=10&t=1732&hilit=tiny+printf

    //uart_printf("TX PKT:%i\r\n", txBuffer[3]);// An example of what we want to show on serial
          
    RFSendPacket(txBuffer, 12);              // Send value over RF
    __delay_cycles(5000);                   // Switch debounce
  }
  TI_CC_SW_PxIFG &= ~(TI_CC_SW1);           // Clr flag that caused int
}

// The ISR assumes the interrupt came from GDO0. GDO0 fires indicating that
// CCxxxx received a packet
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR()
{
    // if GDO fired
  if(TI_CC_GDO0_PxIFG & TI_CC_GDO0_PIN)
  {
    char len=11;                            // Len of pkt to be RXed (only addr
                                            // plus data; size byte not incl b/c
                                            // stripped away within RX function)
    if (RFReceivePacket(rxBuffer,&len))  
    {   
        // Fetch packet from CCxxxx
        if (rxBuffer[1] != 0xFF)
        {
        	TI_CC_LED_PxOUT ^= rxBuffer[1];         // Toggle LEDs according to pkt data (if it is actually data)
        	__delay_cycles(5000);					// force a slight delay between receive and ack (Remove this later as it should be instantaneous)
        	// Send ACK
			RFSendPacket(ackBuffer, ACKLEN);              // Send value over RF

			//uart_printf("RX PKT:%i\r\n", rxBuffer[2]);// An example of what we want to show on serial
			//uart_printf("TX ACK\r\n"); // Similar to printf, but printf probably takes longer
        }
        // Handle ARQ here
     }
  }

  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // After pkt RX, this flag is set.
}
