/*
 * main.c
 *
 *  Created on: Nov 8, 2015
 *      Author: jeffb
 *      Contributions from LarsRF GitHub project and Texas Instruments drivers
 *      Information on these can be found either by searching or
 *      viewing my GitHub page for this project at https://github.com/jeffrobots/senseRF
 */

/*
* This file is licensed under BSD and originally written by Texas Instruments, though
* a few elements have been adapted from LarsRF, an open source implementation of the TI drivers.
*
* This file handles the main control, state logic, initialization, and data handling of
* 	a simple wireless sensor network consisting of one node and one listener.
* 	The microcontroller begins by initializing all of the peripherals. It will then
* 	wait for either a user button press or a timer interrupt, at which point
* 	a template packet will be updated using a rolling counter enumeration,
* 	onboard temperature, and any other information that is needed.
*
* 	The uc will then pass this packet on to the tranceiver over SPI, which acts as a
* 	black box from this point on. The controller is then safe to return to a low
* 	power state.
*
* 	Upon receiving a packet, the radio wakes up the microcontroller, which
* 	reads the data and operates on said data. Note that this requires a constantly active radio,
* 	which is a source of a large amount of power drain on the system. Future efforts will include
* 	efforts to reduce power waste in general, this source of power drain included.
*
*
* For detailed descriptions of code function beyond associated comments,
* 	please consult GitHub for diagrams,	where you will also
* 	find issue/feature tracking and current project status.
*/


//#define interrupt(x) void __attribute__((interrupt (x)))
#define ACKLEN 4
#define MSGLEN 12
#define TEMPINDEX 3		// index location of temperature within stored data (before trim)
#define TEMPDATASIZE 10 // Allowed size of temperature buffer
#define PACKETNUM 4 	// index location of packet enumeration within stored data (Offset needed to account for trim)
#define RSSI_OFFSET 74 // RSSI offset taken from datasheet


#include "headers/include.h"
#include "uart.h"
#include "utils.h"

extern char paTable[];
extern char paTableLen;
volatile char sendDataFlag; 		// flag to determine when a new data pack should be send.
volatile char buttonPressed; 		// flag to determine when to start a transmission (on button press)
volatile char sendACK;				// Flag to determine when to send ACK

char txBuffer[MSGLEN];
char ackBuffer[ACKLEN] = {ACKLEN-1, 0x01, 0xFF, 0x00}; // acknowledgement message - Do not change
char rxBuffer[MSGLEN];
char RSSI_latest;
signed char RSSI_dbm;


void main (void)
{
	char tempData[TEMPDATASIZE] = {0}; // storage for temperature data
	unsigned int i;
	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

	// delay to compensate for radio startup
	__delay_cycles(5000);

	uartInit();
	TI_CC_SPISetup();                         // Initialize com protocols

	TI_CC_PowerupResetCCxxxx();               // Reset CCxxxx
	writeRFSettings();                        // Write RF settings to config reg
	TI_CC_SPIWriteBurstReg(TI_CCxxx0_PATABLE, paTable, paTableLen);//Write PATABLE -- See CC110L datasheet for more on these

	// Configure ports -- switch inputs, LEDs, GDO0 to RX packet info from CC110L
	TI_CC_SW_PxREN |= TI_CC_SW1;           	// Enable Pull up resistor (active low on switch)
	TI_CC_SW_PxOUT |= TI_CC_SW1;         	// Set to pullup (can change to active low by leaving it 0)
	TI_CC_SW_PxIES |= TI_CC_SW1;        	// Int on falling edge
	TI_CC_SW_PxIFG &= ~(TI_CC_SW1);         // Clr flags
	TI_CC_SW_PxIE |= TI_CC_SW1;            	// Enable port1 interrupts
	TI_CC_LED_PxOUT &= ~(TI_CC_LED1);		// Outputs = 0
	TI_CC_LED_PxDIR |= TI_CC_LED1;			// LED Direction to Outputs

	BCSCTL3 |= LFXT1S_2; 					// source ACKL from VLO (12KHz)
	// Enabling these lines below will configure the sensor to operate on a timer
	// Turn these off for debug/test
	TACCTL0 = CCIE;
	TACCR0 = 65535; // set delay at max amount for this timer configuration (~5 seconds)
					// delay can be extended by making use of other counting modes
	TACTL = TASSEL_1 + MC1;


	TI_CC_GDO0_PxIES |= TI_CC_GDO0_PIN;       // Int on falling edge (end of pkt)
	TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // Clear flag
	TI_CC_GDO0_PxIE |= TI_CC_GDO0_PIN;        // Enable int on end of packet

	TI_CC_SPIStrobe(TI_CCxxx0_SRX);           // Initialize CCxxxx in RX mode.
											// When a pkt is received, it will
                                            // signal on GDO0 and wake CPU
  // Build generic packet
    txBuffer[0] = MSGLEN-1;                        // Packet length
	txBuffer[1] = 0x01;                     // Packet address - If this is 0xFF, it's an ack and not data.
	// Begin data
	// ------
	txBuffer[2] = TI_CC_LED1;				// data to toggle LED1.
	txBuffer[3] = 0x00;					// temperature data
	txBuffer[4] = 0x00;					// packet number
	txBuffer[5] = 0x34;
	txBuffer[6] = 0x35;
	txBuffer[7] = 0x36;
	txBuffer[8] = 0x37;
	txBuffer[9] = 0x38;
	txBuffer[10] = 0x39;
	// ------
	// End Data
	txBuffer[11] = 0x00;					// terimate

  __bis_SR_register(LPM3_bits + GIE);       	// Enter LPM3, interrupts enabled

   	  while(1) {
  		  if (sendDataFlag || buttonPressed) {	// Service data transmission
  			txBuffer[TEMPINDEX] = readTemp();
  			for (i=TEMPDATASIZE; i>1; i--) {
  				tempData[i-1] = tempData[i-2];	// Cycle through and shift data through the buffer
  			}
  			tempData[0] = txBuffer[TEMPINDEX];	// Store new data in the first slot.
  			RFSendPacket(txBuffer, MSGLEN);
  			__delay_cycles(5000);
  			uart_printf("Temperature Sent: %i\r\n", txBuffer[TEMPINDEX]);
  			uart_printf("Packet #%i Sent.\r\n", txBuffer[PACKETNUM]);
  			txBuffer[PACKETNUM]++; // increment packet number
  			__delay_cycles(5000);
  			buttonPressed = 0; //
			sendDataFlag = 0; 					// clear associated flags
			__bis_SR_register(LPM3_bits + GIE); // re-enter LPM3 + interrupts
  		  }
  		  else if (sendACK) {
  			TI_CC_SPIReadBurstReg(TI_CC110_RSSI_STATUS, &RSSI_latest, sizeof(char)); // grab newest RSSI data
  			// RSSI is read as an offset 2s compliment.
  			// The line below normalizes\ to an absolute level in dBm
			RSSI_dbm = (RSSI_latest >> 1) - RSSI_OFFSET;//(RSSI_latest < 128) ? ((RSSI_latest/2) - RSSI_OFFSET) : ((RSSI_latest-256)/2 - RSSI_OFFSET);

  			uart_printf("Received Temperature: %i\r\n", rxBuffer[2]);// An example of what we want to show on serial
			uart_printf("Packet #%i Received. ", rxBuffer[PACKETNUM-1]);
			uart_printf("RSSI: -%i\r\n", RSSI_dbm);
  			__delay_cycles(5000);					// force a slight delay between receive and ack
  													// this seems to limit the few rare collision cases
  													// Though uart_printf should be slow enough to not need it.
  			  RFSendPacket(ackBuffer, ACKLEN);
  			  sendACK = 0;							// Clear ack flag
  			  __bis_SR_register(LPM3_bits + GIE);	// return to low power mode.
  		  }
  	  }
}

// Interrupt handler for TimerA0
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMERA0_ISR(void)
{
	_BIC_SR(LPM3_EXIT); 		// exit low power mode when we get an interrupt.
	sendDataFlag = 1; 			// set flag to send new data
}

// The ISR assumes the interrupt came from a pressed button
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR()
{
  // If Switch was pressed
  if(TI_CC_SW_PxIFG & TI_CC_SW1)
  {
	_BIC_SR(LPM3_EXIT); 				// exit low power mode
    buttonPressed = 1; 					// set flag for processing later on
    __delay_cycles(100000);				// Switch debounce
    TI_CC_SW_PxIFG &= ~(TI_CC_SW1);		// Clr flag that caused int and exit
  }
}

// CC110L received a packet (Port 2 interrupted on GDO0)
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR()
{
    // if GDO fired
  if(TI_CC_GDO0_PxIFG & TI_CC_GDO0_PIN)
  {
    char len=MSGLEN-1;                            //length should not include the first byte, which gets trimmed by CC110L
    _BIC_SR(LPM3_EXIT); 				// exit low power mode
    if (RFReceivePacket(rxBuffer,&len))
    {
        // Fetch packet from CCxxxx
        if (rxBuffer[1] != 0xFF)
        {
        	TI_CC_LED_PxOUT ^= rxBuffer[1];         // Toggle LEDs according to pkt data (if it is actually data)
        	// Send ACK
			sendACK = 1;
        }
        // ARQ would go here
     }
  }
  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // After pkt RX, this flag is set.
}
