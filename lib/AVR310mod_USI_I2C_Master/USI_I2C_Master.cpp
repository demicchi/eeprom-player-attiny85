/*
 * USI_I2C_Master.cpp
 *
 * Created: 2024/09/01 15:19:11
 *  Author: demicchi
 */ 

/*****************************************************************************
 * This is an implementation of a TWI(I2C) master using the USI.
 * Heavily modified from the original AVR310 example found at https://start.atmel.com/
*****************************************************************************/

#include <avr/io.h>

#include "USI_I2C_Master.h"

unsigned char tempUSISR_8bit = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC)
                                |                 // Prepare register value to: Clear flags, and
                                (0x0 << USICNT0); // set USI to shift 8 bits i.e. count 16 clock edges.
unsigned char tempUSISR_1bit = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC)
                                |                 // Prepare register value to: Clear flags, and
                                (0xE << USICNT0); // set USI to shift 1 bit i.e. count 2 clock edges.
unsigned char USI_TWI_errorState;


/*---------------------------------------------------------------
 USI TWI single master initialization function
---------------------------------------------------------------*/
void USI_TWI_Master_Initialise()
{
    PORT_USI |= (1 << PIN_USI_SDA); // Enable pullup on SDA, to set high as released state.
    PORT_USI |= (1 << PIN_USI_SCL); // Enable pullup on SCL, to set high as released state.

    DDR_USI |= (1 << PIN_USI_SCL); // Enable SCL as output.
    DDR_USI |= (1 << PIN_USI_SDA); // Enable SDA as output.

    USIDR = 0xFF;                                           // Preload data register with "released level" data.
    USICR = (0 << USISIE) | (0 << USIOIE) |                 // Disable Interrupts.
            (1 << USIWM1) | (0 << USIWM0) |                 // Set USI in Two-wire mode.
            (1 << USICS1) | (0 << USICS0) | (1 << USICLK) | // Software strobe as counter clock source
            (0 << USITC);
    USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | // Clear flags,
            (0x0 << USICNT0);                                             // and reset counter.
}

/*---------------------------------------------------------------
Use this function to get hold of the error message from the last transmission
---------------------------------------------------------------*/
unsigned char USI_TWI_Get_State_Info()
{
    return (USI_TWI_errorState); // Return error state.
}


/*---------------------------------------------------------------
 Core function for shifting data in and out from the USI.
 Data to be sent has to be placed into the USIDR prior to calling
 this function. Data read, will be returned from the function.
---------------------------------------------------------------*/
unsigned char USI_TWI_Master_Transfer(unsigned char temp)
{
    USISR = temp;                                          // Set USISR according to temp.
                                                           // Prepare clocking.
    temp = (0 << USISIE) | (0 << USIOIE) |                 // Interrupts disabled
           (1 << USIWM1) | (0 << USIWM0) |                 // Set USI in Two-wire mode.
           (1 << USICS1) | (0 << USICS0) | (1 << USICLK) | // Software clock strobe as source.
           (1 << USITC);                                   // Toggle Clock Port.
    do {
        DELAY_T2TWI;
        USICR = temp; // Generate positive SCL edge.
        while (!(PIN_USI & (1 << PIN_USI_SCL)))
            ; // Wait for SCL to go high.
        DELAY_T4TWI;
        USICR = temp;                   // Generate negative SCL edge.
    } while (!(USISR & (1 << USIOIF))); // Check for transfer complete.

    DELAY_T2TWI;
    temp  = USIDR;                 // Read out data.
    USIDR = 0xFF;                  // Release SDA.
    DDR_USI |= (1 << PIN_USI_SDA); // Enable SDA as output.

    return temp; // Return the data from the USIDR
}

/*---------------------------------------------------------------
 Function for generating a TWI Stop Condition. Used to release
 the TWI bus.
---------------------------------------------------------------*/
bool USI_TWI_Master_Stop()
{
    PORT_USI &= ~(1 << PIN_USI_SDA); // Pull SDA low.
    PORT_USI |= (1 << PIN_USI_SCL);  // Release SCL.
    while (!(PIN_USI & (1 << PIN_USI_SCL)))
        ; // Wait for SCL to go high.
    DELAY_T4TWI;
    PORT_USI |= (1 << PIN_USI_SDA); // Release SDA.
    DELAY_T2TWI;

#ifdef SIGNAL_VERIFY
    if (!(USISR & (1 << USIPF))) {
        USI_TWI_errorState = USI_TWI_MISSING_STOP_CON;
        return false;
    }
#endif

    return true;
}



bool USI_TWI_Start_Transmission(unsigned char slave_addr, bool is_read, unsigned char *buf)
{
    USI_TWI_errorState = 0;
    
    slave_addr = (slave_addr << TWI_ADR_BITS) | ((is_read ? 1 : 0) << TWI_READ_BIT);
    
    /* Release SCL to ensure that (repeated) Start can be performed */
    PORT_USI |= (1 << PIN_USI_SCL); // Release SCL.
    while (!(PIN_USI & (1 << PIN_USI_SCL))); // Verify that SCL becomes high.
    #ifdef TWI_FAST_MODE
    DELAY_T4TWI; // Delay for T4TWI if TWI_FAST_MODE
    #else
    DELAY_T2TWI; // Delay for T2TWI if TWI_STANDARD_MODE
    #endif

    /* Generate Start Condition */
    PORT_USI &= ~(1 << PIN_USI_SDA); // Force SDA LOW.
    DELAY_T4TWI;
    PORT_USI &= ~(1 << PIN_USI_SCL); // Pull SCL LOW.
    PORT_USI |= (1 << PIN_USI_SDA);  // Release SDA.

    /* Write address */
    PORT_USI &= ~(1 << PIN_USI_SCL);         // Pull SCL LOW.
    USIDR = slave_addr;
    USI_TWI_Master_Transfer(tempUSISR_8bit); // Send 8 bits on bus.

    /* Clock and verify (N)ACK from slave */
    DDR_USI &= ~(1 << PIN_USI_SDA); // Enable SDA as input.
    if (USI_TWI_Master_Transfer(tempUSISR_1bit) & (1 << TWI_NACK_BIT)) {
        USI_TWI_errorState = USI_TWI_NO_ACK_ON_ADDRESS;
        return false;
    }
    
    if (is_read) {
        /* Read a data byte */
        DDR_USI &= ~(1 << PIN_USI_SDA); // Enable SDA as input.
        *buf = USI_TWI_Master_Transfer(tempUSISR_8bit);
    }        
    
    /* Transmission successfully completed*/
    return true;
}

bool USI_TWI_Write(unsigned char *msg, unsigned char msgSize)
{
    do {
        /* Write a byte */
        PORT_USI &= ~(1 << PIN_USI_SCL);         // Pull SCL LOW.
        USIDR = *(msg++);                        // Setup data.
        USI_TWI_Master_Transfer(tempUSISR_8bit); // Send 8 bits on bus.

        /* Clock and verify (N)ACK from slave */
        DDR_USI &= ~(1 << PIN_USI_SDA); // Enable SDA as input.
        if (USI_TWI_Master_Transfer(tempUSISR_1bit) & (1 << TWI_NACK_BIT)) {
            USI_TWI_errorState = USI_TWI_NO_ACK_ON_DATA;
            return false;
        }
    } while (--msgSize); // Until all data sent/received.

    /* Transmission successfully completed*/
    return true;
}


unsigned char USI_TWI_Read_Next_One_Byte()
{
    USIDR = 0x00; // Load ACK. Set data register bit 7 (output for SDA) low.
    USI_TWI_Master_Transfer(tempUSISR_1bit); // Generate ACK.

    /* Read a data byte */
    DDR_USI &= ~(1 << PIN_USI_SDA); // Enable SDA as input.
    return USI_TWI_Master_Transfer(tempUSISR_8bit);
}

void USI_TWI_Read_Finish()
{
    USIDR = 0xFF; // Load NACK to confirm End Of Transmission.
    USI_TWI_Master_Transfer(tempUSISR_1bit); // Generate NACK.
}

