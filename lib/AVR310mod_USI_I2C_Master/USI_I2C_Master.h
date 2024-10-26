/*
 * USI_I2C_Master.h
 *
 * Created: 2024/09/01 15:18:58
 *  Author: demicchi
 */ 

/*****************************************************************************
 * This is an implementation of a TWI(I2C) master using the USI.
 * Heavily modified from the original AVR310 example found at https://start.atmel.com/
*****************************************************************************/

#ifndef USI_I2C_MASTER_H_
#define USI_I2C_MASTER_H_

#ifndef F_CPU
#error "F_CPU should be set correctly in Hz based on the fuse configurations. Using -D option is recommended."
#endif
#include <avr/io.h>
#include <util/delay.h>

//********** Defines **********//
// Defines controlling timing limits
//#define TWI_FAST_MODE

#ifdef TWI_FAST_MODE                            // TWI FAST mode timing limits. SCL = 100-400kHz
#define DELAY_T2TWI (_delay_us(2)) // >1,3us
#define DELAY_T4TWI (_delay_us(1))  // >0,6us
#else                                           // TWI STANDARD mode timing limits. SCL <= 100kHz
#define DELAY_T2TWI (_delay_us(5)) // >4,7us
#define DELAY_T4TWI (_delay_us(4)) // >4,0us
#endif

// Defines controlling code generating
//#define SIGNAL_VERIFY

// USI_TWI messages and flags and bit masks
//#define SUCCESS   7
//#define MSG       0
/****************************************************************************
  Bit and byte definitions
****************************************************************************/
#define TWI_READ_BIT 0 // Bit position for R/W bit in "address byte".
#define TWI_ADR_BITS 1 // Bit position for LSB of the slave address bits in the init byte.
#define TWI_NACK_BIT 0 // Bit position for (N)ACK bit.

#define USI_TWI_NO_DATA 0x00           // Transmission buffer is empty
#define USI_TWI_DATA_OUT_OF_BOUND 0x01 // Transmission buffer is outside SRAM space
#define USI_TWI_UE_START_CON 0x02      // Unexpected Start Condition
#define USI_TWI_UE_STOP_CON 0x03       // Unexpected Stop Condition
#define USI_TWI_UE_DATA_COL 0x04       // Unexpected Data Collision (arbitration)
#define USI_TWI_NO_ACK_ON_DATA 0x05    // The slave did not acknowledge  all data
#define USI_TWI_NO_ACK_ON_ADDRESS 0x06 // The slave did not acknowledge  the address
#define USI_TWI_MISSING_START_CON 0x07 // Generated Start Condition not detected on bus
#define USI_TWI_MISSING_STOP_CON 0x08  // Generated Stop Condition not detected on bus

// Device dependant defines
#if defined(__AVR_AT90Mega169__) || defined(__AVR_ATmega169PA__) || defined(__AVR_AT90Mega165__)                       \
    || defined(__AVR_ATmega165__) || defined(__AVR_ATmega325__) || defined(__AVR_ATmega3250__)                         \
    || defined(__AVR_ATmega645__) || defined(__AVR_ATmega6450__) || defined(__AVR_ATmega329__)                         \
    || defined(__AVR_ATmega3290__) || defined(__AVR_ATmega649__) || defined(__AVR_ATmega6490__)
#define DDR_USI DDRE
#define PORT_USI PORTE
#define PIN_USI PINE
#define PORT_USI_SDA PORTE5
#define PORT_USI_SCL PORTE4
#define PIN_USI_SDA PINE5
#define PIN_USI_SCL PINE4
#endif
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_AT90Tiny26__) \
    || defined(__AVR_ATtiny26__)
#define DDR_USI DDRB
#define PORT_USI PORTB
#define PIN_USI PINB
#define PORT_USI_SDA PORTB0
#define PORT_USI_SCL PORTB2
#define PIN_USI_SDA PINB0
#define PIN_USI_SCL PINB2
#endif
#if defined(__AVR_AT90Tiny2313__) || defined(__AVR_ATtiny2313__)
#define DDR_USI DDRB
#define PORT_USI PORTB
#define PIN_USI PINB
#define PORT_USI_SDA PORTB5
#define PORT_USI_SCL PORTB7
#define PIN_USI_SDA PINB5
#define PIN_USI_SCL PINB7
#endif

//********** Prototypes **********//

void USI_TWI_Master_Initialise();
bool USI_TWI_Master_Stop();
bool USI_TWI_Start_Transmission(unsigned char, bool, unsigned char *);
inline bool USI_TWI_Start_Transmission_Write(unsigned char slave_addr) { return USI_TWI_Start_Transmission(slave_addr, false, 0); }
inline bool USI_TWI_Start_Transmission_Read(unsigned char slave_addr, unsigned char *buf) { return USI_TWI_Start_Transmission(slave_addr, true, buf); }
bool USI_TWI_Write(unsigned char *, unsigned char);
unsigned char USI_TWI_Read_Next_One_Byte();
void USI_TWI_Read_Finish();
unsigned char USI_TWI_Get_State_Info();
unsigned char USI_TWI_Master_Transfer(unsigned char);

#endif /* USI_I2C_MASTER_H_ */
