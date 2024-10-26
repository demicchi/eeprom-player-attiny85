/*
* eeprom-player.cpp
*
* Created: 2024/08/30 0:05:26
* Author : demicchi
*/

// 16MHz
// F_CPU should be defined as a compiler option (-DF_CPU=16000000UL)
//#define F_CPU 16000000


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <string.h>

#include "lib/AVR310mod_USI_I2C_Master/USI_I2C_Master.h"

/************************************************************************
 * Configuration Begin
 ************************************************************************/

#define SOUND_TABLE_MAX_ATTEMPT 8 // how many table data to read for each EEPROM
#define SOUND_ID_MAX 8 // how many sounds to define
#define SOUND_BANK_MAX 8 // how many EEPROMs to support
#define SOUND_QUEUE_LEN 8 // how long the sounds can wait for being played
#define LONG_RUN_TIME 30UL // how many seconds to count as a long run
#define DELAY_ACC_START 5UL // how many seconds to wait to play sounds after ACC power becomes ON
#define DELAY_ACC_STOP 3UL // how many seconds to wait to play sounds after ACC power becomes ON
volatile uint8_t sound_bank[SOUND_BANK_MAX] = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57}; // EEPROM addresses for each bank_id's starting from 0
static uint8_t sound_acc_start[] = {1}; // sound_id's to play when ACC power becomes ON
static uint8_t sound_acc_stop[] = {3}; // sound_id's to play when ACC power becomes OFF
static uint8_t sound_long_run[] = {2}; // sound_id's to play when ACC power is ON for a long time

/************************************************************************
 * Configuration End
 ************************************************************************/

volatile uint8_t wait_count_sound = 0;
volatile uint8_t wait_count_second = 0;
volatile uint16_t running_time = 0;
volatile uint8_t sound_id = 0;
volatile bool load_next_byte = false;
volatile unsigned char sound_byte = 0x00;
volatile uint16_t bytes_loaded = 0;
volatile uint8_t sound_queue[SOUND_QUEUE_LEN];
volatile uint8_t sound_queue_pos = 0;
volatile bool acc_stop_detected = false;
volatile bool long_run_detected = false;
volatile uint8_t wait_count_acc_stop = 0;

volatile struct SoundTable {
    uint8_t bank;
    uint16_t addr;
    uint16_t len;
} sound_table[SOUND_ID_MAX + 1];

void wdt_init() __attribute__((naked)) __attribute__((section(".init3")));

void initializeSoundQueue();
bool addToSoundQueue(uint8_t);
void addListToSoundQueue(uint8_t *, uint8_t);
uint8_t pickFromSoundQueue();

void readMemory(uint8_t, uint16_t, unsigned char *, uint8_t);

void scanSoundBank();
void readSoundTable();

void startSoundInQueue();
bool playSound(uint8_t);
void loadSoundByte();
void stopSound();

bool checkAccStop();
void checkLongRun();

void doProcedureForAccStart();
void doProcedureForAccStop();
void doProcedureForLongRun();

void shutdown();


ISR(TIM0_OVF_vect) {
    // Note: the clock is 16 MHz, the counter is 8bit (0..255)
    if (++wait_count_sound >= 8) { // drop down to 8000 Hz (16 MHz / 256 / 8 = 7812.5 Hz)
        wait_count_sound = 0;
        if (sound_id == 0) {
            OCR0B = 0x00;
        } else {
            if (!load_next_byte) {
                OCR0B = sound_byte;
                load_next_byte = true;
            }
        }
    }
}

ISR(TIM1_OVF_vect) {
    // Note: the clock is 16 MHz, prescaled by 16384, the counter is 8bit (0..255)
    if (++wait_count_second >= 4) { // one second has passed (16 MHz / 16384 / 256 / 4 = 0.9537 Hz ~ 1s)
        wait_count_second = 0;
        if (++running_time >= LONG_RUN_TIME) {
            running_time = 0;
            long_run_detected = true;
        }
        if (wait_count_acc_stop > 0) {
            wait_count_acc_stop--;
        }
    }
}


void initializeSoundQueue()
{
    memset((uint8_t *)sound_queue, 0x00, sizeof(sound_queue));
    sound_queue_pos = 0;
}

bool addToSoundQueue(uint8_t data)
{
    uint8_t index = sound_queue_pos;
    
    for (uint8_t i = 0; i < SOUND_QUEUE_LEN; i++) {
        if (sound_queue[index] == 0x00) {
            sound_queue[index] = data;
            return true;
        }
        if (index >= SOUND_QUEUE_LEN - 1) {
            index = 0;
        } else {
            index++;
        }
    }
    // Queue is occupied
    return false;
}

void addListToSoundQueue(uint8_t *data, uint8_t size)
{
    while (size--) {
        addToSoundQueue(*(data++));
    }
}

uint8_t pickFromSoundQueue()
{
    uint8_t picked_data = sound_queue[sound_queue_pos];
    
    if (picked_data == 0x00) {
        return picked_data;
    }
    
    sound_queue[sound_queue_pos] = 0x00;
    if (sound_queue_pos >= SOUND_QUEUE_LEN - 1) {
        sound_queue_pos = 0;
    } else {
        sound_queue_pos++;
    }
    return picked_data;
}

void readMemory(uint8_t device_addr, uint16_t memory_addr, unsigned char *buf, uint8_t len) {
    unsigned char addr[2];
    
    addr[0] = memory_addr >> 8;
    addr[1] = memory_addr & 0xff;
    
    USI_TWI_Start_Transmission_Write(device_addr);
    USI_TWI_Write(addr, 2);
    
    USI_TWI_Start_Transmission_Read(device_addr, buf);

    while (--len > 0) {
        *(++buf) = USI_TWI_Read_Next_One_Byte();
    }
    USI_TWI_Read_Finish();
    
    USI_TWI_Master_Stop();
}


void scanSoundBank()
{
    uint8_t index = 0;
    
    do {
        if (sound_bank[index] == 0x00) {
            continue;
        }
        if (!USI_TWI_Start_Transmission_Write(sound_bank[index])) {
            // EEPROM not found
            sound_bank[index] = 0x00;
        }
        USI_TWI_Master_Stop();
    } while (++index < SOUND_BANK_MAX);
}

void readSoundTable() {
    unsigned char buf[3];
    uint8_t index;
    uint8_t bank_id = 0;
    uint8_t id;
    
    for (index = 0; index <= SOUND_ID_MAX; index++) {
        sound_table[index].bank = 0x00;
        sound_table[index].addr = 0x00;
        sound_table[index].len = 0x00;
    }
    
    do {
        if (sound_bank[bank_id] == 0x00) {
            continue;
        }
        index = 0;
        do {
            memset(buf, 0, sizeof(buf));
            readMemory(sound_bank[bank_id], 8 * index, buf, 2);
            if (buf[0] == 0x00 && buf[1] == 0x00) {
                break;
            }
            if (buf[0] == 0x00 && buf[1] >= 1 && buf[1] <= SOUND_ID_MAX) {
                id = buf[1];
                sound_table[id].bank = bank_id;
                memset(buf, 0, sizeof(buf));
                readMemory(sound_bank[bank_id], 8 * index + 2, buf, 3);
                sound_table[id].addr = (buf[1] << 8) | (buf[2]);
                memset(buf, 0, sizeof(buf));
                readMemory(sound_bank[bank_id], 8 * index + 5, buf, 3);
                sound_table[id].len = (buf[1] << 8) | (buf[2]);
                break;
            }
        } while (++index < SOUND_TABLE_MAX_ATTEMPT);
    } while (++bank_id < SOUND_BANK_MAX);
}

void startSoundInQueue()
{
    uint8_t id = pickFromSoundQueue();
    if (id == 0x00) {
        return;
    }
    playSound(id);
}

bool playSound(uint8_t id)
{
    unsigned char addr[2];
    
    stopSound();
    if (id == 0 || id > SOUND_ID_MAX || sound_table[id].len == 0) {
        return false;
    }

    addr[0] = sound_table[id].addr >> 8;
    addr[1] = sound_table[id].addr & 0xff;
    USI_TWI_Start_Transmission_Write(sound_bank[sound_table[id].bank]);
    USI_TWI_Write(addr, 2);
    USI_TWI_Start_Transmission_Read(sound_bank[sound_table[id].bank], (unsigned char*)&sound_byte);
    load_next_byte = false;
    sound_id = id;
    
    return true;
}

void loadSoundByte()
{
    if (!load_next_byte || sound_id == 0) {
        return;
    }
    if (bytes_loaded >= sound_table[sound_id].len) {
        stopSound();
        startSoundInQueue();
    } else {
        sound_byte = USI_TWI_Read_Next_One_Byte();
        bytes_loaded++;
        load_next_byte = false;
    }
}

void stopSound()
{
    if (sound_id == 0) {
        return;
    }
    USI_TWI_Read_Finish();
    USI_TWI_Master_Stop();
    bytes_loaded = 0;
    sound_id = 0;
}

bool checkAccStop() {
    if (acc_stop_detected) {
        if (wait_count_acc_stop != 0) {
            return false;
        }
        if (PINB & _BV(PORTB4)) {
            acc_stop_detected = false;
            return false;
        }
        acc_stop_detected = false;
        doProcedureForAccStop();
        return true;
    }
    
    if (PINB & _BV(PORTB4)) {
        return false;
    }
    acc_stop_detected = true;
    wait_count_acc_stop = DELAY_ACC_STOP;
    return false;
}

void checkLongRun() {
    if (!long_run_detected) {
        return;
    }
    long_run_detected = false;
    doProcedureForLongRun();
}

void doProcedureForAccStart()
{
    initializeSoundQueue();
    addListToSoundQueue(sound_acc_start, sizeof(sound_acc_start));
    startSoundInQueue();
}

void doProcedureForAccStop()
{
    TIMSK &= ~_BV(TOIE1); // Disable TIM1_OVF interrupt to stop the long run detection
    initializeSoundQueue();
    addListToSoundQueue(sound_acc_stop, sizeof(sound_acc_stop));
    startSoundInQueue();
}

void doProcedureForLongRun()
{
    initializeSoundQueue();
    addListToSoundQueue(sound_long_run, sizeof(sound_long_run));
    startSoundInQueue();
}

void shutdown()
{
    PORTB &= ~_BV(PORTB3);
    _delay_ms(2000);
    wdt_enable(WDTO_30MS);
    while(1);
}

void wdt_init()
{
    MCUSR = 0;
    wdt_disable();
}

int main(void)
{
    DDRB |= _BV(DDB1); // Set PB1(Pin 6) to output (PWM)
    DDRB |= _BV(DDB3); // Set PB3(Pin 2) to output (LED)
    DDRB &= ~_BV(DDB4); // Set PB4(Pin 3) to input (ACC check)
    PORTB |= _BV(PORTB3);
    
    cli();
    // Timer 0
    TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); // Clear OC0B on Compare Match, set OC0B at BOTTOM, Fast PWM
    TCCR0B = _BV(CS00); // No Prescaling (= 16MHz)
    // Timer 1
    TCCR1 = _BV(CS13) | _BV(CS12) | _BV(CS11) | _BV(CS10); // Prescale CK/16384 (= 1kHz)
    sei();
    
    USI_TWI_Master_Initialise();
    scanSoundBank();
    readSoundTable();
    sound_id = 0;
    
    TIMSK |= _BV(TOIE0) | _BV(TOIE1); // Enable TIM0_OVF and TIM1_OVF interrupt
    
    _delay_ms(DELAY_ACC_START * 1000);
    doProcedureForAccStart();
    
    while (1) {
        loadSoundByte();
        if (checkAccStop()) {
            break;
        }
        checkLongRun();
    }
    while (sound_id != 0) {
        loadSoundByte();
    }
    shutdown();
}

