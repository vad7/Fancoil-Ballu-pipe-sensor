/*
 * FanCoilPower.c
 *
 * Created: 07.11.2016 14:09:39
 *  Author: Vadim Kulakov, vad7@yahoo.com
 *
 * ATTINY85
 */
#define F_CPU 1000000UL
// Fuses: BODLEVEL = 2V7 (BODLEVEL[2:0] = 101), RESET disabled 
 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/eeprom.h>
#include "onewire.h"

#define RELAYPORT			PORTB
#define RELAY				(1<<PORTB0)
#define RELAY_ON			RELAYPORT |= RELAY
#define RELAY_OFF			RELAYPORT &= ~RELAY
#define RELAY_INIT			DDRB |= RELAY
#define RELAY_STATUS		(PORTB & RELAY)

#define LED1PORT			PORTB
#define LED1				(1<<PORTB3)
#define LED1_ON				LED1PORT |= LED1
#define LED1_OFF			LED1PORT &= ~LED1
#define LED1_INIT			DDRB |= LED1

#define SENS_FREEZING		(PINB & (1<<PORTB2))
#define SENS_INIT_INTR		GIMSK |= (1<<PCIE); PCMSK |= (1<<PCINT2)

// KEY: Short press - show temp; medium press (2..5 sec) - switch freezing mode; long press (>10 sec) - enter setup temp
#define KEYSPIN				PINB
#define KEY					(1<<PORTB1)
#define KEY_PRESSED			!(KEYSPIN & KEY)
#define KEYS_INIT			PORTB |= KEY // pullup

struct _EEPROM {
	uint8_t _OSCCAL;
	uint8_t FreezeSensorFlag;	// 0 - On[5V]/Off[0V], 1 - switch button
	uint8_t FlagFreezing;		// used when FreezeSensorFlag = 1
	uint8_t SensorScanPeriod;	// *0.1 sec
	int16_t HeatingTempMin;		// Min temperature for heating, *10
	int16_t FreezingTempMax;	// Max temperature for freezing, *10
	int16_t DeltaTemp;
	uint16_t RelayOffDelay;		// *0.1 sec
	uint8_t SensorROM_Pipe[8];	// DS18B20 starts with 0x28; AM23xx starts with 'A'
} __attribute__ ((packed));
struct _EEPROM EEMEM EEPROM;

uint8_t TimerCntSec			= 0;
uint8_t FlagFreezing		= 0;
uint8_t FreezeSensorFlag;
uint8_t TempConversionFlag  = 0;
uint8_t TempConversionCnt	= 10; // *0.1 sec
uint8_t TempValid			= 0;
int16_t TempPipe;
int16_t HeatingTempMin;
int16_t FreezingTempMax;
int16_t DeltaTemp;
uint16_t RelayDelay			= 0; // sec
uint8_t	ErrorFlag			= 0;
uint8_t SensFreezingArm		= 0;
uint8_t SensFreezingCnt		= 0;
volatile uint8_t Timer		= 0;

#if(1)
void Delay100ms(unsigned int ms)
{
	while(ms-- > 0) {	
		_delay_ms(100);
		wdt_reset();
	}
}

void FlashLED(uint8_t num, uint8_t toff, uint8_t ton)
{
	while (num-- > 0) {
		LED1_OFF;
		Delay100ms(toff);
		LED1_ON;
		Delay100ms(ton);
	}
	LED1_OFF;
}

void FlashLED2(uint8_t num)
{
	FlashLED(num,num,num);
}

void FlashNumberOnLED(int16_t Number) // -1000..1000
{
	if(Number < 0) {
		FlashLED(3, 1, 1); // minus
		Number = -Number;
	}
	Delay100ms(5);
	FlashLED(Number / 100, 5, 10);
	Delay100ms(10);
	FlashLED(Number % 100 / 10, 5, 5);
	Delay100ms(10);
	FlashLED(Number % 10, 7, 2);
}
#endif

ISR(PCINT0_vect)
{
	// AC sens
	uint8_t s = SENS_FREEZING;
	uint8_t i = 5;
	for(; i > 0; i--) if(SENS_FREEZING != s) break;
	if(i == 0) SensFreezingArm = 1 + (SENS_FREEZING != 0); // 2 = [VCC], 1 = [GND]
}

uint8_t LED_Warning = 0, LED_WarningOnCnt = 0, LED_WarningOffCnt = 0;

ISR(TIM0_OVF_vect) // 0.10035 sec
{
	if(++TimerCntSec == 10) { // 1 sec
		TimerCntSec = 0;
		if(RelayDelay) RelayDelay--;
	}
	if(Timer) Timer--;
	if(TempConversionCnt) TempConversionCnt--;
	if(FreezeSensorFlag) { // 1 - switch button
		if(SensFreezingCnt) SensFreezingCnt--;
		if(SensFreezingArm == 2) {
			if(SensFreezingCnt == 0) {
				FlagFreezing ^= 1;
				SensFreezingCnt = 100; // 10 sec
				eeprom_update_byte(&EEPROM.FlagFreezing, FlagFreezing);
				LED_Warning = FlagFreezing + 1; // 1 led flash - heating, 2 - freezing mode
			}
		}
		SensFreezingArm = 0;
	} else { // 0 - On[5V]/Off[0V]
		if(SensFreezingArm) {
			if(SensFreezingArm / 2 != FlagFreezing) {
				if(++SensFreezingCnt >= 50) {
					FlagFreezing = SensFreezingArm / 2; // 5 sec
					SensFreezingCnt = 0;
					SensFreezingArm = 0;
				}
			} else {
				if(SensFreezingCnt) {
					SensFreezingCnt--;
					SensFreezingArm = 0;
				}
			}
		}
	}
	if(LED_WarningOnCnt) {
		LED1_ON;
		LED_WarningOnCnt--;
	} else if(LED_WarningOffCnt) {
		LED1_OFF;
		LED_WarningOffCnt--;
	} else if(LED_Warning) { // short flashes
		LED_WarningOffCnt = 3;
		LED_WarningOnCnt = 3;
		if(--LED_Warning == 0) LED_WarningOffCnt = 5;
	}
}

/*
void SearchTempSensor(void)
{
	FlashLED(10, 2, 2);
	Delay100ms(10);
	uint8_t rom[8];
	if(OneWire_ReadSerialSingle(rom) == OW_OK) {
		FlashLED(1, 0, 20);
		uint8_t * sens;
		if(eeprom_read_byte((uint8_t*)&EEPROM.SensorROM_Air) == 0xFF && eeprom_read_byte((uint8_t*)&EEPROM.SensorROM_Pipe) != 0xFF) {
			sens = (uint8_t*)&EEPROM.SensorROM_Air;
			FlashLED(2, 5, 5);
		} else {
			sens = (uint8_t*)&EEPROM.SensorROM_Pipe;
			eeprom_update_byte((uint8_t*)&EEPROM.SensorROM_Air, 0xFF); // Clear second sensor
		}
		eeprom_update_block(rom, sens, sizeof(EEPROM.SensorROM_Pipe));
	} else {
		FlashLED(5, 3, 3); // not found
	}
}
*/

#define SETUP_WATCHDOG WDTCR = (1<<WDCE) | (1<<WDE); WDTCR = (1<<WDE) | (0<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0); //  Watchdog 1 s

int main(void)
{
	CLKPR = (1<<CLKPCE); CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (1<<CLKPS1) | (1<<CLKPS0); // Clock prescaler division factor: 8
	MCUCR = (1<<SE) | (0<<SM1) | (0<<SM0); // Idle sleep enable
	PORTB = (1<<PORTB5); // pullup not used pins
	RELAY_INIT;
	LED1_INIT;
	KEYS_INIT;
	// Timer 8 bit
	TCCR0A = (1<<WGM01) | (1<<WGM00);  // Timer0: Fast PWM OCRA
	TCCR0B = (1<<WGM02) | (1 << CS02) | (0 << CS01) | (1 << CS00); // Timer0 prescaller: 1024
	TIMSK |= (1<<TOIE0); // Timer/Counter0 Overflow Interrupt Enable
	OCR0A = 97; // OC0A(TOP)=Fclk/prescaller/Freq - 1; Freq=Fclk/(prescaller*(1+TOP))
	//OCR0B = 0; // 0..OCR0A, Half Duty cycle = ((TOP+1)/2-1)
	//TCCR0A |= (1<<COM0B1); // Start PWM out
	SETUP_WATCHDOG;
	FreezeSensorFlag = eeprom_read_word((uint16_t*)&EEPROM.FreezeSensorFlag);
 	if(FreezeSensorFlag == 0xFF) {
 		eeprom_update_byte(&EEPROM._OSCCAL, OSCCAL);
		eeprom_update_byte(&EEPROM.FreezeSensorFlag, FreezeSensorFlag = 1);
		eeprom_update_byte(&EEPROM.FlagFreezing, 0);
 		eeprom_update_byte(&EEPROM.SensorScanPeriod, 100); // 10 sec
		eeprom_update_word((uint16_t*)&EEPROM.HeatingTempMin, 290);  // *10 C
		eeprom_update_word((uint16_t*)&EEPROM.FreezingTempMax, 250); // *10 C
		eeprom_update_word((uint16_t*)&EEPROM.DeltaTemp, 20); // *10 C
		eeprom_update_word(&EEPROM.RelayOffDelay, 300); // 5 min
 	}
	//OSCCAL = eeprom_read_byte(&EEPROM._OSCCAL);
	HeatingTempMin = eeprom_read_word((uint16_t*)&EEPROM.HeatingTempMin);
	FreezingTempMax = eeprom_read_word((uint16_t*)&EEPROM.FreezingTempMax);
	DeltaTemp = eeprom_read_word((uint16_t*)&EEPROM.DeltaTemp);
	if(FreezeSensorFlag == 0) FlagFreezing = SENS_FREEZING != 0; 
	else FlagFreezing = eeprom_read_byte(&EEPROM.FlagFreezing);
	SENS_INIT_INTR;
	sei();
	while(1) {
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		sleep_cpu();
		wdt_reset();
		if(TempConversionCnt == 0) {
			if(TempConversionFlag) { // Temp ready
				int16_t T = OneWire_ReadTempSingle();
				if((uint8_t)(T >> 8) == 0x80) { // error
					if(ErrorFlag == 0) {
						FlashLED(T & 0x0F, 3, 3);
						ErrorFlag = 1;
					}
				} else {
					TempPipe = T;
					TempValid = 1;
				} 
				TempConversionFlag = 0;
				TempConversionCnt = eeprom_read_byte(&EEPROM.SensorScanPeriod);
			} else {
				if(OneWire_ConvertTemp() == OW_OK) TempConversionFlag = 1;
				TempConversionCnt = 10; // 1 sec
			}
		}
		if(TempValid) {
			if((FlagFreezing && TempPipe <= FreezingTempMax - DeltaTemp)
			|| (!FlagFreezing && TempPipe >= HeatingTempMin + DeltaTemp)) {
				if(RELAY_STATUS == 0) ATOMIC_BLOCK(ATOMIC_FORCEON) RelayDelay = eeprom_read_word(&EEPROM.RelayOffDelay);
				RELAY_ON;
			} else if((FlagFreezing && TempPipe > FreezingTempMax)
					|| (!FlagFreezing && TempPipe < HeatingTempMin)) {
				ATOMIC_BLOCK(ATOMIC_FORCEON) if(RelayDelay == 0) {
					RELAY_OFF;
				}
			}
		}
		if(KEY_PRESSED) {
			Delay100ms(1);
			if(KEY_PRESSED) {
				Timer = 100; // 10 sec
				while(KEY_PRESSED) {
					wdt_reset();
					if(Timer == 0) break;
				}
                if(Timer <= 80 && Timer >= 50) { // 2..5 sec press
                    FlagFreezing ^= 1;
                    eeprom_update_byte(&EEPROM.FlagFreezing, FlagFreezing);
                    LED_Warning = FlagFreezing + 1; // 1 led flash - heating, 2 - freezing mode     
				} else if(Timer == 0) { // Setup
					FlashLED(5, 1, 1);
					TempConversionCnt = 255;
					while(1) {
						__asm__ volatile ("" ::: "memory"); // Need memory barrier
						wdt_reset();
						if(KEY_PRESSED) {
							Delay100ms(1);
							Timer = 12; // 1.2 sec
							while(KEY_PRESSED) wdt_reset();
							if(Timer == 0) { // > 1.2 sec
								FlashLED(2, 2, 2);
								if(FlagFreezing) {
									FreezingTempMax += 10;
								} else {
									HeatingTempMin += 10;
								}
							} else if(Timer <= 11) {
								FlashLED(1, 2, 2);
								if(FlagFreezing) {
									FreezingTempMax -= 10;
								} else {
									HeatingTempMin -= 10;
								}
							}
							TempConversionCnt = 255;
						}
						if(TempConversionCnt == 0) {
							if(FlagFreezing) {
								FlashNumberOnLED(FreezingTempMax);
								eeprom_update_word((uint16_t*)&EEPROM.FreezingTempMax, FreezingTempMax);
							} else {
								FlashNumberOnLED(HeatingTempMin);
								eeprom_update_word((uint16_t*)&EEPROM.HeatingTempMin, HeatingTempMin);
							}
							break;
						}
					}
				} else FlashNumberOnLED(TempPipe);
				Delay100ms(1);
			}
		}
	}
}
