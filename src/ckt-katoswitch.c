/*************************************************************************
Title:    CKT-KATOSWITCH
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2026 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h> // For ccp_write_io()
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

#include "debouncer.h"
#include "eepromWearLevel.h"


WearLeveledEEPROM stateSaveEEP;
#define WL_EEPROM_SIZE  63
#define WL_EEPROM_START_ADDR 0

FUSES = 
{
    .OSCCFG = FREQSEL_16MHZ_gc,
};
 
void init_1000hz_tick(void) 
{
	/*
	* Timing calculation:
	* Clock Source: CLK_PER / 2 = 8,000,000 / 2 = 4,000,000 Hz (4 MHz)
	* Desired Interrupt Rate: 1000 Hz (1 ms)
	* 
	* TOP (CCMP) = (f_clk / Target_Freq) - 1
	* TOP = (4,000,000 / 1000) - 1 = 3999
	*/
	TCB0.CCMP = 3999;

	/* Set mode to Periodic Interrupt mode (default is 0x00, but explicit here) */
	TCB0.CTRLB = TCB_CNTMODE_INT_gc;

	/* Enable Capture/Compare interrupt */
	TCB0.INTCTRL = TCB_CAPT_bm;

	/*
	* Configure Clock Source and Enable:
	* - TCB_CLKSEL_CLKDIV2_gc: Use CLK_PER / 2 (4 MHz)
	* - TCB_ENABLE_bm: Enable the timer
	*/
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
}

volatile bool systick = false;

ISR(TCB0_INT_vect) 
{
	/* Clear the interrupt flag */
	TCB0.INTFLAGS = TCB_CAPT_bm;
	systick = true;
}
 
void init_clock(void) 
{
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PEN_bm | CLKCTRL_PDIV_2X_gc);

    /* 3. Wait for oscillator to stabilize */
    while (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm);
}

void init_gpio(void) 
{
	// PORT A
	//  PA7 - Output - no connection, drive low
	//  PA6 - Output - Relay (high = reverse)
	//  PA5 - Output - DRV8231A IN1
	//  PA4 - Output - DRV8231A IN2
	//  PA3 - Output - no connection, drive low
	//  PA2 - Input  - Input B
	//  PA1 - Input  - Input A
	//  PA0 - UPDI   - Reset/programming pin

	// PORT B
	//  PA3 - Input  - Option Jumper C
	//  PA2 - Input  - Option Jumper B
	//  PA1 - Input  - Option Jumper A
	//  PB0 - Output - no connection, drive low

	// Initial pin states for outputs 
	PORTA.OUTCLR = PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm | PIN3_bm;
	PORTB.OUTCLR = PIN0_bm;

	// Outputs are set, inputs are clr in the DIR register
	PORTA.DIRSET = PIN7_bm | PIN6_bm | PIN5_bm | PIN4_bm | PIN3_bm;
	PORTB.DIRSET = PIN0_bm;
	
	// PORTA Pins 1 & 2 are the inputs
	PORTA.DIRCLR = PIN2_bm | PIN1_bm;
	PORTB.DIRCLR = PIN1_bm | PIN2_bm | PIN3_bm;

	// Turn on pull-ups and invert all the inputs since they're active low
	PORTA.PIN2CTRL = PORT_PULLUPEN_bm | PORT_INVEN_bm;
	PORTA.PIN1CTRL = PORT_PULLUPEN_bm | PORT_INVEN_bm;
	PORTB.PIN3CTRL = PORT_PULLUPEN_bm | PORT_INVEN_bm;
	PORTB.PIN2CTRL = PORT_PULLUPEN_bm | PORT_INVEN_bm;
	PORTB.PIN1CTRL = PORT_PULLUPEN_bm | PORT_INVEN_bm;
}

void init()
{
	init_clock();
	init_gpio();
	init_1000hz_tick();
	sei();

	ewlInit(&stateSaveEEP, (const uint8_t*)WL_EEPROM_START_ADDR, WL_EEPROM_SIZE * (sizeof(uint8_t)+1), sizeof(uint8_t));	
};

#define SWITCH_A_MASK  PIN1_bm
#define SWITCH_B_MASK  PIN2_bm

uint8_t getInputs()
{
	return (PORTA.IN & (PIN2_bm | PIN1_bm));
}

#define OPTION_A_MASK  PIN1_bm
#define OPTION_B_MASK  PIN2_bm
#define OPTION_C_MASK  PIN3_bm

uint8_t getOptionJumpers()
{
	return (PORTB.IN & (PIN3_bm | PIN2_bm | PIN1_bm));
}
#define SWITCH_READ_TIME_MS  25
#define COIL_ON_TIME_MS      600

#define STATE_NORMAL  0x01
#define STATE_REVERSE 0x02


// Returns true if the coil is active (points moving) or false if the coil is idle

void setRelayReverse(bool isReverse)
{
	if (isReverse)
		PORTA.OUTSET = PIN6_bm;
	else
		PORTA.OUTCLR = PIN6_bm;
}

bool operateCoilDriver(bool setNormal, bool setReverse, uint16_t currentMillis)
{
	static uint16_t coilDriverStarted = 0;
	static uint8_t currentState = 0;

	if (!(PORTA.OUT & (PIN4_bm | PIN5_bm)))
	{
		// Coil is inactive
		if (setNormal)
		{
			PORTA.OUTSET = PIN5_bm;
			PORTA.OUTCLR = PIN4_bm;
			coilDriverStarted = currentMillis;
			return true;
		}
		else if (setReverse)
		{
			PORTA.OUTSET = PIN4_bm;
			PORTA.OUTCLR = PIN5_bm;
			coilDriverStarted = currentMillis;
			return true;
		}

		return false;
	}


	// If we're here, the coil is active
	if (currentMillis - coilDriverStarted > COIL_ON_TIME_MS)
	{
		// Don't change the relay until we finish the throw
		if (PORTA.OUT & PIN4_bm)
		{
			currentState = STATE_REVERSE;
		} else if (PORTA.OUT & PIN5_bm) {
			currentState = STATE_NORMAL;
		}
		// Shut coil off
		PORTA.OUTCLR = PIN5_bm | PIN4_bm;
		setRelayReverse(STATE_REVERSE == currentState);

		return false;
	}
	return true;
}



int main(void) 
{
	DebounceState8_t inputDebouncer;
	uint16_t lastInputRead = 0;
	uint8_t optionJumpers = 0;
	uint8_t currentState = 0;
	uint16_t millis = 1;

	init();

	_delay_ms(10); // Just wait for everything to stabilize electrically

	initDebounceState8(&inputDebouncer, getInputs());

	// This doesn't need to be debounced.  They're solder jumpers, don't change them
	//  at runtime with the power on
	optionJumpers = getOptionJumpers();

	if (ewlRead(&stateSaveEEP, (const uint8_t*)&currentState, sizeof(uint8_t)))
	{

		// Only do this if we're in pushbutton mode
		// Set up current state based on state on power loss
		if (STATE_REVERSE == currentState)
		{
			operateCoilDriver(false, true, millis);
			setRelayReverse(true); // set relay immediately since we're restoring state
		} else if (STATE_NORMAL == currentState) {
			operateCoilDriver(true, false, millis);
			setRelayReverse(false); // set relay immediately since we're restoring state
		}
	}
	
	while (1) 
	{
		if (systick)
		{
			systick = false;
			millis++;
		}

		bool isPointsMoving = operateCoilDriver(false, false, millis);

		if (!isPointsMoving && (millis - lastInputRead > SWITCH_READ_TIME_MS))
		{
			uint8_t changed = debounce8(getInputs(), &inputDebouncer);
			lastInputRead = millis;

			uint8_t buttonsPressed = getDebouncedState(&inputDebouncer);

			// Different modes
			// Mode 0 is toggle switch - A is level sensitive
			// Mode 1 is pushbuttons - A is normal, B is reverse
			uint8_t newState = 0;
			if (1)
			{
				buttonsPressed = buttonsPressed & changed;
				if (buttonsPressed & SWITCH_A_MASK)
				{
					newState = STATE_NORMAL;
				} else if (buttonsPressed & SWITCH_B_MASK) {
					newState = STATE_REVERSE;
				}
			}
			else if (0)
			{
				// Level sensitive mode
				if (buttonsPressed & SWITCH_A_MASK)
					newState = STATE_REVERSE;
				else
					newState = STATE_NORMAL;				
			}

			if (0 != newState)
			{
				bool setNormal = (newState == STATE_NORMAL);
				bool setReverse = (newState == STATE_REVERSE);
				currentState = newState;
				ewlWrite(&stateSaveEEP, (const uint8_t*)&currentState, sizeof(uint8_t));
				operateCoilDriver(setNormal, setReverse, millis);
			}
		}
	}
	return 0;
}



