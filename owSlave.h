/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Ben Coughlan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef OWSLAVE_H
#define OWSLAVE_H

#include "ch32v003fun.h"

#define CMD_SEARCH_ROM 0xF0
#define CMD_MATCH_ROM 0x55
#define CMD_SKIP_ROM 0xCC
#define CMD_READ_ROM 0x33
#define CMD_ALARM_SEARCH 0xEC

typedef enum
{
	WAIT_RESET,		// When this device was deselected and is waiting for a reset pulse
	PRESENCE_DELAY, // This device is responding to a reset pulse with a presence pulse
	PRESENCE_PULSE,
	WRITE, // The master is writing to the bus
	READ   // The master is reading from the bus

} state_t;

volatile state_t state; // the current state of this onewire device
volatile uint8_t bit_count;
volatile uint8_t current_byte;
volatile uint8_t ROM_command;
volatile uint8_t rom_matched;
volatile uint8_t id_index;
volatile uint8_t read_val;
volatile uint8_t tx_byte;
volatile uint8_t *id;

volatile uint32_t resetPulseTime = 0;
volatile int32_t pulse_width = 0;

uint8_t (*_callback_byte_received)(
	uint8_t byte);				   // called at the end of each byte received from master after ROM commands
void (*_callback_byte_sent)(void); // called at the end of each byte sent to master
void (*_callback_selected)(void);  // called when this device is selected

#define set_timer(us_inc)      \
	{                          \
		TIM1->CNT = 0;         \
		TIM1->CH1CVR = us_inc; \
	}

#define ONEWIRE_PIN_RELEASE() funDigitalWrite(PC6, FUN_HIGH)  
#define ONEWIRE_PIN_DRIVE_LOW() funDigitalWrite(PC6, FUN_LOW) 
#define ONEWIRE_PIN_READ() funDigitalRead(PC6)

/**
 * @brief Starts the OneWire slave device.
 *
 * This function initializes the OneWire slave device with the provided bus ID.
 *
 * @param[in] bus_id The bus ID of the OneWire slave device.
 */
void onewireslave_start(uint8_t *bus_id)
{
	id = bus_id;
	state = WAIT_RESET;
}

/*
 * Register callback for bytes received from master.
 *
 * The callback takes the received byte as it's only argument.
 *
 * If the device wishes to transition into a transmit state, this function
 * returns non-zero.  If it needs to receive more bytes, it returns zero.
 * If it returns non-zero it should also setup the next byte to be written
 * (if it hasn't been already) by calling set_txbyte().
 *
 * The callback is called from interrupt context.
 */
void onewireslave_set_received(uint8_t (*callback)(uint8_t))
{
	_callback_byte_received = callback;
}

/*
 * Register callback for bytes sent to master.
 *
 * The callback is called at the end of each byte written to the master.
 * This callback should set up the next byte to be sent by calling set_txbyte().
 *
 * The callback is called form interrupt context.
 */
void onewireslave_set_sent(void (*callback)(void))
{
	_callback_byte_sent = callback;
}

/*
 * Register callback for when this device is selected.
 *
 * The callback is called whenever this device is selected at the beginning of a
 * new transaction.  This includes SkipROM commands.  This is an opportunity to
 * reset state to known values.
 *
 * The callback is called form interrupt context.
 */
void onewireslave_set_selected(void (*callback)(void))
{
	_callback_selected = callback;
}

/*
 * Set the next byte to send to the master.  This must be done before the final
 * bit of the previous byte on the bus, usually from the received and sent callbacks.
 *
 * The previous byte will be repeated if a new one hasn't been set.
 *
 * Byte will be sent once this device is in a transmit state, following a non-zero return
 * from the receive callback.
 */
void onewireslave_set_txbyte(uint8_t data)
{
	tx_byte = data;
}

/**
 * these function form a sort of hierachy with process_bit() as the root, and the branching
 * determined by the values of ROM_command and rom_matched.  These are reset to 0 on every
 * reset pulse, then each bit received from the master is sent through process_bit() to
 * whichever function is currently being executed.
 *
 * This will process all ROM commands and then forward bytes on to application code to deal
 * with function commands and other data transfer.
 */

void do_match_rom(uint8_t val)
{
	// assume state == WRITE
	uint8_t id_bit_val = (id[id_index] >> bit_count++) & 0x01; // LSB first
	if (val != id_bit_val)
	{
		state = WAIT_RESET; // deselected
	}
	else if (bit_count == 8)
	{
		if (id_index == 0)
		{
			rom_matched = 0x01;
			if (_callback_selected)
				_callback_selected();
		}
		bit_count = 0;
		id_index--;
	}
}

void do_search_rom(uint8_t val)
{
	if (state == WRITE)
	{
		if (!val == !(read_val & 0x01))
		{
			state = READ;
			if (bit_count == 8)
			{
				if (id_index == 0)
				{
					rom_matched = 0x01;
					if (_callback_selected)
						_callback_selected();
				}
				bit_count = 0;
				id_index--;
			}
			read_val = (id[id_index] >> bit_count) & 0x01;
		}
		else
		{
			state = WAIT_RESET; // we were deselected
		}
	}
	else
	{
		read_val = ~read_val;
		if (read_val == ((id[id_index] >> bit_count) & 0x01))
		{
			state = WRITE;
			bit_count++;
		}
	}
}

void do_read_rom()
{
	read_val = ((id[id_index] >> bit_count) & 0x01);
	if (++bit_count == 8)
	{
		if (id_index == 0)
		{
			rom_matched = 0x01;
			if (_callback_selected)
				_callback_selected();
		}
		bit_count = 0;
		id_index--;
	}
}

// TODO expose alarm condition API
volatile uint8_t alarm_condition = 0;
void do_alarm_search(uint8_t val)
{
	if (alarm_condition)
	{
		do_search_rom(val);
	}
	else
	{
		state = WAIT_RESET;
	}
}

void get_rom_command(uint8_t val)
{
	// assume we're in WRITE
	current_byte = (current_byte >> 1) | (val != 0 ? 0x80 : 0x00);
	if (++bit_count == 8)
	{
		ROM_command = current_byte;
		id_index = 7;
		bit_count = 0;
		// decide if we need to start reading/writing

		switch (current_byte)
		{
		case CMD_SEARCH_ROM:
			do_search_rom(0);
			break;
		case CMD_ALARM_SEARCH:
			do_alarm_search(0);
			break;
		case CMD_SKIP_ROM:

			rom_matched = 0x01;
			if (_callback_selected)
				_callback_selected();
			break;
		case CMD_READ_ROM:
			state = READ;
			do_read_rom();
			break;
		default:
			break;
		}
	}
}

/**
 * If we're still selected then start processing bits as higher levels functions.
 * This will start in WRITE, and send a byte at a time to user callback.  If the callback
 * returns non-zero, then we switch to READ and start transmitting tx_byte.
 */
void do_function_bytes(uint8_t val)
{
	if (state == WRITE)
	{
		current_byte = (current_byte >> 1) | (val != 0 ? 0x80 : 0x00);
		if (++bit_count == 8)
		{
			bit_count = 0;
			if (_callback_byte_received)
			{
				if (_callback_byte_received(current_byte))
				{
					read_val = tx_byte & 0x01;
					state = READ;
				}
			}
		}
	}
	else
	{
		if (++bit_count == 8)
		{
			bit_count = 0;
			if (_callback_byte_sent)
			{
				_callback_byte_sent();
			}
		}

		read_val = (tx_byte >> bit_count) & 0x01;
	}
}

/**
 * @brief Processes a bit value based on the ROM command and the ROM matching status.
 *
 * This function is responsible for processing a bit value based on the current ROM command and the ROM matching status.
 * If the ROM is matched, it calls the `do_function_bytes` function to perform the necessary operations.
 * If the ROM is not matched, it switches based on the ROM command and performs the corresponding actions.
 * If the ROM command is unknown or equal to CMD_SKIP_ROM, it sets the state to WAIT_RESET.
 *
 * @param val The bit value to be processed.
 */
void process_bit(uint8_t val)
{
	if (rom_matched)
	{
		do_function_bytes(val);
	}
	else
	{
		switch (ROM_command)
		{
		case 0x00:
			get_rom_command(val);
			break;
		case CMD_SEARCH_ROM:
			do_search_rom(val);
			break;
		case CMD_ALARM_SEARCH:
			do_alarm_search(val);
			break;
		case CMD_MATCH_ROM:
			do_match_rom(val);
			break;
		case CMD_READ_ROM:
			do_read_rom(val);
			break;
		case CMD_SKIP_ROM:
		default:
			state = WAIT_RESET; // unknown ROM command
			break;
		}
	}
}
#endif // OWSLAVE_H