#include "ch32v003fun.h"
#include "owSlave.h"

uint8_t bus_id[8] = {0xC4, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0xF0}; // CRC, Family, Serial LSB..MSB
uint8_t data_byte = 0x00;

void TIM1_CC_IRQHandler(void) __attribute__((interrupt));
void TIM1_CC_IRQHandler(void)
{
	if (TIM1->INTFR & TIM_CC1IF)
	{
		if ((TIM1->CNT >= (TIM1->CH1CVR - 2)) && (TIM1->CNT <= (TIM1->CH1CVR + 10)))
		{
			volatile uint8_t pin_val = ONEWIRE_PIN_READ();

			if (state == WRITE || state == READ)
			{
				ONEWIRE_PIN_RELEASE();
				process_bit(pin_val);
			}
			else if (state == PRESENCE_PULSE)
			{
				ONEWIRE_PIN_RELEASE();
				state = WRITE;
			}
			else if (state == PRESENCE_DELAY)
			{
				ONEWIRE_PIN_DRIVE_LOW();
				state = PRESENCE_PULSE;
				set_timer(70);
			}
		}
		TIM1->INTFR &= ~TIM_CC1IF;
	}
	else if (TIM1->INTFR & TIM_CC2IF)
	{
		state = WAIT_RESET;
		bit_count = 0;
		ROM_command = 0;
		id_index = 0;
		read_val = 0;
		rom_matched = 0;
		TIM1->INTFR &= ~TIM_CC2IF;
	}
}

void EXTI7_0_IRQHandler(void) __attribute__((interrupt));
void EXTI7_0_IRQHandler(void)
{
	if (EXTI->INTFR & EXTI_INTF_INTF6)
	{
		if (ONEWIRE_PIN_READ())
		{
			if (state == WAIT_RESET)
			{
				pulse_width = TIM1->CNT;
				EXTI->RTENR &= EXTI_RTENR_TR6;

				if (pulse_width < 450 || pulse_width > 510)
				{
					return;
				}
				if (pulse_width > 470)
				{
					state = PRESENCE_DELAY;
					set_timer(30);
				}
			}
		}
		else
		{
			if (state == WRITE)
			{
				set_timer(40);
			}
			else if (state == READ)
			{
				if (read_val & 0x01)
				{
					ONEWIRE_PIN_RELEASE();
				}
				else
				{
					ONEWIRE_PIN_DRIVE_LOW();
				}
				set_timer(50);
			}
			else if (state == WAIT_RESET)
			{
				TIM1->CH1CVR = 600;
				TIM1->CNT = 0;
				EXTI->RTENR |= EXTI_RTENR_TR6;
			}
		}

		EXTI->INTFR |= EXTI_INTF_INTF6;
	}
}

uint8_t byte_received(uint8_t data)
{
	switch (data)
	{
	case 0xBB:
		data_byte = 0x00; // reset the counter
		onewireslave_set_txbyte(data_byte++);
		return 1; // when you want to start writing bytes
	default:
		return 0; // when you still want to receive bytes
	}
}

void byte_sent()
{
	onewireslave_set_txbyte(data_byte++);
}

int main()
{
	SystemInit();

	RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO;

	funPinMode(PC6, (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD));
	ONEWIRE_PIN_RELEASE();

	AFIO->EXTICR |= 0x02 << (2 * 6); // PC6
	EXTI->INTENR |= EXTI_INTENR_MR6;
	EXTI->FTENR |= EXTI_FTENR_TR6;

	NVIC_EnableIRQ(EXTI7_0_IRQn);

	TIM1->PSC = 47;
	TIM1->ATRLR = 1000;
	TIM1->CH1CVR = 500;
	TIM1->CH2CVR = 500;
	TIM1->DMAINTENR = TIM_CC1IE | TIM_CC2IE;
	TIM1->CTLR1 |= TIM_CEN;

	NVIC_EnableIRQ(TIM1_CC_IRQn);

	onewireslave_set_received(byte_received);
	onewireslave_set_sent(byte_sent);

	onewireslave_start(bus_id);

	while (1)
	{
	}
}