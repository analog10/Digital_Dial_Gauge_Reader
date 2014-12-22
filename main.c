#include <stdint.h>
#define INCTYPE <msp430g2452.h>
#include <msp430.h>

#define BAUD 9600
#define DATA_SAMPLE_PERIOD (196 * 16)
#define CLOCK_CUTOFF_TIME  ((352 + 200) * 16)
#define DATA_SAMPLE_INTERVAL (DATA_SAMPLE_PERIOD / 9)

#define START_MEASURE 'M'
#define START_ERROR 'E'

enum {
	ERR_SHIFT = 0xE0
};

enum {
	/* P1 */
	P1_TX = BIT2
	,READ_OK_BIT = BIT4
	,DATA_BIT = BIT5
	,CLOCK_BIT = BIT0
};

enum {
	ST_IDLE
	,ST_SAMPLE_DATA
	,ST_WAIT_END
};

unsigned wait_end = 0;
volatile int16_t shift_val = 0;
volatile uint16_t shift_count = 0;
volatile uint16_t one_bit_count = 0;
volatile uint16_t total_bit_count = 0;

volatile uint16_t tx_char = 0;

volatile unsigned output_count = 0;
volatile uint8_t output_buffer[16];
volatile const uint8_t *cur_output;

/* Use constants here for efficiency..
 * but it would be nice to design for runtime set baud rates. */

static uint8_t counter = 0;
void br(void){
	++counter;
}

#define TICKS_PER_BIT (F_CPU / BAUD)
static const uint16_t rx_ticks_per_bit = TICKS_PER_BIT;

__attribute__((interrupt(TIMER0_A1_VECTOR)))
void SoftUART_TX(void){
	switch(TA0IV){
		case TA0IV_TA0CCR1:
			if(wait_end){
				P2OUT ^= 0x80;
				wait_end = 0;

				/* Let's get started on the stop bit now.. */
				TA0CCTL1 = OUTMOD_0;

				/* Schedule timer transition. */
				TA0CCR1 = TAR;
				TA0CCR1 += rx_ticks_per_bit;

				register int16_t value;
				/* Comparator timeout occurred, time to output shift data to serial.*/
				if(!shift_count && 3 == output_count){
					P1OUT |= READ_OK_BIT;

					/* Compose outbound packet. */
					value = START_MEASURE;

					register uint16_t tmp = output_buffer[2];
					output_buffer[5] = (tmp >> 4) + 'A';
					output_buffer[4] = (tmp & 0xF) + 'A';

					tmp = output_buffer[1];
					output_buffer[3] = (tmp >> 4) + 'A';
					output_buffer[2] = (tmp & 0xF) + 'A';

					tmp = output_buffer[0];
					output_buffer[1] = (tmp >> 4) + 'A';
					output_buffer[0] = (tmp & 0xF) + 'A';

					output_buffer[6] = '\n';
					output_count = 7;
				}
				else{
					/* Output shift error count packet. */
					output_buffer[0] = '5';
					output_buffer[1] = shift_count + 'A';
					output_buffer[2] = output_count + 'a';
					output_buffer[3] = '\n';
					output_count = 4;

					value = START_ERROR;
				}

				cur_output = output_buffer;

				/* Add stop bit */
				value |= 0x100;

				/* Shift the LSB into the C bit of the SR.  */
				value >>= 1;
				tx_char = value;

				/* This ASM clause defines output bit at CCR1 trigger. */
				__asm__ (
					" mov #176,%[value]\n"
					" jnc 10f\n"
					" xor #128,%[value]\n"
					"10:"
					" mov %[value],%[cctl]\n"
					:
					:
					[cctl] "m" (TA0CCTL1),     /* 3 */
					[value] "r" (value)     /* 3 */
				);

				/* Reset all other state. */
				shift_val = 0;
				shift_count = 0;
			}
			else{
				int16_t tx_wait = 0;
				if((0x1 == tx_char) && output_count){
					--output_count;
					tx_wait = *cur_output;
					tx_wait |= 0x100;
					tx_wait <<= 1;
					++cur_output;
				}

				__asm__ (
						/* Schedule next bit transition. */
						" add %[TICKS],%[ccr]\n"

						/* Assume the next bit is a 1.
						 * Note if the carry bit is 1 then assumption was correct.
						 * Move the next bit to output to the C flag;
						 * If the result is nonzero */
						" mov %[init],%[cctl]\n"
						" rra %[tx_buf]\n"
						" jnz 3f\n"

						/* If the carry bit is nonzero, check the buffer for pending data. */
						" jnc 5f\n"

						/* At this point, scheduled the final stop bit.
						 * Wait buffer may have more stuff.
						 * XOR because nonzero result sets Z flag.
						 * If not then set to idle. */
						" xor %[tx_w], %[tx_buf]\n"
						" mov #0,%[tx_w]\n"
						" jnz 4f\n"

						/* Value is zero, move into the end state. */
						" mov %[end],%[cctl]\n"
						" jmp 4f\n"

						/* Set to idle. (setting to OUT) */
						"5:\n"
						" mov #4,%[cctl]\n"
						" jmp 4f\n"

						/* Otherwise assume we will output 0.
						 * If the stop bit is still in tx_buf go to the end. */
						"3:\n"
						" jc 4f\n"
						" bis %2,%[cctl]\n"
						"4:\n"
						:
						:
						[TICKS] "m" (rx_ticks_per_bit), /* 0 */
					[ccr] "m" (TA0CCR1),      /* 1 */
					[mod2] "i" (OUTMOD2),      /* 2 */
					[cctl] "m" (TA0CCTL1),     /* 3 */
					[tx_buf] "m" (tx_char),     /* 4 */
					[tx_w] "m" (tx_wait),               /* 5 */
					[init] "i" (OUTMOD_1 | CCIE),      /* 6 */
					[end] "i" (OUTMOD_1 | OUT)      /* 6 */
						:
							"cc"
								);
			}
			break;

		case TA0IV_TAIFG:
			/* Turn off Read OK LED */
			P1OUT &= ~READ_OK_BIT;
			break;

		default:
			break;
	}
}

__attribute__((interrupt(TIMER0_A0_VECTOR)))
void DataBit(void){
	TA0CCR0 += DATA_SAMPLE_INTERVAL;
	one_bit_count += (TA0CCTL0 & SCCI) ? 1 : 0;
	++total_bit_count;
}

__attribute((interrupt(COMPARATORA_VECTOR)))
void Comparator(void){
	CACTL1 ^= CAIES;
	if(!(CACTL1 & CAIES)){
		/* Falling edge. */
		TA0CCR0 += DATA_SAMPLE_INTERVAL;

		/* Cancel timeout. */
		TA0CCTL1 = OUT;
		wait_end = 0;

		/* Change capture source to DATA bit. */
		TA0CCTL0 = SCS | CM_2 | CCIE;
		P1SEL |= DATA_BIT;

		one_bit_count = (P1IN & DATA_BIT) ? 1 : 0;
		total_bit_count = 1;
	}
	else{
		/* Rising edge. */

		/* Determine whether this was 1 or 0 data based on majority vote. */
		if(one_bit_count > total_bit_count / 2)
			shift_val |= 0x100;
		shift_val >>= 1;
		++shift_count;

		if(8 == shift_count){
			P2OUT ^= 0x40;
			output_buffer[output_count] = shift_val;
			++output_count;
			shift_val = 0;
			shift_count = 0;
		}

		/* Change back to capturing comparator change. */
		TA0CCTL0 = SCS | CM_3 | CAP | CCIS_1;
		P1SEL &= ~DATA_BIT;

		/* Repurpose TA0CCR1 for timeout. */
		TA0CCR1 = TA0R;
		TA0CCR1 += CLOCK_CUTOFF_TIME;
		TA0CCTL1 = OUT | CCIE;
		wait_end = 1;
	}
	CACTL1 &= ~CAIFG;
}

int main(void){
	WDTCTL = WDTPW | WDTHOLD;

	/* Set to 16 MHz. */
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;

	/* SMCLK = DCO / DIVS = nMHz */
	/* ACLK = VLO = ~ 12 KHz */
	BCSCTL2 &= ~(DIVS_0);
	BCSCTL3 |= LFXT1S_2; 

	/* Disable external crystal. */
	P2SEL = 0;

	/* Setup comparator. */
	CACTL1 = CAREF_1 | CARSEL | CAIES | CAIE;
	CACTL2 = P2CA0 | CAF;
	CAPD = CAPD0;

	/*  */
	CACTL1 |= CAON;
	CACTL1 &= ~CAIFG;

	/* Trigger on comparator change. */
	TA0CCTL0 = SCS | CM_3 | CAP | CCIS_1;
	TA0CCTL1 = OUT;

	/* Enable RX pullup */
	P1OUT = P1_TX;
	P1DIR = P1_TX | READ_OK_BIT;
	P1SEL |= P1_TX;

	P2DIR = 0xFF;

	/* Set up serial. */
	TA0CTL = TASSEL_2 | MC_2 | TACLR| TAIE;

	__eint();

	while(1){
		LPM0;
	}
}
