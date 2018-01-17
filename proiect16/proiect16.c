/*
Translated from Romanian so sorry if it doesn't always sound good
SM Project  - Guitar tuner
Alexandru Petrencu
Mihnea Gheorghiu
Victor Tone

group 344C2

PA4 \
PA5 \ data output for LCD (data is being transmited on 4 bits)  
PA6 /
PA7 /

PA0 - Pin for  operation selection (LCD)
PA1 - Pin for Read/ Write (LCD)
PA2 - Enable Pin(LCD)

PB2		microfon (input)
PB3		\	
PB4		- string selection(unfortunatly, not implemented)
PB5		/
PB6		led too high (output)
PB7		led too low (output)
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>

/************************************************************************ 
 * DEFINEs for LCD module parametrization
 ************************************************************************/
#define LcdDATA_DDR		DDRA			// Port on which we connect the data wires to the LCD
#define LcdDATA_PORT	PORTA
#define LcdDATA_PIN		PINA

#define LcdCMD_DDR		DDRA			// Port on which we connect the command wires to the LCD
#define LcdCMD_PORT		PORTA
#define LcdCMD_PIN		PINA

#define LcdD4			PA4				// Pin for data wire D4 on the LCD
#define LcdD5			PA5				// Pin for data wire D5 on the LCD
#define LcdD6			PA6				// Pin for data wire D6on the LCD
#define LcdD7			PA7				// Pin for data wire D7 on the LCD

#define LcdRS			PA0				// Pin for operation selection (LCD)
#define LcdRW			PA1				// Pin for Read/ Write (LCD)
#define LcdE			PA2				// Enable Pin (LCD)

#define LCD_INSTR_4wire 		0x28 	// 4 data wires, font 5x8
#define LCD_INSTR_display 		0x0C 	// Display On, Cursor On, Blinking On ( 1 Display Cursor Blink )
#define LCD_INSTR_clearDisplay 	0x01 	// Clear Display
#define LCD_INSTR_returnHome 	0x02 	// Return Cursor and LCD to Home Position
#define LCD_INSTR_nextLine 		0xC0 	// Return Cursor and LCD to Home Position

/* CPU frequency */
#define F_CPU				16000000			
/* prescaler value */
#define PRESCALER			64				
/* timer frequency	*/
#define BASE_FREQUENCY		(F_CPU/PRESCALER)	

#define E_STRING	164.82		/* MI grav  # in English I think*/
#define A_STRING	220.00		/* LA 		*/
#define D_STRING	293.66		/* RE		*/
#define G_STRING	392.00		/* SOL		*/
#define B_STRING	493.88		/* SI		*/
#define EH_STRING   659.26		/* MI high	*/

#define nop() __asm__ __volatile__ ("nop" ::)

/************************************************************************ 
 * LCD API.. implementation is in the lowest part of the file
 ************************************************************************/
void LCD_init();											// Initialize LCD module. Must be called before performing any operation on the LCD
void LCD_writeInstruction(unsigned char _instruction);	// Send an instruction to the lcd (see datasheet)
void LCD_writeData(unsigned char _data);					// Send data to theLCD for displaying
void LCD_write(unsigned char _byte);						// Send a  byte to the LCD  (doesn't matter if it's instruction or data)
void LCD_waitNotBusy();									// Wait until the LCD becomes available for a new command
void LCD_print(char* _msg);								// Display information on the LCD (just 1 line, first 16 chars)
void LCD_print2(char* _msg1, char* _msg2);				// Print on  2 lines on the LCD
void LCD_printDecimal2u(unsigned int _n);					// Display number in base 10 on the LCD
void LCD_printHexa(unsigned int _n);						// Display a number in HEX on the LCD
void LCD_waitInstructions(unsigned char _instructions);	// Wait a number of clock cycles.. loop


/* timer value for the 6 strings */
unsigned int Center_Count[] =
{
	BASE_FREQUENCY/EH_STRING,			
	BASE_FREQUENCY/B_STRING,			
	BASE_FREQUENCY/G_STRING,			 
	BASE_FREQUENCY/D_STRING,			 
	BASE_FREQUENCY/A_STRING,			 
	BASE_FREQUENCY/E_STRING,			 
};

/* values for the middle timer */
/*intervals between the oscilation periods of the strings  */
unsigned int Transition_Count[] =
{
	BASE_FREQUENCY/(B_STRING+(EH_STRING-B_STRING)/2),		
	BASE_FREQUENCY/(G_STRING+(B_STRING-G_STRING)/2),		
	BASE_FREQUENCY/(D_STRING+(G_STRING-D_STRING)/2),		
	BASE_FREQUENCY/(A_STRING+(D_STRING-A_STRING)/2),		
	BASE_FREQUENCY/(E_STRING+(A_STRING-E_STRING)/2),		
};

// overflow accumulator
volatile unsigned char count_hi;	

//timer 0 overflow interrupt
SIGNAL(SIG_OVERFLOW0)	
{
	count_hi++;						
}


/* MAIN */

int main(void) 
{
	unsigned int i;
	unsigned int j;
	unsigned int k; 
	unsigned int count;
	unsigned int val;

/* Initialize inputs and outputs */

/* PB2 = input */
	cbi(DDRB, 2);
/* no pullups */
	cbi(PORTB, 2);

/* PB6 = output */
	sbi(DDRB, 6);
/* PB7 = output */
	sbi(DDRB, 7);			

	sbi(DDRA, 0);
	sbi(DDRA, 1);
	sbi(DDRA, 2);

	sbi(DDRA, 4);
	sbi(DDRA, 5);
	sbi(DDRA, 6);
	sbi(DDRA, 7);

/* set prescaler for  counter 0 to f/64 (250 kHz) */
	outp(0x03,TCCR0);
/* interrupt on counter overflow = enable */
	sbi(TIMSK,TOIE0);		

	/* global interrupt = enable */
	asm volatile ("sei");

// Functon for initializin LCD and printing welcome message
	LCD_init();
	LCD_writeInstruction(LCD_INSTR_clearDisplay);
	LCD_print("Hello");

/*	for(k=0;k<0x09;k++)
		for(i=0;i<=0xFF;i++)
			for(j=0;j<=0xFF;j++)
				nop();
	LCD_writeInstruction(LCD_INSTR_clearDisplay);*/

/* infinite cycle */
	while (1)								
	{

/* reset time average value */
		count = 0;							

/* wait to receive something on the microphone */
		loop_until_bit_is_set(PINB,2);		
/* we have a rising edge =>start samplingl */

/* reset counter and the overflow counter */
		outp(0,TCNT0);						
		count_hi = 0;						

/* sampling loop */
/* for a string, we make 16 measurements */		
		for (i=0;i<16;i++)
		{
/* we wait for transition from 1 to 0 (downward edge, high-to-low)*/
			while (bit_is_set(PINB,2))		
/*no downward front => bad reading, we ignore */
				if (count_hi > 80)			
					break;					

/* we wait for a value change from 0 to 1 (rising signal edge) */
			while (bit_is_clear(PINB,2))	
/* no rising edge => bad reading, ignore */
				if (count_hi > 80)			
					break;					

/*we store the results of the 16 counts in  "count"  */
			count += (count_hi << 8) + inp(TCNT0); 

/* reset counter */
			outp(0,TCNT0);					

/* if the reading is bad, we go out from the sampling loop and */
/* we wait again for a leading edge, that was the word */
			if (count_hi > 80)		
				break;				

/* reset overflow counter */
			count_hi = 0;					
		}

/* turn off leds */				
		sbi(PORTB,PB6);
		sbi(PORTB,PB7);

/* if the measurement is ok */
		if (count_hi <= 80)	
		{
/*calulating average value for measurements */
			count = count >> 4;
	
/* going through the time intervals to find out what string we want to tune */			
			for (i = 0; i < sizeof(Transition_Count) / sizeof(Transition_Count[0] ); i++)
			{
/* if "count" is in a time interval we found the string */
				if (count < Transition_Count[i])
					break;
			}			

			val = 0;	
			/* based on the value of variable "val",  we find out if the string
			  needs to be tightened or loosened or if it is tuned correctly	*/
/* if count <= correct period,  turn on the led "too high" */

			if (count-5 <= Center_Count[i])	
				{	
					val=val+1;
					cbi(PORTB,6);			
				}
				

			
/* if count >= correct period, turn on led for "too low" */	
			if (count+9 >= Center_Count[i])
			{			
				val=val+2;
				cbi(PORTB,7);	
			}			
		
/* if both leds turn on it means the string is tuned correct */
/* with a certain margin of error */ 

/* we make busy-waiting one second to calm the leds */
			for(k = 0; k < 0x0C; k++)
				for(i = 0; i <= 0xFF; i++)
					for(j = 0; j <= 0xFF; j++)
						nop();
			sbi(PORTB,6);
			sbi(PORTB,7);
			if(val == 1)
			{
				LCD_writeInstruction(LCD_INSTR_clearDisplay);
				LCD_print("m");	
				/* m = string needs to be loosened */
			}
			if( val == 2)
			{
				LCD_writeInstruction(LCD_INSTR_clearDisplay);
				LCD_print("n");	
				/* n = string needs to be tightened */
			}
			if( val == 3)
			{
				LCD_writeInstruction(LCD_INSTR_clearDisplay);
				LCD_print("e");	
				/* e = string is correctly tuned */
			}
			val = 0;
		//	asm volatile ("sei");
		}
	}
} 

/************************************************************************ 
 * IMPLEMENTARE API LCD.. implementare este in partea de jos a fisierului
 ************************************************************************/

void LCD_init()
{
	LcdDATA_DDR |=  (1<<LcdD4)|(1<<LcdD5)|(1<<LcdD6)|(1<<LcdD7);	// set data pins for outpute
	LcdCMD_DDR  |=  (1<<LcdRS)|(1<<LcdRW)|(1<<LcdE);				// set command pins as output pins 

	LCD_waitNotBusy();
	
	LcdCMD_PORT   &= ~(1<<LcdRS);									// Set RS line on low
	LcdCMD_PORT   &= ~(1<<LcdRW);									// Set RW line on low (now we are in send instructions mode)
	LcdDATA_PORT  &= ~(1<<LcdD4)&~(1<<LcdD6)&~(1<<LcdD7); 		// Specify that we want 4 data wires, command first  (LcdD5 activ, the rest not)
	LcdDATA_PORT  |=  (1<<LcdD5);									// set command pins as output pins
	
	LcdCMD_PORT |=  (1<<LcdE);					// Set line E(nable) on high; this specifies to the LCD to take over the data
	LCD_waitInstructions(6);					// Wait a time period of T
	LcdCMD_PORT &= ~(1<<LcdE );				// Set line E(nable) on low; transfer is over

	LCD_writeInstruction(LCD_INSTR_4wire); 		// Load command: 4 bit data, 2 lines, 5x8 font
	LCD_writeInstruction(LCD_INSTR_display); 	// Display On, Cursor On, Blinking On
	LCD_writeInstruction(0x06);					// Increment, no shift
	LCD_writeInstruction(0x01);					// Clear Display
}

void LCD_writeInstruction(unsigned char _instruction)
{
	LCD_waitNotBusy();							// wait for the  LCD to be free to receive the commands 
	LcdCMD_PORT &= ~(1<<LcdRS);				// set RS pin on low.. low=instructions, high=data
	LcdCMD_PORT &= ~(1<<LcdRW);				// set RW pin on low (we are in commands module now)
	LCD_write(_instruction);					// calling the procedure that sends the byte through the data wires
}


void LCD_writeData(unsigned char _data)
{
	LCD_waitNotBusy();							// waiting for the LCD to be free to receive commands
	LcdCMD_PORT |=  (1<<LcdRS);				// set RS pin on high
	LcdCMD_PORT &= ~(1<<LcdRW);				// set  RW pin on low (we are in the data module now)
	LCD_write(_data);							// calling the procedure that sends the byte through the data wires
}

void LCD_write(unsigned char _byte)
{
	unsigned char _byte2;
	
	_byte2 = _byte>>4;

	LcdDATA_PORT &= ~(1<<LcdD4);
	if ( bit_is_set( _byte2, 0 ) )
		LcdDATA_PORT |= (1<<LcdD4);
		
	LcdDATA_PORT &= ~(1<<LcdD5);
	if ( bit_is_set( _byte2, 1 ) )
		LcdDATA_PORT |= (1<<LcdD5);

	LcdDATA_PORT &= ~(1<<LcdD6);
	if ( bit_is_set( _byte2, 2 ) )
		LcdDATA_PORT |= (1<<LcdD6);

	LcdDATA_PORT &= ~(1<<LcdD7);
	if ( bit_is_set( _byte2, 3 ) )
		LcdDATA_PORT |= (1<<LcdD7);

	LcdCMD_PORT |= (1<<LcdE);						// Set Pin E on high
	LCD_waitInstructions(6);						// Wait a time period T
	LcdCMD_PORT &= ~(1<<LcdE);						// SetPin E on low

	LCD_waitInstructions(6);						// Wait a time period T
	
	LcdDATA_PORT &= ~(1<<LcdD4);
	if ( bit_is_set( _byte, 0 ) )
		LcdDATA_PORT |= (1<<LcdD4);
	
	LcdDATA_PORT &= ~(1<<LcdD5);
	if ( bit_is_set( _byte, 1 ) )
		LcdDATA_PORT |= (1<<LcdD5);

	LcdDATA_PORT &= ~(1<<LcdD6);
	if ( bit_is_set( _byte, 2 ) )
		LcdDATA_PORT |= (1<<LcdD6);

	LcdDATA_PORT &= ~(1<<LcdD7);
	if ( bit_is_set( _byte, 3 ) )
		LcdDATA_PORT |= (1<<LcdD7);

	LcdCMD_PORT |= (1<<LcdE);						// Set Pin E on high
	LCD_waitInstructions(6);						// Wait a period of time T
	LcdCMD_PORT &= ~(1<<LcdE);						// Set Pin E on low
}


void LCD_waitNotBusy()
{
	unsigned char _loop = 1;

	while (_loop)
	{
		LcdDATA_DDR &= ~(1<<LcdD4 | 1<<LcdD5 | 1<<LcdD6 | 1<<LcdD7);	// Set data pins of the LCD on in to read the busy flag
		LcdDATA_PORT &= ~(1<<LcdD4 | 1<<LcdD5 | 1<<LcdD6 | 1<<LcdD7); // Deactivate pullup resistor for in pins
		
		LcdCMD_PORT &= ~(1<<LcdE);						// Set pin E on low; it should already be on low, just making sure
		LcdCMD_PORT &= ~(1<<LcdRS);					// Set pin RS on low
		LcdCMD_PORT |=  (1<<LcdRW);					// Set RW on high (now we are in the interogation busy/adr mode)
		
		LcdCMD_PORT |= (1<<LcdE);						// Set Pin  E on high
		LCD_waitInstructions(6);						// Wait a time period T
		_loop = LcdDATA_PIN & (1<<LcdD7);				// Read the busy flag
		LcdCMD_PORT &= ~(1<<LcdE);						// Set Pin  E on low
		
		LCD_waitInstructions(6);						// Wait a time period T
		
		LcdCMD_PORT |= (1<<LcdE);						// Set Pin E on high
		LCD_waitInstructions(6);						// Wait a time period T
		LcdCMD_PORT &= ~(1<<LcdE);						// Set Pin E on low

		LcdDATA_DDR |= (1<<LcdD4 | 1<<LcdD5 | 1<<LcdD6 | 1<<LcdD7); // Set the LCD port back as output port
	}
}




void LCD_printDecimal2u(unsigned int _n)
{
	unsigned char tmp=0;
	
	// Extracting 100 units
	while(_n>=100)
		_n-=100;

	while(_n>=10){
		tmp++;
		_n-=10;
	}

	LCD_writeData(tmp+'0');
	LCD_writeData(_n+'0');
}

void LCD_printHexa(unsigned int _n)
{
	unsigned char _tmp = _n>>4;
	if (_tmp>9)
		_tmp += 'A'-10;
	else
		_tmp += '0';
	LCD_writeData( _tmp );
	_tmp = _n & 0x0F;
	if (_tmp>9)
		_tmp += 'A'-10;
	else
		_tmp += '0';
	LCD_writeData( _tmp );
}


void LCD_print(char* _msg)
{
	unsigned char i=0;
	for( ; _msg[i]!=0 && i<16; i++)
		LCD_writeData( _msg[i] );
}

void LCD_print2(char* _msg1, char* _msg2)
{
	LCD_writeInstruction(LCD_INSTR_clearDisplay);
	LCD_print(_msg1);
	LCD_writeInstruction(LCD_INSTR_nextLine);
	LCD_print(_msg2);
}


void LCD_waitInstructions(unsigned char _instructions)
{
	while (_instructions--)
		;
}
