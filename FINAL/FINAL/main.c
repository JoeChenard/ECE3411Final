//Joe Chenard 12/17/21

/* ------- Preamble -------- */
#define F_CPU 16000000UL // Tells the Clock Freq to the Compiler.
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/io.h>      // Defines pins, ports etc.
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "uart.h"		 //standard uart.h provided by Dr. Chandy
#include <inttypes.h>
#include <math.h>

#define SPI_DDR  DDRB
#define SPI_SS   2
#define SPI_MOSI 3
#define SPI_MISO 4
#define SPI_SCK  5
#define ldac	 6

#define BTN1 (!(PINB & (1 << PINB0))) //for testing
#define BTN2 (!(PINB & (1 << PINB1))) //for testing

volatile int cameraRefresh = 5000;
volatile int killTime = 0;

volatile int turn = 0;		//pixycam x coordinate
volatile int turn10 = 0;	//pixycam x coordinate overflow
volatile int fstop = 0;		//pixycam y coordinate
volatile int botState = 0;	//state variable for changing autonomous modes

///////////////////////////////////////////////////////////////////ISR CODE///////////////////////////////////
ISR(TIMER0_COMPA_vect){
	if (cameraRefresh>0)
	cameraRefresh--;
	
	if (killTime > 0){
		killTime--;
	}
}

ISR(TIMER1_COMPA_vect){
	/*
	main logic of autonomous control
	programmed to move forward until it sees a red block (or any color the camera is set to interprets as it's first color signature),
	grabs the block and lifts it up once the block is centered and close enough to the claw, turn a little to the right, and then drops the it.
	The bot can only move forward given the limited number of motor controllers available.
	*/
	
	
	/*if (BTN1){				   //manual control for testing, not on final bot
		PORTD |= (1<<5);		
	}
	if (BTN2){
		PORTD |= (1<<4);
	}*/
	
	switch (botState)
	{
		case 0:{					//if no signal then move forward
			if ((fstop == 0) && (turn == 0) && (turn10 == 0)){ 
				PORTD |= (1<<4) | (1<<5);
				//printf("alpha");
			}else{
				botState++;
			}
		}break;
		
		case 1:{					//has a signal, moving towards it
			if (fstop > 120){
				botState++;
				killTime = 2500;
			}
			
			if ((turn > 100) || turn10){
				PORTD |= (1<<5);	//turn left
			}	
			if (turn <= 200 ){		
				PORTD |= (1<<4);	//turn right
			}
			
			PORTE |= (1<<1) | (1<<2) | (1<<3); //white light (RGB)
			
		}break;
		
		case 2:{					//target in range, picking up
			if (killTime < 5){
				botState++;
				killTime = 3000;
			}
			PORTD |= (1<<7);		//grab
			PORTD &= ~(1<<5);		//left wheel off
			PORTD &= ~(1<<4);		//right wheel off
			PORTE &= ~(1<<3);		//RG
		}break;
		
		case 3:{					//turning right for 3 seconds (left wheel moves robot right)
			if (killTime < 5){
				botState++;
				killTime = 2500;
			}
			PORTD |= (1<<5);		//left on
			PORTD &= ~(1<<7);		//grab off
			PORTD &= ~(1<<6);		//release off
			PORTD &= ~(1<<4);		//right off
			PORTE &= ~(1<<2);		//R
		}break;
		
		case 4:{					//drops target for 2.5s
			if (killTime < 5){		
				botState++;
			}
			PORTD &= ~(1<<7);		//grab off
			PORTD |= (1<<6);		//release
			PORTD &= ~(1<<5);		//left off
			PORTD &= ~(1<<4);		//right off
			PORTE |= (1<<3);		//RB
		}break;
		
		case 5:{					//stops everything
			PORTD &= ~(1<<7);		//grab off
			PORTD &= ~(1<<6);		//release off
			PORTD &= ~(1<<5);		//left off
			PORTD &= ~(1<<4);		//right off
			PORTE &= ~(1<<1);		//B
		}break;
	}	
	//implied hardware turn on
	
}

ISR(TIMER1_COMPB_vect){
	
	PORTD &= ~(1<<7);				//grab off
	PORTD &= ~(1<<6);				//release off
	PORTD &= ~(1<<5);				//left off
	PORTD &= ~(1<<4);				//right off
	//implied hardware turn off
	
}

void InitTimer0(void){						// 1ms timer with 8bit precision
	TCCR0A |= (1<<WGM01);					// Clear on Compare A
	OCR0A = 249;							// Set number of ticks for Compare A
	TIMSK0 = (1<<OCIE0A); ;					// Enable Timer 0 Compare A ISR
	TCCR0B = 3;								// Set Prescaler to 64 & Timer 0 starts
}

void InitTimer1(void){						// 1 sec timer with 16bit precision	 
	TCCR1A |= (1<<WGM10) | (1<<WGM11);		// enabling fast PWM mode
	TCCR1B |= (1<<WGM12) | (1<<WGM13);		// enabling fast PWM mode
	OCR1A = 1999;							// 1ms = (8*(1+1999))/16E6
	TCCR1B |= (1<<CS11);					//setting prescaler to 8
	OCR1B = 1000;							// half of OCR1A
	TIMSK1 |= (1<<OCIE1A);					// Enable Timer 1 Compare A ISR
	TIMSK1 |= (1<<OCIE1B);					// Enable timer 1 compare B ISR
	TCCR1A |= (1<<COM1B1) | (0<<COM1B0);	// non-inverting mode
}

///////////////////////////////////////////////////////////////////INIT CODE//////////////////////////////////
void SPI_MasterInit(void){												//uses SPI0
	SPI_DDR = (1<<SPI_MOSI) | (1<<SPI_SCK);								//Set MOSI and SCK output, all others input
	SPCR0 = (1<<SPE) | (1<<MSTR) | (1<<SPR0) | (1<<CPOL) | (1<<CPHA);	//Enable SPI, Master, set clock rate fck/8, set SPI mode 3
	
	//SPI0 PINOUT
	//PB4 = MISO
	//PB5 = SCK
	//N/A = RST
	//Vcc = Vcc
	//PB3 = MOSI
	//GND = GND
}

uint8_t SPI_Master_Transceiver(uint8_t cData)
{
	SPDR0 = cData;					// Start transmission, highest 8bits first
	while( !(SPSR0 & (1<<SPIF)) );	// Wait for transmission complete
	return SPDR0;					// Return received data
}


////////////////////////////////////////////////////////////////////MAIN LOOP///////////////////////////////////////

int main(void)
{
    DDRD = 0b11111000; //motor controller output, software PWM pins
	DDRE = 0b00001110; //RGB LED pins
	sei();
	SPI_MasterInit();	//Pixycam vision processing camera
	uart_init(0);		//debug
	InitTimer0();		//camera refresh and logic timing
	InitTimer1();		//PWM signals for motor control and indicator light
	
	printf("\nprogram is running\n");
	uint8_t x;
	
	OCR1B = 1000; //can be used to vary motor speed, left static in this version
	
	
    while (1) 
    {
		if (cameraRefresh <= 5){
			//get 1 block's data
			//packet reference here: https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:protocol_reference
			
			x = SPI_Master_Transceiver(0xae);		//16 bit sync first half, fixed number
			printf("%d\n", x);
			x = SPI_Master_Transceiver(0xc1);		//16bit sync second half, fixed number
			printf("%d\n", x);
		
			x = SPI_Master_Transceiver(32);			//type of packet, fixed number
			printf("%d\n", x);
			x = SPI_Master_Transceiver(2);			//length of payload, fixed number
			printf("%d\n", x);
		
			x = SPI_Master_Transceiver(1);			//which block signature to return
			printf("%d\n", x);
			x = SPI_Master_Transceiver(1);			//number of blocks to return
			printf("%d\n", x);
		
			for (int i = 0; i < 7; i++){			//data returned on what the Pixycam detected
				
				x = SPI_Master_Transceiver(0);
				printf("%d\n", x);
				//the pixycam's camera has a resolution of 315,207. To account for a number larger than 255 there's 2 packets for the x-coordinate
				
				if (i == 2){
					turn = x;						//x-coordinate of detected block
				}
				if (i == 3){
					turn10 = x;						//overflow of x-coordinate, treated as a bool
				}
				if (i == 4){						//y-coordinate of detected block
					fstop = x;
				}
			}
			printf("---round over---\n");
			printf("botstate: %x\n", botState);
			cameraRefresh = 100;					//delay added to SPI to not overburden hardware and for ease of debugging
		}		
    }
}

