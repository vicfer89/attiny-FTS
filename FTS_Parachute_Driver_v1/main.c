/*
 * FTS_Parachute_Driver - Código de driver de parachute para FTS Avanzado
 *
 * Created: 19/04/2016
 *	 Author : VFF
 *	 Company: Aertec Solutions
 */ 

#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


void SystemConfig(void);
int ADCRead(void);
void PWMWrite(int pwmval);
void PWMEnable(void);
void PWMDisable(void);
int milis;
int contador_vuelta = 0;

typedef enum _Modo_type
{
	Waiting,
	Deploy
}Modo_type;

Modo_type Modo = Waiting;

int ADCLectura = 0;

int main(void)
{	

	cli();
	SystemConfig();	
	PORTB &= ~(1 << PB3);
	PORTB &= ~(1 << PB4);
	cli();		

	ADCLectura = ADCRead();		
	if(ADCLectura < 810)
	{
		while (1)
		{
			PORTB |= (1 << PB4);
			_delay_ms(100);
			PORTB &= ~(1 << PB4);
			_delay_ms(100);	
		}				
	}
	else
	{
		PORTB |= (1 << PB4);
		PORTB |= (1 << PB3);
		PWMEnable();				
		PWMWrite(31); // Servo at initial position (16 previous)
		_delay_ms(1500);				
		PORTB &= ~(1 << PB4);
		PORTB &= ~(1 << PB3);
		PWMDisable();		
	}
	//PORTB &= ~(1<<PB1); // Elimino Pull-UP
	
    while (1) 
    {
		switch (Modo)
		{
		
			case Waiting:
			{
				if(!(PINB & (1 << PINB1)))
				{
					milis = 0;
					do 
					{
						
						if( ( PINB & (1 << PINB1)) )
						{
							Modo = Waiting;
						}
						else
						{
							Modo = Deploy;
						}
						milis++;
						_delay_ms(1);
					} while (milis < 1000);
					
				}
				else
				{
					Modo = Waiting;
				}
			
			}
			break;
		
			case Deploy:
			{
				PORTB |= (1 << PB3);
				PWMEnable();
				PWMWrite(16);
				_delay_ms(1500);				
				PORTB &= ~(1 << PB3);
				PWMDisable();
				_delay_ms(500);
				do
				{
					if( ( PINB & (1 << PINB1)) )
					{						
						if (contador_vuelta == 10)						
						{
							PORTB |= (1 << PB4);
							PORTB |= (1 << PB3);
							PWMEnable();
							PWMWrite(31); // Servo at initial position (16 previous)
							_delay_ms(1500);
							PORTB &= ~(1 << PB4);
							PORTB &= ~(1 << PB3);
							PWMDisable();
							contador_vuelta = 0;							
							Modo = Waiting;							
							
						}
						else
						{
							contador_vuelta++;							
						}
					}					
					else
					{
						contador_vuelta = 0;
						PORTB |= (1 << PB4);
						_delay_ms(500);
						PORTB &= ~(1 << PB4);
						_delay_ms(500);					
					}
				}while(Modo == Deploy);
			}
			break;		
		}
    }
}

void SystemConfig(void)
{
	
	/************************************************************************/
	/* GPIO Configuration                                                   */
	/************************************************************************/			
	DDRB |= (1<<DDB0) | (1<<DDB3) | (1<<DDB4); // Pin PB0 configured as output (PWM Output), Pin PB3 configured as output (SSR), Pin PB4 configured as output (Status LED)
	DDRB &= ~(1<<DDB1); // PB1 Configured as input	
	asm("NOP"); // Synchronization
	/************************************************************************/
	/* Analog input configuration                                           */
	/************************************************************************/
	ADMUX |= (1<<MUX0); // ADC input set to PB2 using the multiplexer		
	ADCSRA |= (1<<ADEN); // Enables ADC
	asm("NOP"); 	
}

int ADCRead(void)
{
	ADCSRA |= (1 << ADSC); // Start reading
	while((ADCSRA & (1<<ADSC))); // Wait until reading was performed	
	return (int) ((ADCH<<8) | ADCL); // Return result
}

void PWMWrite(int pwmval)
{
	// 60 equals to 90º and 30 equals to 0º, so the line is: pwmval = (1/3)*angle+30
	OCR0A = pwmval;
}

void PWMEnable(void)
{
	TCCR0B |= (1<<CS01) | (1<<CS00); // Set prescaler at f_clk/1024.
	TCCR0A |= (1<< WGM01) | (1<<WGM00) | (1<<COM0A1); // Set to FAST PWM mode	
}

void PWMDisable(void)
{
	TCCR0A &= ~(1<<COM0A1);
}



