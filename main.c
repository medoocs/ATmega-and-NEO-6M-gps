#define F_CPU 8000000UL

#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include "lcd.h"

#define USART_BAUDRATE 9600	// necemo koristiti ovu varijablu, ali to je BAUDRATE koji koristimo za komunikaciju s modulom

static uint8_t flag = 1;	
static uint8_t ready = 0;	
static char msg[256];	
int count = 0;	
static char value;	

char lock[2] = {0};		
char sats[3];		
char latitude[15];
char longitude[15];		
char alt[6];	
char time[15];	
char angle[10];		
char speed[6];		
char ns[2];		
char ew[2];		
char a[15];		

void usart_init(){		// funkcija za inicijalizaciju USART protokola
	UCSRB =  (1 << RXEN) | (1 << TXEN);	
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); 
	UBRRL = 51;		
	UBRRH = 0;		
}					
unsigned int usart_getch(){	
	while (!(UCSRA & (1 << RXC)));	
	return(UDR);	
}
void db(){	//funkcija za non blocking debounce
	GICR &= ~_BV(INT0);
	GICR &= ~_BV(INT1); 
	sei();
	_delay_ms(200);
	GIFR = _BV(INTF0)| _BV(INTF1);
	GICR |= _BV(INT0);
	GICR |= _BV(INT1);
	cli();
}

ISR(INT0_vect){	//INT0 promjena prikaza unaprijed
	if(ready == 1){
		flag++;
		if(flag > 6){
			flag = 1;
		}	
	}
	db();
}

ISR(INT1_vect){	//INT1 promjena prikaza unatrag
	if(ready == 1){
		flag--;
		if(flag < 1){
			flag = 6;
		}
	}
	db();
}

void getstuff(){	//funkcija koja sprema sve charove stringa u buffer
	value = usart_getch();	
	while(value != '\n'){	
		msg[count] = value;
		count++;
		value = usart_getch();
	}
	count = 0;	
}

void getmessage(){	// funkcija koja parsa buffer i sprema u varijable
	if(strstr(msg,"$GPGGA") != NULL){	 
		sscanf(msg, "$GPGGA,%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,]", time, latitude, ns, longitude, ew, lock, sats, a, alt, a, a);
	}else if(strstr(msg,"$GPRMC") != NULL){	
		sscanf(msg, "$GPRMC,%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,]", a, a, a, a, a, a, speed, angle, a, a, a);
	}														
}

void check_ready(){	//funkcija koja pregledava je li GPS spojen
	if(strcmp(lock, "") == 0){	
		ready = 0;	
		lcd_clrscr();
		lcd_puts("GPS nije spojen");	
	}else{
		ready = 1;	
	}	
}
void print_time(){	//funkcija koja ispisuje vrijeme
	char data[16];
	data[0] = time[0];
	data[1] = time[1];
	data[2] = ':';
	data[3] = time[2];
	data[4] = time[3];
	data[5] = ':';
	data[6] = time[4];
	data[7] = time[5];
	data[8] = '.';
	data[9] = time[7];
	data[10] = time[8];
	data[11] = time[9];
	data[12] = '\0';
	
	
	lcd_clrscr();
	lcd_gotoxy(0,0);
	lcd_puts("UTC vrijeme");
	lcd_gotoxy(0,1);
	lcd_puts(data);
	
}
void print_coordinates(){	//funkcija koja ispisuje koordinate
	char data[16];
	char data1[16];
	data[0] = latitude[0];
	data[1] = latitude[1];
	data[2] = (char)0xDF;	
	data[3] = latitude[2];
	data[4] = latitude[3];
	data[5] = latitude[4];
    data[6] = latitude[5];
	data[7] = latitude[6];
	data[8] = latitude[7];
    data[9] = latitude[8];
	data[10] = latitude[9];
	data[11] ='\0';
	lcd_clrscr();
	lcd_gotoxy(0,0);
	lcd_puts(data);
	lcd_gotoxy(12,0);
	lcd_putc('"');
	lcd_gotoxy(14,0);
	lcd_puts(ns);	

	data1[0] = longitude[1];
	data1[1] = longitude[2];
	data1[2] = (char)0xDF;	
	data1[3] = longitude[3];
	data1[4] = longitude[4];
	data1[5] = longitude[5];
	data1[6] = longitude[6];
	data1[7] = longitude[7];
	data1[8] = longitude[8];
	data1[9] = longitude[9];
	data1[10] = longitude[10];
	data1[11] ='\0';
	lcd_gotoxy(0,1);
	lcd_puts(data1);
	lcd_gotoxy(12,1);
	lcd_putc('"');
	lcd_gotoxy(14,1);
	lcd_puts(ew);	
}

void print_alt(){	//funkcija koja ispisuje nadmorsku visinu
	lcd_clrscr();
	lcd_gotoxy(0,0);
	lcd_puts("Visina");
	lcd_gotoxy(0,1);
	lcd_puts(alt);
	lcd_gotoxy(15,1);
	lcd_puts("M");
}

void print_fix(){	//funkcija koja ispisuje je li spojen na GPS, i koliko je GPS-a trenutno spojeno
	lcd_clrscr();
	lcd_gotoxy(0,0);
	lcd_puts("SVs");
	lcd_gotoxy(5,0);
	lcd_puts(sats);	
	lcd_gotoxy(0,1);
	lcd_puts("FIX");
	lcd_gotoxy(5,1);
	lcd_puts(lock);	
}

void print_speed(){	//funkcija koja ispisuje trenutnu brzinu
	lcd_clrscr();
	lcd_gotoxy(0,0);
	lcd_puts("Brzina");
	lcd_gotoxy(0,1);
	lcd_puts(speed);
	lcd_gotoxy(11,1);
	lcd_puts("knots");
}

void print_angle(){	//funkcija koja ispisuje azimut kuta
	lcd_clrscr();
	lcd_gotoxy(0,0);
	lcd_puts("Kut");
	lcd_gotoxy(0,1);
	lcd_puts(angle);
	lcd_gotoxy(15,1);
	lcd_putc((char)0xDF);	
}

int main(void)
{	PORTB = 0x0f;
	DDRB = 0x00;
	DDRD = _BV(4);

	TCCR1A = _BV(COM1B1) | _BV(WGM10);
	TCCR1B = _BV(WGM12) | _BV(CS11);
	OCR1B = 60;
	MCUCR = _BV(ISC01)|_BV(ISC11);
	GICR = _BV(INT0)| _BV(INT1);
		
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	lcd_gotoxy(0,0);
	
	usart_init();	//inicjalizacija USART protokola
	sei();
	   
    while (1) 
    {
		getstuff();                                  
		getmessage();
		check_ready();
		if(ready == 1){
			if(flag == 1){
				print_time();
			}else if(flag == 2){
				print_coordinates();
			}else if(flag == 3){
				print_alt();
			}else if(flag == 4){
				print_speed();
			}else if(flag == 5){
				print_angle();
			}else if(flag == 6){
				print_fix();
			}
		}
	 }
}





