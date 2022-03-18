#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/iom32.h>	
#include <util/delay.h>
#include <avr/sleep.h>
#include "lcd.h"
#include <avr/io.h>			/* Include AVR std. library file */
#include <stdio.h>			/* Include Std. i/p o/p file */
#include <string.h>			/* Include String header file */
#include <unistd.h>

#define F_CPU 8000000UL			/* Define CPU Frequency 8MHz */
#define MOSI 5								/* Define SPI bus pins */
#define MISO 6
#define SCK 7
#define SS 4
#define SS_Enable PORTB &= ~(1<<SS)			/* Define Slave enable */
#define SS_Disable PORTB |= (1<<SS)			/* Define Slave disable */

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)


/*CS for transducers*/
#define INIT_PC0     DDRC |= (1<<PC0)
#define INIT_PC7     DDRC |= (1<<PC7)

#define STOP_TRANSITION_C0 PORTC |=(1<<PC0) 
#define START_TRANSITION_C0 PORTC &= ~(1<<PC0)

#define STOP_TRANSITION_C7 PORTC |=(1<<PC7)
#define START_TRANSITION_C7 PORTC &= ~(1<<PC7)


#define INIT_SW_A DDRD &= ~(1<<PD2) //wejscie
#define SW_CONFIG_PULLUP_A PORTD |= (1<<PD2) //wewnetrzny pullup

#define INIT_SW_B DDRD &= ~(1<<PD3) //wejscie
#define SW_CONFIG_PULLUP_B PORTD |= (1<<PD3) //wewnetrzny pullup

#define INIT_SW_C DDRB &= ~(1<<PB2) //wejscie
#define SW_CONFIG_PULLUP_C PORTB |= (1<<PB2) //wewnetrzny pullup

uint8_t count1, count2, count3; // master value
uint8_t count1_s, count2_s, count3_s; 
volatile uint16_t sum_first_device = 0; // sum 0-8 
volatile uint16_t sum_second_device = 0; //sum 8-16
volatile uint8_t channel_i = 1; //choose cgannel
volatile uint8_t start_program = 0; 
volatile uint8_t accept = 0;
volatile int c = 0; // data from python 


ISR(INT0_vect){

    channel_i++;
    if(channel_i == 10){
        channel_i = 1;
    }
}

ISR(INT1_vect){

    start_program++;
}

ISR(INT2_vect){

    accept++;
}

void UART_init()
{
	UCSRB |= (1 << RXEN) | (1 << TXEN);/* Turn on transmission and reception */
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);/* Use 8-bit character sizes */
    UBRRL = BAUD_PRESCALE;		/* Load lower 8-bits of the baud rate value */
	UBRRH = (BAUD_PRESCALE >> 8);	/* Load upper 8-bits*/

}


void SPI_Init_Master()					
{
	DDRB |= (1<<MOSI)|(1<<SCK)|(1<<SS);	//  MOSI, SCK, SS jako wyjscie
			
	DDRB &= ~(1<<MISO);			// MISO jako wejscie
						
	SS_Disable;
	SPCR |= (1<<SPE)|(1<<MSTR)|(1<<SPR0);	//  Fosc/16
}


char SPI_Write_Master(char data)		/* SPI write data function */
{
	uint8_t a;
	SPDR = data;			/* Skopiowanie danych do przeslania */
	while(!(SPSR & (1<<SPIF)));	/* Czekanie na koniec transmisji */
    a=SPDR;
	return a;		/* Skopiowanie danych */
}


char SPI_Read_Master()				
{
	SPDR = 0xFF;
	while(!(SPSR & (1<<SPIF)));	// czekanie na przyjecie danych
	return(SPDR);			// zwrocenie otrzymanej danej
}


void UART_TxChar(uint8_t ch)
{
	while (! (UCSRA & (1<<UDRE)));	/* Wait for empty transmit buffer*/
	UDR = ch ;
}

unsigned char UART_RxChar()
{
	while ((UCSRA & (1 << RXC)) == 0);/* Wait till data is received */
	return(UDR);			/* Return the byte*/
}


void Chose_Channel(char channel){

    switch(channel)
    {
        case 0:
            count1 = 0x06;
            count2 = 0x00;
            count3 = 0x00;
            break;


        case 1: 
            count1 = 0x06;
            count2 = 0x40;
            count3 = 0x00;
            break;

        case 2:
            count1 = 0x06;
            count2 = 0x80;
            count3 = 0x00;
            break;

        case 3:

            count1 = 0x06;
            count2 = 0xC0;
            count3 = 0x00;
            break;
        
        case 4:
            count1 = 0x07;
            count2 = 0x00;
            count3 = 0x00;
            break;

        case 5:
            count1 = 0x07;
            count2 = 0x40;
            count3 = 0x00;            
            break;

        case 6:
            count1 = 0x07;
            count2 = 0x80;
            count3 = 0x00;
            break;

        case 7:
            count1 = 0x07;
            count2 = 0xC0;
            count3 = 0x00;
            break;

    }

}

uint8_t change_byte(uint8_t number){

    return number & 0x0F; 

}

void UART_SendString(uint16_t number)
{	     
    UART_TxChar(number >> 8);
	UART_TxChar(number & 0x00FF);
    UART_TxChar('\n');
}

void FirstDeviceEnable()
{
    START_TRANSITION_C0;
}

void SecondDeviceEnable()
{
    START_TRANSITION_C7;
}

void FirstDeviceDisable()
{
    STOP_TRANSITION_C0;
}

void SecondDeviceDisable()
{
    STOP_TRANSITION_C7;
}

void SendFirstData(){

    uint8_t num =0;
    uint16_t sum1=0; 
    uint16_t sum2=0;

       for(int k=0; k<8; k++){

        Chose_Channel(k);
   
        FirstDeviceEnable();

        SPI_Write_Master(count1);
        count2_s = SPI_Write_Master(count2);
        count3_s = SPI_Write_Master(count3);

        FirstDeviceDisable();
       
        num = change_byte(count2_s);
        UART_TxChar(num);
        UART_TxChar(count3_s);
        UART_TxChar('\n');

        sum1 = sum1 + num; 
        sum2 = sum2 + count3_s; 
          
    }
 
    sum_first_device = sum2 + sum1*256;

}

void SendSecondData(){

    uint8_t num =0;
    uint16_t sum1=0; 
    uint16_t sum2=0;


       for(int k=0; k<8; k++){

        Chose_Channel(k);
        SecondDeviceEnable();

        SPI_Write_Master(count1);
        count2_s = SPI_Write_Master(count2);
        count3_s = SPI_Write_Master(count3);

        SecondDeviceDisable();
        


        num = change_byte(count2_s);
        UART_TxChar(num);
        UART_TxChar(count3_s);
        UART_TxChar('\n');

        sum1 = sum1 + num; 
        sum2 = sum2 + count3_s; 
 
    }

    sum_second_device = sum2 + sum1*256;
   
}


char askLevel(int lvl){

    switch(lvl)
    {
        case 1:
         return '1';

        case 2:
         return '2';

        case 3:
         return '3'; 

        case 4:
         return '4';

        case 5:
         return '5';

        case 6: 
         return '6';

        case 7:
         return '7';

        case 8:
         return '8';

        case 9:
         return '9';

        case 10:
         return '10';
    }
}

void lcd_option(int b){

    char buffer1[2];
    sprintf(buffer1, "%d", channel_i);    


    //0
    if(b==48){
                
        LCD_String("    Hello " );	/* Write string on 1st line of LCD*/
        LCD_Command(0xC0);		/* Go to 2nd line*/
        LCD_String("     user! ");	/* Write string on 2nd line*/
    } 
    //1
    if(b==49){
                
        LCD_String(" Waiting for  " );	/* Write string on 1st line of LCD*/
        LCD_Command(0xC0);		/* Go to 2nd line*/
        LCD_String("    device...");	/* Write string on 2nd line*/
    } 
    //2
    if(b==50){
                
        LCD_String("   Device " );	/* Write string on 1st line of LCD*/
        LCD_Command(0xC0);		/* Go to 2nd line*/
        LCD_String(" is working ");	/* Write string on 2nd line*/
    }
    
    //3
    if(b==51){
         
        LCD_String(" Choose lvl:" );	/* Write string on 1st line of LCD*/
        LCD_Command(0xC0);		/* Go to 2nd line*/
        LCD_String(buffer1);	/* Write string on 2nd line*/
    } 
    //4
    if(b==52){
             
        LCD_String("You chose lvl:" );	/* Write string on 1st line of LCD*/
        LCD_Command(0xC0);		/* Go to 2nd line*/
        LCD_String(buffer1);	/* Write string on 2nd line*/
    } 

}


int main(){

    INIT_PC0;
    INIT_PC7;
    INIT_SW_A;
    INIT_SW_B;
    INIT_SW_C;
    LCD_Init();			/* Initialization of LCD*/
    STOP_TRANSITION_C0;
    STOP_TRANSITION_C7;
    SW_CONFIG_PULLUP_A;
    SW_CONFIG_PULLUP_B;
    SW_CONFIG_PULLUP_C;

    sei();

    
    /*generacja przerwan - guziki*/
	MCUCR |= ((1<<ISC01) | (1<<ISC11)); //sposob generacji przerwania
    MCUCSR |=(1<<ISC2);
    GICR |=((1<<INT0) | (1<<INT1) | (1<<INT2));


	UART_init();
    SPI_Init_Master();

    
    UART_TxChar(0xFF); //start bit
    UART_TxChar('\n');
    UART_TxChar(0x10); // number of channels
    UART_TxChar('\n');

 
    SendFirstData();

   STOP_TRANSITION_C0;
   STOP_TRANSITION_C7;

   SendSecondData();

   UART_SendString(sum_first_device); 
   UART_SendString(sum_second_device);

   UART_TxChar(0xFF); // stop bit
   UART_TxChar('\n');    

    lcd_option(48); //hello user
    _delay_ms(5000);
    LCD_Clear();

    lcd_option(49); //waiting for device
    _delay_ms(5000);

    //while(start_program == 0);

    c = UART_RxChar();

    LCD_Clear();
    lcd_option(50); //device is working
    _delay_ms(5000);
    LCD_Clear();

    lcd_option(51); //choose lvl   
    _delay_ms(3000);
    
    while(accept == 0){
        _delay_ms(1000);
    }
    
    LCD_Clear();
    lcd_option(52); 
     _delay_ms(3000);

    c = UART_RxChar();
    char buffer[8];
    sprintf(buffer, "%d", c);


    while(1);

    

    return 0; 

}