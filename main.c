#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/iom32.h>	
#include <util/delay.h>
#include <avr/sleep.h>

#include "lcd.h"



#define F_CPU 8000000UL			/* Define CPU Frequency 8MHz */
#include <avr/io.h>			/* Include AVR std. library file */
#include <stdio.h>			/* Include Std. i/p o/p file */
#include <string.h>			/* Include String header file */


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


uint8_t count1, count2, count3; // master value
uint8_t count1_s, count2_s, count3_s; 
volatile uint16_t sum_first_device = 0; // sum 0-8 
volatile uint16_t sum_second_device = 0; //sum 8-16


ISR(INT0_vect){

    //guzik  pierwszy - potwierdzenie lvlu

}
ISR(INT1_vect){

    //guzik drugi

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



int main(int arc, char *argv[]){




    INIT_PC0;
    INIT_PC7;
    LCD_Init();			/* Initialization of LCD*/
    STOP_TRANSITION_C0;
    STOP_TRANSITION_C7;

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

    int c = UART_RxChar();
    char buffer[8];
    sprintf(buffer, "%d", c);


    

       if(c==48){
                
            LCD_String("  Connected to " );	/* Write string on 1st line of LCD*/
            LCD_Command(0xC0);		/* Go to 2nd line*/
            LCD_String("     device");	/* Write string on 2nd line*/
        } 
        if(c==49){
                
            LCD_String("  Upload to " );	/* Write string on 1st line of LCD*/
            LCD_Command(0xC0);		/* Go to 2nd line*/
            LCD_String("     device");	/* Write string on 2nd line*/
        } 
        if(c==50){
                
            LCD_String(" Waiting " );	/* Write string on 1st line of LCD*/
            LCD_Command(0xC0);		/* Go to 2nd line*/
            LCD_String(" for device");	/* Write string on 2nd line*/
        } 

     

    while(1);

    return 0; 

}