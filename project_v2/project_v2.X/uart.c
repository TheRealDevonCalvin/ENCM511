/*
 * File:   uart.c
 * Author: psbta
 *
 * Created on September 27, 2025, 8:07 PM
 */


#include "uart.h"

uint8_t received_char = 0;
uint8_t RXFlag = 0;
QueueHandle_t uart_rx_queue;

void InitUART2(void) 
{

    RPINR19bits.U2RXR = 11; // uart receive
    RPOR5bits.RP10R = 5;    // uart transmit

    U2MODE = 0b0000000010001000;    // high speed

    U2BRG = 103;            // ~9600 baud
    
	U2STAbits.UTXISEL0 = 0;
    U2STAbits.UTXISEL1 = 0;
    U2STAbits.URXEN = 1;
    U2STAbits.UTXEN = 1;
    U2STAbits.URXISEL = 0b00;

	IFS1bits.U2TXIF = 0;	
    IPC7bits.U2TXIP = 3; 
    
	IEC1bits.U2TXIE = 1; 
	IFS1bits.U2RXIF = 0; 
	IPC7bits.U2RXIP = 4; 
    IEC1bits.U2RXIE = 1;

	U2MODEbits.UARTEN = 1;	

	U2STAbits.UTXEN = 1;
     
	return;
}

void Disp2String(char *str) //Displays String of characters
{
    unsigned int i;
    for (i=0; i<= strlen(str); i++)
    {
        XmitUART2(str[i],1);
    }
    return;
}

void XmitUART2(char CharNum, unsigned int repeatNo)
{	
	U2STAbits.UTXEN = 1;
	while(repeatNo!=0) 
	{
		while(U2STAbits.UTXBF==1) 
		{
		}	
		U2TXREG=CharNum;            
		repeatNo--;
	}
	while(U2STAbits.TRMT==0)       
	{
	}

    U2STAbits.UTXEN = 0;
}

void Disp2Dec(uint16_t Dec_num) {
    uint8_t rem; //remainder in div by 10
    uint16_t quot;
    uint8_t ctr = 0; //counter
    XmitUART2(' ',1); // Disp Gap
    while(ctr<5) {
        quot = Dec_num/(pow(10,(4-ctr)));
        rem = quot%10;
        XmitUART2(rem + 0x30 , 1);
        ctr = ctr + 1;
    }
    XmitUART2(' ',1); // Disp Gap
    return;
}

void int_to_str_dec(uint16_t dec, char *str){
    // a function that converts a decimal number to a string. this allows it to be used by the UART
    int i = 0;
    if(dec == 0){
        // if the number is just zero, put that in the string and move on
        str[i] = '0';
        i++;    // dont forget to increment the index variable!
    }
    else{
        while(dec > 0){
            // while the number is more than 0, we modulo off the digits and put them in the string
            str[i++] = (dec % 10) + '0';
            dec /= 10;
        }
    }
    
    int start = 0;
    int end = i - 1;
    while(start < end){
        // this loop just reverses the string so it's the right order
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end --;
    }
    str[i] = '\0';  // null-terminate the string 
    return; 
}

void __attribute__ ((interrupt, no_auto_psv)) _U2RXInterrupt(void) {
    /*
     * This UART RX ISR is used to collect and queue user keyboard inputs
     */

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t c;      // variable for char storage
    
    c = U2RXREG;    // read the UART2 receive register for the char
    
    xQueueSendFromISR(uart_rx_queue, &c, &xHigherPriorityTaskWoken);    // send to queue for use in main
    
    IFS1bits.U2RXIF = 0;    // clear interrupt flag
    RXFlag = 1;             // set rx flag (used in other uart functions, like recvuart)
    
    if(xHigherPriorityTaskWoken){
        portYIELD();        // context switch
    }    
}

void __attribute__ ((interrupt, no_auto_psv)) _U2TXInterrupt(void) {
	IFS1bits.U2TXIF = 0;

}




/*--------------------------Functions Given but Not Used-----------------------------*/

/************************************************************************
 * Receive a buf_size number of characters over UART
 * Description: This function allows you to receive buf_size number of characters from UART,
 * provided the UART module has been initialized correctly. The function currently returns
 * if the "enter" key (ASCII 0x0D) is received. The function does not handle receiving
 * the DELETE or BACKSPACE keys meaningfully. 
 * 
 * Note: there is commented-out skeleton code that could be (re)implemented to allow the function
 * return early without the ENTER key given an interrupt-set global flag. 
 ************************************************************************/
void RecvUart(char* input, uint8_t buf_size)
{	
    uint16_t i = 0;
    char last_char;
    // wait for enter key
    while (last_char != 0x0D) {
        if (RXFlag == 1) {
            // only store alphanumeric characters
            if (received_char >= 32 && received_char <= 126) {
                if (i > buf_size-2) {
                    Disp2String("\ntoo long\n\r");
                    RXFlag = 0;
                    return;
                }
                input[i] = received_char;
                i++;
                XmitUART2(received_char,1); // loop back display
                U2STAbits.OERR = 0;
            }
            last_char = received_char;
            RXFlag = 0;
        }
        // wait for next character
        
        // if (CNflag == 1) { // this allows breaking out of the busy wait if CN interrupts are enabled...
        //     add logic here
        // }
    }
}

/************************************************************************
 * Receive a single (alphanumeric) character over UART
 * Description: This function allows you to receive a single character, which will only be 
 * "accepted" (returned) after receiving the ENTER key (0x0D). 
 * While receiving characters, the program is designed to send back the received character.
 * To display this, it sends a BACKSPACE (0x08) to clear the previous character from the 
 * receiving terminal, before sending the new current character. 
 * 
 * Note: there is commented-out skeleton code that could be (re)implemented to allow the function
 * return early without the ENTER key given an interrupt-set global flag. 
 ************************************************************************/
char RecvUartChar()
{	
    char last_char;
    XmitUART2(' ',1);
    // wait for enter key
    while (last_char != 0x0D) {
        if (RXFlag == 1) {
            
            // return the last character received if you see ENTER
            if (received_char == 0x0D) {
                RXFlag = 0;
                return last_char;
            }
            
            // only store alphanumeric characters
            if (received_char >= 32 && received_char <= 126) {
                XmitUART2(0x08,1); // send backspace
                last_char = received_char;
                XmitUART2(received_char,1); // loop back display
            }
           
            U2STAbits.OERR = 0;
            RXFlag = 0;
        }
        
        // if (CNflag == 1) { // this allows breaking out of the busy wait if CN interrupts are enabled...
        //     add logic here
        // }
    }
}