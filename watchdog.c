/*
 * File:   watchdog.c
 * Author: nemet
 *
 * Created on 2023. március 14., 1:35
 */

// PIC12F1822 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <pic12f1822.h>
#define _XTAL_FREQ 500000
#define LOW_TRESH 100
#define HIGH_TRESH 100
unsigned int timer;
void main(void) {
    TRISAbits.TRISA2    = 0;//digital output
    TRISAbits.TRISA4    = 1;//analog input
    TRISAbits.TRISA5    = 1;//digital input
    ADCON0bits.CHS      = 0b00011;//an3
    ADCON0bits.ADON = 1;
    ADCON1bits.ADPREF = 0b11;
    FVRCONbits.ADFVR = 0b10;//2048mV
    FVRCONbits.FVREN = 1;
    
    
    while(1)
    {
        timer++;
        if(timer > 100)
        {
            timer = 0;
            if(ADRESH > 128)
            {
                LATAbits.LATA2 = 1;
            }
            else
            {
               LATAbits.LATA2 = 0; 
            }
            ADCON0bits.ADGO = 1;
        }
        __delay_ms(1);
    }
            
    return;
}
