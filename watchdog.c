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
#include <string.h>
#define _XTAL_FREQ 500000
#define LOW_TRESH 125//feszultseg csokkenes utan reset
#define HIGH_TRESH 146
#define CHARGE_LOW_TRESH 160//4 voltos tolto lekapcsolas
#define CHARGE_HIGH_TRESH 158

//256 érték mellett 160nál lekapcs 158nál visszakapcs
//3mp nincs low akkor 2mp reset

#define NEXT                      0x01
#define PREV                      0x02
#define SPEC_VOL                  0x06
#define SPEC_EQ                   0x07
#define SPEC_PLAYBACK_MODE        0x08
#define SPEC_PLAYBACK_SRC         0x09
#define PAUSE                     0x0E
#define PLAY                      0x0D
#define REPEAT_PLAY               0x11

#define HANGERO 16



long timer,pintimer;
unsigned int laststate, actstate,resettimer;
unsigned char adcreset,pinreset,sendcommands;
unsigned char buffer[20];


void WriteSerial(unsigned char *buffer,uint8_t len)
    {
    uint8_t i;
    for(i = 0;i < len;i++)
    {
        TXREG = *buffer++;
        while(TXSTAbits.TRMT == 0);
    }
    }
void SendCommand(uint8_t command,uint8_t par1,uint8_t par2)
{
    int16_t checksum;
    buffer[0] = 0x7E;//start byte
    buffer[1] = 0xff;//version
    buffer[2] = 0x06;//length
    buffer[3] = command;//command
    buffer[4] = 0;//feedback, no
    buffer[5] = par1;//parameter high byte
    buffer[6] = par2;//parameter low byte
    checksum = 1-(buffer[1]+buffer[2]+buffer[3]+buffer[4]+buffer[5]+buffer[6]);
    checksum = checksum-1;
    buffer[7] = checksum>>8;//checksum high byte
    buffer[8] = checksum&0xff;//checksum low byte
    buffer[9] = 0xEF;//end byte
    WriteSerial(&buffer,10);
    
}

void main(void) {
    TRISAbits.TRISA2    = 0;//digital output,MP3 RESET, p-s fetet kapcsol
    TRISAbits.TRISA0    = 0;//TX
    TRISAbits.TRISA5    = 0;//digital output,CHIP ENABLE, high:tolt, low: nem tolt
    TRISAbits.TRISA1    = 1;//digital input//MP3BUSY
    TRISAbits.TRISA4    = 1;//analog input
    ANSELAbits.ANSA1    = 0;
    ANSELAbits.ANSA0    = 0;
    ADCON0bits.CHS      = 0b00011;//an3
    ADCON0bits.ADON     = 1;
    ADCON1bits.ADPREF   = 0b11;
    FVRCONbits.ADFVR    = 0b10;//2048mV
    FVRCONbits.FVREN    = 1;
           
    TXSTAbits.BRGH = 1;
    TXSTAbits.TXEN = 1;
    RCSTAbits.SPEN = 1;
    BAUDCONbits.BRG16 = 1;
    SPBRGH = 0;
    SPBRGL = 12;
    
    
    __delay_ms(2000);
    SendCommand(SPEC_VOL,0,HANGERO);
    __delay_ms(200);
    SendCommand(REPEAT_PLAY,0,1);
    __delay_ms(500);
    SendCommand(SPEC_VOL,0,HANGERO);
    pintimer = 0;
    while(1)
    {
        timer++;
        
        if(timer >100)
        {
            ADCON0bits.ADGO = 1;
            while (ADCON0bits.nDONE != 0);
            if(ADRESH < LOW_TRESH)
            {
                adcreset = 1;
            }
            if(ADRESH > HIGH_TRESH)
            {
                adcreset = 0;
            }
            if(ADRESH < CHARGE_LOW_TRESH)
            {
                LATAbits.LATA5 = 1;//reset state//CE
            }
            if(ADRESH > CHARGE_HIGH_TRESH)
            {
                LATAbits.LATA5 = 0;//normal state//CE
            }
            
            if(PORTAbits.RA1 == 1)//device is not doing anything 
            {
               pintimer++; 
            }
            if(pintimer >= 30)
            {
                resettimer = 20;
                pinreset = 1;
                pintimer = -20;
            }
            if(resettimer > 0 )
            {
                resettimer--;
            }
            if(resettimer == 0)
            {
                pinreset = 0;
            }
            if( pinreset || adcreset)
            {
                LATAbits.LATA2 = 1;//reset state//RESET
                sendcommands = 1;
            }
            else
            {
                LATAbits.LATA2 = 0;//normal state//RESET
                if(sendcommands)
                {
                    __delay_ms(500);
                    SendCommand(SPEC_VOL,0,HANGERO);
                    __delay_ms(5.00);
                    SendCommand(REPEAT_PLAY,0,1);
                    __delay_ms(500);
                    SendCommand(SPEC_VOL,0,HANGERO);
                    sendcommands = 0; 
                }
            }
            timer= 0;
        }
        __delay_ms(1);
    }
            
    return;
}
//  timer++;
//        pintimer++;
//        if(timer > 100)
//        {
//            timer = 0;
//            if(ADRESH > 147)
//            {
//                LATAbits.LATA2 = 1;
//                adcResetState = 0;//no reset from adc
//            }
//            if(ADRESH < 125)
//            {
//               LATAbits.LATA2 = 0; 
//               adcResetState = 1;//reset by adc
//            }
//            ADCON0bits.ADGO = 1;
//        }
//        __delay_ms(1);
//        actstate = PORTAbits.RA5;
//        
//        if(laststate != actstate)
//        {
//            if((laststate == 0)&&(actstate == 1))
//            {
//                //do something
//                pintimer = 0;
//            }
//            laststate = actstate;
//        }
//        if(pintimer > 5000)
//        {
//            lastPinState = LATAbits.LATA2;
//            LATAbits.LATA2 = 0; 
//            __delay_ms(3000);
//            LATAbits.LATA2 = lastPinState; 
//            pintimer = 0;
//        }