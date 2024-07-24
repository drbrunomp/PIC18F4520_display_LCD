// PIC18F4520 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include"LCD4b.h"
#define XTAL_FREQ 8000000
#define B1 PORTBbits.RB0

unsigned int VdigADC_AN0;

float Voltage_AN0;

unsigned char VdigBuffer[16],
              VoltageBuffer[16];

int D1,
    D2,
    D3,
    D4;

char Buffer_D1[8],
     Buffer_D2[8],
     Buffer_D3[8],
     Buffer_D4[8];

void config_FOSC()
{
    OSCCON = 0X00;
    OSCTUNE = 0X00;
}

void config_IO()
{
    TRISB = 0X01;
    TRISC = 0X00;
    PORTC = 0XFF;
}

void config_ADC()
{
    ADCON0 = 0X01;
    ADCON1 = 0X0E;
    ADCON2 = 0X87;
}

void conv_AN0()
{
    __delay_us(30);
    ADCON0bits.GO = 1;
    while(ADCON0bits.GO);
    VdigADC_AN0 = ADRESH;
    VdigADC_AN0 = (VdigADC_AN0 << 8) + ADRESL;
    Voltage_AN0 = VdigADC_AN0 * 4.89e-3;
}

void lcd_SENSOR()
{
    sprintf(VdigBuffer,
            "%01d DEC      ",
            VdigADC_AN0);
    
    sprintf(VoltageBuffer,
            "%0.2f V       ",
            Voltage_AN0);
    
    lcd_write(1,1,"ADC:   ");
    lcd_write(1,8,VdigBuffer);
    lcd_write(2,1,"Volt:   ");
    lcd_write(2,8,VoltageBuffer);
}

void logic_CONTROL()
{
    if(Voltage_AN0 == (0))
    {
        D1 = 0;
        D2 = 0;
        D3 = 0;
        D4 = 0;
        
    }
    else if(Voltage_AN0 > (0) && Voltage_AN0 <= (5*0.25))
    {
        D1 = 1;
        D2 = 0;
        D3 = 0;
        D4 = 0;
        
    }
    else if(Voltage_AN0 > (5*0.25) && Voltage_AN0 <= (5*0.50))
    {
        D1 = 1;
        D2 = 1;
        D3 = 0;
        D4 = 0;
    }
    else if(Voltage_AN0 > (5*0.50) && Voltage_AN0 <= (5*0.75))
    {
        D1 = 1;
        D2 = 1;
        D3 = 1;
        D4 = 0;
    }
    else if(Voltage_AN0 > (5*0.75))
    {
        D1 = 1;
        D2 = 1;
        D3 = 1;
        D4 = 1;
    }
    PORTCbits.RC0 =! D1;
    PORTCbits.RC1 =! D2;
    PORTCbits.RC2 =! D3;
    PORTCbits.RC3 =! D4;  
}
    
void lcd_LEDS()
{    
    sprintf(Buffer_D1,
            "%01d",
            D1);
    
    sprintf(Buffer_D2,
            "%01d",
            D2);
    
    sprintf(Buffer_D3,
            "%01d",
            D3);
    
    sprintf(Buffer_D4,
            "%01d",
            D4);
    
    lcd_write(1,1,"LED 1:");
    lcd_write(1,7,Buffer_D1);
    lcd_write(1,8," LED 2:");
    lcd_write(1,15,Buffer_D2);
    lcd_write(2,1,"LED 3:");
    lcd_write(2,7,Buffer_D3);
    lcd_write(2,8," LED 4:");
    lcd_write(2,15,Buffer_D4);
}

void main()
{
    config_FOSC();
    config_IO();
    config_ADC();
    lcd_init();
    
    while(1)
    {
        conv_AN0();        
        logic_CONTROL();
        
        if(B1 == 1)
        {
            
            lcd_SENSOR();
        }
        else
        {
           
           lcd_LEDS();
           
           
        }             
    }
}
