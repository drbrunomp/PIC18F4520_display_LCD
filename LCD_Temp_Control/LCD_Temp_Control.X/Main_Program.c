
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
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
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
#define _XTAL_FREQ 8000000
#define B1 PORTBbits.RB0

unsigned int VdigADC_AN0;

unsigned char BufferLCD_Temp[16],
              BufferLCD_Percent[16];

char BufferFAN1[8],
     BufferFAN2[8],
     BufferRES[8];

float TempSensor,
      TempPercent,
      VoltageSensor;

int FAN1,
    FAN2,
    RES;

void config_FOSC()
{
    OSCCON = 0X00;
    OSCTUNE = 0X00;
}

void config_IO()
{
    TRISB = 0X01;
    TRISC = 0X00;
    PORTC = 0X00;
}

void config_ADC()
{
    ADCON0 = 0X01;       // Seleção dos canais analógicos; Estado da conversão e Habilitação do Conversor A/D
    ADCON1 = 0X0E;       // Tensão de referência; Seleção de entrada analógica
    ADCON2 = 0X80;       // Alinhamento dos Bits (ADRES); Tempo de aquisição; Fonte de Clock para o converesor A/D
}

void conv_AN0()
{
    __delay_ms(50);
    ADCON0bits.GO = 1;                              // Inicia o ciclo de conversão
    while(ADCON0bits.GO);                           // Aguarda o término do ciclo de conversão
    VdigADC_AN0 = ADRESH;                           // Atribui os 2 bits + significativos do ADRES
    VdigADC_AN0 = (VdigADC_AN0 << 8) + ADRESL;      // Mantém os 2 bits + significativos e soma os 8 bits - significativos do ADRES
}

void equation_SENSOR()
{
    VoltageSensor = 0.0049 * VdigADC_AN0;            // Conversão de valor Digital para Tensão Elétrica
    TempSensor = VoltageSensor / 0.0119047619;             // Equação do sensor (Tensão x Temperatura 'Celsius') 0C --> 420C  
    TempPercent = (TempSensor * 100) / 420;           // Equação percentual do sensor
}

void lcd_SENSOR()
{
   sprintf(BufferLCD_Temp,
           "%0.1f%cC      ",
           TempSensor, 0xDF);
   
   sprintf(BufferLCD_Percent,
           "%0.1f%c      ",
           TempPercent, 0x25);
    
    lcd_write(1,1,"Temp:    ");
    lcd_write(1,10,BufferLCD_Temp);
    lcd_write(2,1,"Percent:");  
    lcd_write(2,10,BufferLCD_Percent);
}

void lcd_PERIPHERAL()
{    
    sprintf(BufferFAN1,
            "%01d",
            FAN1);
    
    sprintf(BufferFAN2,
            "%01d",
            FAN2);
    
    sprintf(BufferRES,
            "%01d ",
            RES);
    
    lcd_write(1,1,"FAN 1:");
    lcd_write(1,7,BufferFAN1);
    lcd_write(1,8," FAN 2:");
    lcd_write(1,15,BufferFAN2);
    lcd_write(1,16," ");
    lcd_write(2,1,"RES:");
    lcd_write(2,5,BufferRES);
    lcd_write(2,6,"          ");
}

void logic_CONTROL()
{
    if(TempSensor <= (0.25*420))
    {
        FAN1 = 0;
        FAN2 = 0;
        RES = 1;
    }
    else if(TempSensor > (0.25*420) && TempSensor <= (0.50*420))
    {
        FAN1 = 1;
        FAN2 = 0;
        RES = 1;
    }
    else if(TempSensor > (0.50*420) && TempSensor <= (0.75*420))
    {
        FAN1 = 1;
        FAN2 = 1;
        RES = 1;
    }
    else if(TempSensor > (0.75*420))
    {
        FAN1 = 1;
        FAN2 = 1;
        RES = 0;
    }
    PORTCbits.RC0 = FAN1;
    PORTCbits.RC1 = FAN2;
    PORTCbits.RC2 = RES;
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
        equation_SENSOR();
        logic_CONTROL();
        
        if(B1 == 0) lcd_SENSOR();
        else lcd_PERIPHERAL();
    }    
}