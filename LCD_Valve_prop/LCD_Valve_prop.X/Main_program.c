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
#include"LCD4b_EXSTO.h"
#define S1 PORTCbits.RC0
#define S2 PORTCbits.RC1
#define S3 PORTCbits.RC2
#define S4 PORTCbits.RC3

unsigned int VdigADC_AN0;

unsigned char BufferLCD_Voltage[16],
              BufferLCD_Press[16];

float PressSensor,
      VoltageSensor,
      PressPercent;


void config_FOSC()
{
    OSCCON = 0X00;
    OSCTUNE = 0X00;
}

void config_IO()
{
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
    VoltageSensor = 0.0048875855 * VdigADC_AN0;      // Conversão de valor Digital para Tensão Elétrica
    PressSensor = VoltageSensor / 0.0769230769;      // Equação do sensor (Tensão x Pressão) 0BAR --> 65BAR  
    PressPercent = (PressSensor * 100) / 65;         // Equação percentual do sensor
}

void lcd_SENSOR()
{
   sprintf(BufferLCD_Press,
           "%0.1fBAR      ",
           PressSensor);
   
   sprintf(BufferLCD_Voltage,
           "%0.2fV     ",
           VoltageSensor);
    
    lcd_write(1,1,"SENSOR:        ");
    lcd_write(1,9,BufferLCD_Voltage);
    lcd_write(2,1,"PRESSAO:       ");  
    lcd_write(2,10,BufferLCD_Press);
}

void logic_CONTROL()
{
    if(PressSensor <= (0.25*65))
    {
        S1 = 1;
        S2 = 0;
        S3 = 0;
        S4 = 0;
    }
    else if(PressSensor > (0.25*65) && PressSensor <= (0.50*65))
    {
        S1 = 1;
        S2 = 1;
        S3 = 0;
        S4 = 0;
    }
    else if(PressSensor > (0.50*65) && PressSensor <= (0.75*65))
    {
        S1 = 1;
        S2 = 1;
        S3 = 1;
        S4 = 0;
    }
    else if(PressSensor > (0.75*65))
    {
        S1 = 1;
        S2 = 1;
        S3 = 1;
        S4 = 1;
    }
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
        lcd_SENSOR();        
    }    
}