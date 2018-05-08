#include <stdio.h>
#include <stdlib.h>



// PIC18F4525 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
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
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#define _XTAL_FREQ 40000000

#define lcd_clear() lcd_command(1)
#define lcd_origin() lcd_command(2)
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 40000000 // îïðåäåëåíèå òàêòîâîé ÷àñòîòû 40 ÌÃö äëÿ ôóíêöèè __delay_ms
#endif
#define E_pulse_with 50 //äëèòåëüíîñòü òàêòîâûõ èìïóëüñîâ LCD â ìêñ
#define LCD_E PORTDbits.RD3
#define LCD_RS PORTDbits.RD2
#define LCD_Data4 LATD // èíèöèàëèçàöèÿ ïîðòîâ D4-D7 LCD



void lcd_clk(void) /*ãåíåðàöèÿ èìïóëüñà íà âõîä EN*/
{
    LCD_E = 1;
    __delay_us(E_pulse_with);
    LCD_E = 0;
    __delay_us(E_pulse_with);
}


void lcd_command(unsigned char outbyte) /*îòïðàâèòü êîìàíäó (4-áèòíûé ðå-æèì ðàáîòû) */
{
    LCD_RS=0; //ðåæèì ïåðåäà÷è êîìàíäû
    LCD_Data4=(LCD_Data4&0x0f)|(outbyte&0xf0); // îòïðàâêà ñòàðøèõ ÷åòûðåõ áèò
    lcd_clk();
    LCD_Data4=(LCD_Data4&0x0f)|((outbyte<<4)&0xf0); // îòïðàâêà ìëàäøèõ ÷å-òûðåõ áèò
    lcd_clk();
    __delay_ms(1);
}


void lcd_putc(char outbyte) /* îòïðàâèòü äàííûå (4-áèòíàÿ îïåðàöèÿ) */
{
    LCD_RS=1; //ðåæèì ïåðåäà÷è äàííûõ
    LCD_Data4=(LCD_Data4&0x0f)|(outbyte&0xf0);
    lcd_clk();
    LCD_Data4=(LCD_Data4&0x0f)|((outbyte<<4)&0xf0);
    lcd_clk();
}


void lcd_puts(unsigned char line,const char *p) // *âûâîä ñòðîêè íà ýêðàí*
{
    lcd_origin(); // ïåðåõîä ê íóëåâîìó àäðåñó LCD
    lcd_command(line); // óñòàíîâèòü àäðåñ LCD 00H
    while(*p) // ïðîâåðèòü, ðàâåí ëè óêàçàòåëü 0
    {
        lcd_putc(*p); // îòïðàâèòü äàííûå íà LCD
        p++; // óâåëè÷èòü àäðåñ íà 1
    }
}

void inttolcd(unsigned char posi, long value) //âûâîä íà ýêðàí çíà÷åíèé ïåðå-ìåííûõ
{
    char buff[16];
    itoa(buff,value,10);
    lcd_puts(posi,buff);
}


void lcd_init() // èíèöèàëèçàöèÿ LCD-äèñïëåÿ
{
    TRISD &= 0x03;// ïåðåâîä âûâîäîâ RD4-RD7â ðåæèì âûõîäîâ
    LCD_Data4 &= 0b00001111;//óñòàíîâêà íóëåé íà ëèíèè ïåðåäà÷è äàííûõ
    LCD_E=0;
    LCD_RS=0;
    __delay_ms(1);
    /*èíèöèàëèçàöèÿ äèñïëåÿ*/
    LCD_Data4=(LCD_Data4&0x0f)|0x30;
    lcd_clk();
    __delay_ms(1);
    LCD_Data4=(LCD_Data4&0x0f)|0x30;
    lcd_clk();
    __delay_ms(1);
    LCD_Data4=(LCD_Data4&0x0f)|0x30;
    lcd_clk();
    __delay_ms(1);
    /*---------------------------------*/
    LCD_Data4=(LCD_Data4&0x0f)|0x20; // ïåðåêëþ÷èòü íà 4-áèòíûé ðåæèì ïåðå-äà÷è
    lcd_clk();
    __delay_ms(1);
    lcd_command(0x28);//óñòàíîâèòü N=1, F=0 (äâå ñòðîêè, ðàçìåð ñèìâîëà 5*8 òî÷åê
    lcd_command(0x01); // î÷èñòèòü âñ¸
    lcd_command(0x06); // àâòîìàòè÷åñêè ïåðåäâèíóòü êóðñîð ïîñëå áàéòà
    lcd_command(0x0C); // äèñïëåé âêëþ÷¸í, êóðñîðà íåò, íå ìîðãàåò
    lcd_command(0x02); // íà÷àëüíàÿ ïîçèöèÿ
    lcd_command(0x01); // î÷èñòèòü âñ¸ ñíîâà
}

long cnt = 0;
int isRunning = 0;


 


void interrupt low_priority  LIisr (void) 
{ 
    if (INTCONbits.TMR0IF) 
    { 
        if(isRunning){
        TMR0H=103; 
        TMR0L=106;        
        inttolcd(0x89,++cnt);
        INTCONbits.TMR0IF=0;
        }
    } 
}

void interrupt HIisr (void) {
    if (INTCONbits.INT0IF)
    {
        __delay_ms(30);// çàäåðæêà äëÿ èñêëþ÷åíèÿ âëèÿíèÿ «äðåáåçãà» êîíòàêòîâ
        
        if (PORTBbits.RB0==0)
        {
            if(isRunning){
                isRunning = 0;
            }else{
               cnt = 0;
               lcd_init();
               lcd_puts(0x0,"Cnt ms = 0");
               isRunning = 1; 
            }
//            inttolcd(0x89,++cnt);
            
            
        }
        INTCONbits.INT0IF=0;
    }    
}
  



int main(int argc, char** argv)
{
    lcd_init();
    lcd_puts(0x0,"Cnt ms = 0");
    ///////////////////////////
    // TRISBbits.RB3=0;//íàñòðîéêà RB3 íà âûõîä
     TRISBbits.RB0=1;//íàñòðîéêà RB0 íà âõîä
     RCONbits.IPEN=1; // ðàçðåøåíèå äâóõóðîâíåâûõ ïðåðûâàíèé
     INTCON2bits.INTEDG0=0; //ïðåðûâàíèå ïî íèñïàäàþùåìó ôðîíòó ñèãíàëà íà âõîäå INT0
     INTCONbits.INT0IF=0; //îáíóëåíèå ôëàãà ïðåðûâàíèÿ îò âíåøíåãî èñòî÷íèêà
     INTCONbits.GIEH=1; //ðàçðåøåíèå âûñîêîóðîâíåâûõ ïðåðûâàíèé
     INTCONbits.INT0IE=1;//ðàçðåøåíèå ïðåðûâàíèÿ îò âíåøíåãî èñòî÷íèêà
     LATBbits.LATB3=1;
    
    
    
  //  TRISBbits.RB3=0;//íàñòðîéêà RB3 íà âûõîä 
    T0CONbits.T08BIT=0;//íàñòðîéêà òàéìåðà ¹0 íà 16-áèòíûé ðåæèì ðàáîòû 
    T0CONbits.PSA=0;//ðàçðåøåíèå èñïîëüçîâàòü ïðåääåëèòåëü 
    T0CONbits.T0PS=0b000;//ïðåääåëèòåëü ðàâåí 256 
    T0CONbits.T0CS=0;//âûáîð âíóòðåííåãî èñòî÷íèêà òàêòîâûõ èìïóëüñîâ 
    TMR0H=103;//çàïèñü ñòàðøåãî áàéòà íà÷àëüíîãî çíà÷åíèÿ 
    TMR0L=106;//çàïèñü ìëàäøåãî áàéòà íà÷àëüíîãî çíà÷åíèÿ 
    T0CONbits.TMR0ON=1; 
    RCONbits.IPEN=1; // ðàçðåøåíèå äâóõóðîâíåâûõ ïðåðûâàíèé 
    INTCON2bits.TMR0IP=0;//ïðèñâîåíèå ïðåðûâàíèþ íèçêîãî ïðèîðèòåòà 
    INTCONbits.TMR0IF=0;//îáíóëåíèå ôëàãà ïðåðûâàíèÿ ïî ïåðåïîëíåíèþ òàéìåðà 0 
    INTCONbits.GIEH=1; //ðàçðåøåíèå âûñîêîóðîâíåâûõ ïðåðûâàíèé 
    INTCONbits.GIEL=1; //ðàçðåøåíèå íèçêîóðîâíåâûõ ïðåðûâàíèé 
    INTCONbits.TMR0IE=1;//ðàçðåøåíèå ïðåðûâàíèÿ ïî ïåðåïîëíåíèþ òàéìåðà 0 
   
    while(1)
    {
//        inttolcd(0x8A,1234); // âûâîä íîìåðà äàò÷èêà
    }
}
