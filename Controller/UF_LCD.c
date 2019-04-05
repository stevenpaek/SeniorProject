//----------------UF_LCD FOR MSP430----------------------
//
//
// 			PIN1:  Vss = GND
//		    PIN2:  Vdd = +5V
//		    PIN3:   Vo = Pot driven by +5V
//		    PIN4:   RS = 4.4
//		    PIN5:  R/W = GND
//		    PIN6:    E = 4.5
//          PIN7:  DB4 = 4.0
//          PIN8:  DB5 = 4.1
//          PIN9:  DB6 = 4.2
//          PIN10: DB7 = 4.3
//
//
//    *note: this code is written for 16 MHz,adjust delays accordingly.
//
//
//
//----------------CODE BY BRANDON CERGE--------------------
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/devices/msp432p4xx/inc/msp432p401r_classic.h>
#include <UF_LCD.h>

char uf_lcd_temp;
char uf_lcd_temp2;
char uf_lcd_x;

void delay() {
    volatile int count = 0;
    while (count < 1001) {
        count++;
    }
}

void lcd_command(char uf_lcd_x){
    P4DIR = 0xFF;
	uf_lcd_temp = uf_lcd_x;
	P4OUT = 0x00;
	delay();
	uf_lcd_x = uf_lcd_x >> 4;
	uf_lcd_x = uf_lcd_x & 0x0F;
	uf_lcd_x = uf_lcd_x | 0x20;
	P4OUT =  uf_lcd_x;
	delay();
	uf_lcd_x = uf_lcd_x & 0x0F;
	P4OUT =  uf_lcd_x;
	delay();
	P4OUT = 0x00;
	delay();
	uf_lcd_x = uf_lcd_temp;
	uf_lcd_x = uf_lcd_x & 0x0F;
	uf_lcd_x = uf_lcd_x | 0x20;
	P4OUT = uf_lcd_x;
	delay();
	uf_lcd_x = uf_lcd_x & 0x0F;
	P4OUT = uf_lcd_x;
	delay();
}

void lcd_init(void){
	lcd_command(0x33);
	lcd_command(0x32);
	lcd_command(0x2C);
	lcd_command(0x0C);
	lcd_command(0x01);
}

void lcd_char(char uf_lcd_x){
	P4DIR = 0xFF;
	uf_lcd_temp = uf_lcd_x;
	P4OUT = 0x10;
	delay();
	uf_lcd_x = uf_lcd_x >> 4;
	uf_lcd_x = uf_lcd_x & 0x0F;
	uf_lcd_x = uf_lcd_x | 0x30;
	P4OUT =  uf_lcd_x;
	delay();
	uf_lcd_x = uf_lcd_x & 0x1F;
	P4OUT =  uf_lcd_x;
	delay();
	P4OUT = 0x10;
	delay();
	uf_lcd_x = uf_lcd_temp;
	uf_lcd_x = uf_lcd_x & 0x0F;
	uf_lcd_x = uf_lcd_x | 0x30;
	P4OUT = uf_lcd_x;
	delay();
	uf_lcd_x = uf_lcd_x & 0x1F;
	P4OUT = uf_lcd_x;
	delay();
}

void lcdString(char *string)
{
    while (*string != 0) {
        lcd_char(*string);
        *string++;
    }
}
