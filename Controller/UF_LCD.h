#ifndef __UF_LCD
#define __UF_LCD

void delay(void);
void lcd_command(char);
void lcd_init(void);
void lcd_char(char);
void lcdString(char *string);

#endif
