#ifndef LCD16X2_4BIT_H_
#define LCD16X2_4BIT_H_

//#define  F_CPU 2000000UL
#include <avr/io.h>
#include <util/delay.h>

#define LCD_DPRT PORTB
#define LCD_DDDR DDRB
#define LCD_RS 0
#define LCD_EN 1

void lcdcommand(unsigned char cmnd);
void lcddata(unsigned int data);
void lcdinit(void);
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_print(char *str);
void lcd_clear(void);

void lcdcommand(unsigned char cmnd)
{
	LCD_DPRT = (LCD_DPRT & 0x0f)|(cmnd & 0xf0);	// SEND COMMAND TO DATA PORT
	LCD_DPRT &= ~ (1<<LCD_RS);						// RS = 0 FOR COMMAND
	LCD_DPRT |= (1<<LCD_EN);						// EN = 1 FOR H TO L PULSE
	_delay_us(1);									// WAIT FOR MAKE ENABLE WIDE
	LCD_DPRT &= ~(1<<LCD_EN);						// EN = 0 FOR H TO L PULSE
	_delay_us(100);									// WAIT FOR MAKE ENABLE WIDE
	
	LCD_DPRT = (LCD_DPRT & 0x0f)|(cmnd << 4);		// SEND COMMAND TO DATA PORT
	LCD_DPRT |= (1<<LCD_EN);						// EN = 1 FOR H TO L PULSE
	_delay_us(1);									// WAIT FOR MAKE ENABLE WIDE
	LCD_DPRT &= ~(1<<LCD_EN);						// EN = 0 FOR H TO L PULSE
	_delay_ms(2);									// WAIT FOR MAKE ENABLE WIDE
}

void lcddata(unsigned int data)
{
	LCD_DPRT = (LCD_DPRT & 0x0f)|(data & 0xf0);		// SEND DATA TO DATA PORT 
	LCD_DPRT |= (1<<LCD_RS);						// MAKE RS = 1 FOR DATA 
	LCD_DPRT |= (1<<LCD_EN);						// EN=0 FOR H TO L PULSE 
	_delay_us(1);									// WAIT FOR MAKE ENABLE WIDE 
	LCD_DPRT &= ~(1<<LCD_EN);						// EN = 0 FOR H TO L PULSE 
	_delay_us(100);									// WAIT FOR MAKE ENABLE WIDE 
	
	LCD_DPRT = (LCD_DPRT & 0x0f)|(data << 4);		
	LCD_DPRT |= (1<<LCD_EN);						// EN=0 FOR H TO L PULSE
	_delay_us(1);									// WAIT FOR MAKE ENABLE WIDE
	LCD_DPRT &= ~(1<<LCD_EN);						// EN = 0 FOR H TO L PULSE
	_delay_ms(2);									// WAIT FOR MAKE ENABLE WIDE
}

void lcdinit(void)
{
	LCD_DDDR = 0xFF;
	_delay_ms(200);									// WAIT FOR SOME TIME
	lcdcommand(0x33);
	lcdcommand(0x32);								// SEND $32 FOR INIT OT 0X02 
	lcdcommand(0x28);								// INIT. LCD 2 LINE, 5 X 7 MATRIX
	lcdcommand(0x0C);								// DISPLAY ON CURSOR ON
	lcdcommand(0x01);								// LCD CLEAR
	_delay_ms(2);
	lcdcommand(0x82);								// SHIFT CURSOR TO WRITE
}

void lcd_gotoxy(unsigned char x, unsigned char y)
{
	unsigned char firstcharadd[]={0x80, 0xC0};
	lcdcommand(firstcharadd[y] + x);
}

void lcd_print(char *str)
{
	unsigned char i=0;
	while (str[i] |= 0)
	{
		lcddata(str[i]);
		i++;
	}
}

void lcd_print_xy(char row, char pos, char* str)
{
	if (row == 0 && pos<16)
	lcdcommand((pos & 0x0F)|0x80);		/* Command of first row and required position<16 */
	else if (row == 1 && pos<16)
	lcdcommand((pos & 0x0F)|0xC0);		/* Command of first row and required position<16 */
	lcd_print(str);					/* Call LCD string function */
}

void lcd_clear(void)
{
	lcdcommand(0x01);
	_delay_ms(2);
}

#endif /* LCD16X2_4BIT_H_ */