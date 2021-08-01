#define F_CPU 2000000UL
#define USART_BAUDRATE 9600 
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#include <util/delay.h>
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "LCD16x2_4bit.h"
#include "TWI_Master.h"

#define bmp180_read_addr            0xEF
#define bmp180_write_addr           0xEE
#define control_reg_addr            0xF4

#define temperature_sel             0x2E
#define pressure_0_sel              0x34
#define pressure_1_sel              0x74
#define pressure_2_sel              0xB4
#define pressure_3_sel              0xF4

#define bmp180_mode_0               0
#define bmp180_mode_1               1
#define bmp180_mode_2               2
#define bmp180_mode_3               3

#define wait_temperature            4.5f
#define wait_pressure_mode_0        4.5f
#define wait_pressure_mode_1        7.5f
#define wait_pressure_mode_2        13.5f
#define wait_pressure_mode_3        25.5f

#define out_msb                     0xF6
#define out_lsb                     0xF7
#define out_xlsb                    0xF8

#define DHT11_PIN 6

#define Device_Write_address	0xD0				// Define RTC DS1307 slave address for write operation
#define Device_Read_address		0xD1				// Make LSB bit high of slave address for read operation

uint8_t c=0,I_RH,D_RH,I_Temp,D_Temp,CheckSum;

int second,minute,hour,day,date,month,year;

const uint16_t bmp180_coeff_addr[11] = {
    0xAAAB, 0xACAD, 0xAEAF, 0xB0B1, 0xB2B3,
    0xB4B5, 0xB6B7, 0xB8B9, 0xBABB, 0xBCBD,
    0xBEBF
};

const int32_t pressure_sea_level = 101325;

typedef struct {
    short AC1;
    short AC2;
    short AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    short B1;
    short B2;
    long B5;
    short MB;
    short MC;
    short MD;
} calibration_coeff;

typedef struct {
    short mode;
    long UT;
    long UP;
    long temperature;
    long pressure;
    float altitude;
    calibration_coeff calib_coeffs;
} bmp180;

bmp180 bmp_180;

//////////////////////////////////////////Function Prototypes //////////////////////////////////////
void dht11_data(void);
void ds1307_data(void);
void rain_drop_data(void);
void bmp180_pressure(void);
void bmp180_altitude(void);

void init_sensor(short);
long calculate_temperature(void);
long calculate_pressure(void);
long read_temperature(void);
long read_pressure(void);
void read_coeffs(void);
long calculate_altitude(void);
void detection(void);

void uart_init(void);
void send_uart(unsigned char val);



int main(void)
{	ds1307_data();
	_delay_ms(3000);
	lcd_clear();
	
	dht11_data();
	_delay_ms(3000);
	lcd_clear();
	
	rain_drop_data();
	_delay_ms(3000);
	lcd_clear();
	
	bmp180_pressure();
	_delay_ms(3000);
	lcd_clear();
	
	bmp180_altitude();
	_delay_ms(3000);
	lcd_clear();
	
	
	while(1){
		detection();
	}
	
	return 0;
}

////////////////////////////////dht11///////////////////////////////////

void Request(void)						                 //Microcontroller send start pulse or request
{
	DDRD |= (1<<DHT11_PIN);
	PORTD &= ~(1<<DHT11_PIN);		                       //set to low pin 
	_delay_ms(20);					                       // wait for 20ms 
	PORTD |= (1<<DHT11_PIN);		                          // set to high pin 
}

void Response(void)						                    //receive response from DHT11 
{
	DDRD &= ~(1<<DHT11_PIN);
	while(PIND & (1<<DHT11_PIN));                               //wait untill receive the response
	while((PIND & (1<<DHT11_PIN))==0);                          //response pulse(low 54us)
	while(PIND & (1<<DHT11_PIN));                               //response pulse(high 80us)
}

uint8_t Receive_data(void)							                  //receive data 
{	
	for (int q=0; q<8; q++)
	{
		while((PIND & (1<<DHT11_PIN)) == 0);	                    //check received bit 0 or 1
		_delay_us(30);                                                  //(0 --> low 54 us, high 24 us),(1-->low 54 us, high 70 us)
		if(PIND & (1<<DHT11_PIN))				                      //if high pulse is greater than 30ms 
		c = (c<<1)|(0x01);						                    // then its logic HIGH 
		else									                      // otherwise its logic LOW 
		c = (c<<1);
		while(PIND & (1<<DHT11_PIN));
	}
	return c;
}

void dht11_data(void){
	lcdinit();					
	lcd_clear();
	
	uart_init();
	
	char data[5];
	
	Request();				                                      //send start pulse 
	Response();				                                            // receive response
	I_RH=Receive_data();	                                         // store first eight bit in I_RH
	D_RH=Receive_data();	                                         // store next eight bit in D_RH 
	I_Temp=Receive_data();	                                                // store next eight bit in I_Temp 
	D_Temp=Receive_data();                                                       	// store next eight bit in D_Temp
	CheckSum=Receive_data();                                          // store next eight bit in CheckSum 
		
		
	if ((I_RH + D_RH + I_Temp + D_Temp) != CheckSum)
	{
		lcd_gotoxy(0,0);
		lcd_print("Error");
	}
		
	else
	{	
		lcd_gotoxy(0,0);			
		lcd_print("Humidity =");
		itoa(I_RH,data,10);
		lcd_gotoxy(11,0);
		send_string(data);
		send_string(".");
		lcd_print(data);
		lcd_print(".");
			
		itoa(D_RH,data,10);
		send_string(data);
		send_string("%");
		send_string("\t");
		lcd_print(data);
		lcd_print("%");

		lcd_gotoxy(0,1);
		lcd_print("Temp = ");
		itoa(I_Temp,data,10);
		lcd_gotoxy(6,1);
		send_string(data);
		send_string(".");
		lcd_print(data);
		lcd_print(".");
			
		itoa(D_Temp,data,10);
		lcd_print(data);
		send_string(data);
		send_string("C");
		send_string("\t");
		lcddata(0xDF);
		lcd_print("C ");
			
		itoa(CheckSum,data,10);	
}
		
}

///////////////////////////ds1307////////////////////////////

void RTC_Read_Clock(char read_clock_address)
{
	TWI_Start();                                                       //start I2C communication
	TWI_Send_Addr(Device_Write_address);                                  //send address to RTC
	TWI_Send_Data(read_clock_address);
	TWI_Restart();
	TWI_Send_Addr(Device_Read_address);
	second = TWI_Read_Data(1);						                                 // Read second
	minute = TWI_Read_Data(1);						                       // Read minute
	hour = TWI_Read_Data(0);                                               // Read hour
	TWI_Stop();
}

void RTC_Read_Calendar(char read_calendar_address)
{
	TWI_Start();                                                           //start I2C communication
	TWI_Send_Addr(Device_Write_address);
	TWI_Send_Data(read_calendar_address);
	TWI_Restart();
	TWI_Send_Addr(Device_Read_address);

	day =TWI_Read_Data(1);							                          //Read day 
	date =TWI_Read_Data(1);							                         // Read date
	month = TWI_Read_Data(1);						                           // Read month 
	year =TWI_Read_Data(0);							                           // Read the year with Nack
	TWI_Stop();									                                  // Stop I2C communication
}

void ds1307_data(void){
	char buffer[20];
	
	TWI_Init();										// Initialize I2C
	uart_init();
	lcdinit();										// Initialize LCD16x2
	lcd_clear();
	
	RTC_Read_Clock(0);							// Read the clock with second address i.e location is 0 
	
	lcd_print("Time=");
	sprintf(buffer, "%02x:%02x:%02x", hour, minute, second);
	lcd_print_xy(6,0,buffer);
	send_string(buffer);
	send_string("\t");
	
	RTC_Read_Calendar(3);                 // Read the calender with day address i.e location is 3
		
	sprintf(buffer, "%02x/%02x/%02x", date, month, year);
	send_string(buffer);
	send_string("\t");
	lcd_gotoxy(0,1);						
	lcd_print("Date=");
	lcd_print_xy(1,6,buffer);	
	
}

////////////////////////////////rain drop sensor/////////////////
void ADC_Init(void)
{
	DDRA &=~(1<<0);		// Make ADC port as input
	ADCSRA=1<<ADEN ;	   // Enable ADC	
}

int ADC_Read(void)
{
	unsigned int val1;
	ADMUX=1<<REFS0;    //AVcc with external capacitor at Aref pin
	ADCSRA|=1<<ADSC;                 //start conversion
	while(ADCSRA & (1<<ADSC)){}     //wait for the complete the conversion 
	val1=ADC;
	
	return val1;					
}

void rain_drop_data(void)
{	lcdinit();		// initialize the 16x2 LCD
	lcd_clear();	//clear the LCD 
	ADC_Init();		// initialize the ADC
	uart_init();
	char array[10];
	unsigned int adc_value,c;
	
	adc_value = ADC_Read();	
	c=((adc_value*100)/1023);
	lcd_print("Percentage=");	
	itoa(c,array,10);
	lcd_print_xy(12,0,array);
	send_string(array);
	lcd_gotoxy(14,0);
	lcd_print("%");
	send_string("%");
	send_string("\t");
	
	if(adc_value<300){
		lcd_print_xy(1,0,"Heavy rain");
		send_string("Heavy rain");
		send_string("\t");
	}else if((300<=adc_value)&(adc_value<500)){
		lcd_print_xy(1,0,"Moderate rain");
		send_string("Moderate rain");
		send_string("\t");
	}else{
		lcd_print_xy(1,0,"No rain");
		send_string("No rain");
		send_string("\t");
	}

}

////////////////serial communication////////////////////////
void uart_init(void) {
	
	UCSRB |= (1 << RXEN) | (1 << TXEN);  // Transmission & Reception Enable (TXEN=1, RXEN=1)

	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); //use 8 bit char size
	UBRRH = (BAUD_PRESCALE >> 8);           //load upper 8 bit baud prescale 
	UBRRL = BAUD_PRESCALE;
}

void send_uart(unsigned char val){
	while ((UCSRA & (1 << UDRE)) == 0) ; 	//wait until transmition is finished
	UDR = val;								//Transmit the charater
}

void send_string(unsigned char * s){
	//	send string
	while(* s){
		send_uart(* s);
		* s++;
	}
}
//////////////////////////////bmp180//////////////////////////////////////
void init_sensor(short mode) {
    read_coeffs();
    bmp_180.mode = mode;
}



long calculate_pressure() {
    long UP, X1, X2, X3, B3, B6, _pressure,Y1,Y2,B5,UT,T;
    unsigned long B4, B7;
    UT = read_temperature();
	Y1 = ((UT-bmp_180.calib_coeffs.AC6)*bmp_180.calib_coeffs.AC5)/pow(2,15);
	Y2 = (bmp_180.calib_coeffs.MC* pow(2, 11))/(Y1+bmp_180.calib_coeffs.MD);
	B5 = Y1+Y2;
	bmp_180.calib_coeffs.B5=B5;
	T = (bmp_180.calib_coeffs.B5+8)/pow(2,4);
    UP = read_pressure();
    B6 =bmp_180.calib_coeffs.B5 - 4000;
    X1 = (bmp_180.calib_coeffs.B2 * (B6 * B6 /pow(2,12))) /pow(2,11);
    X2 = bmp_180.calib_coeffs.AC2 * B6 /pow(2,11);
    X3 = X1 + X2;
    B3 = ((((long)bmp_180.calib_coeffs.AC1 * 4 + X3) << bmp_180.mode) + 2) / 4;
    X1 = bmp_180.calib_coeffs.AC3 * B6 /pow(2,13);
    X2 = (bmp_180.calib_coeffs.B1 * (B6 * B6 /pow(2,12))) /pow(2,16);
    X3 = ((X1 + X2) + 2) /pow(2,2);
    B4 = bmp_180.calib_coeffs.AC4 * (unsigned long) (X3 + 32768) /pow(2,15);
    B7 = ((unsigned long) UP - B3) * (50000UL >> bmp_180.mode);
    if (B7 < 0x80000000UL)
        _pressure = (B7 * 2) / B4;
    else
        _pressure = (B7 / B4) * 2;
    X1 = (_pressure /pow(2,8)) * (_pressure /pow(2,8));
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * _pressure) /pow(2,16);
    bmp_180.pressure = _pressure + ((X1 + X2 + 3791) /pow(2,4));
	return bmp_180.pressure;
}

long calculate_altitude() {
    bmp_180.altitude = (float) 44330 * (1 - pow(((float) bmp_180.pressure / pressure_sea_level), 0.1903));
	return bmp_180.altitude;
}


long read_pressure() {
    long UP;
    uint8_t msb, lsb, xlsb;
    
    TWI_Start();
    TWI_Send_Addr(bmp180_write_addr);
    TWI_Send_Data(control_reg_addr);
    
    switch (bmp_180.mode) {
        default:
        case bmp180_mode_0:
            TWI_Send_Data(pressure_0_sel);
            TWI_Stop();
            _delay_ms(wait_pressure_mode_0);
            break;
        case bmp180_mode_1:
            TWI_Send_Data(pressure_1_sel);
            TWI_Stop();
            _delay_ms(wait_pressure_mode_1);
            break;
        case bmp180_mode_2:
            TWI_Send_Data(pressure_2_sel);
            TWI_Stop();
            _delay_ms(wait_pressure_mode_2);
            break;
        case bmp180_mode_3:
            TWI_Send_Data(pressure_3_sel);
            TWI_Stop();
            _delay_ms(wait_pressure_mode_3);
            break;
    }
    
 
    TWI_Start();
    TWI_Send_Addr(bmp180_write_addr);
    TWI_Send_Data(out_msb);
    TWI_Send_Data(out_lsb);
    TWI_Restart();
    TWI_Send_Addr(bmp180_read_addr);
    msb = TWI_Read_Data(1);
    lsb = TWI_Read_Data(0);
    TWI_Stop();
    
    TWI_Start();
    TWI_Send_Addr(bmp180_write_addr);
    TWI_Send_Data(out_xlsb);
    TWI_Restart();
    TWI_Send_Addr(bmp180_read_addr);
    xlsb = TWI_Read_Data(0);
    TWI_Stop();
    
    UP = ((unsigned long) msb << 16 | (unsigned long) lsb << 8 | (unsigned long) xlsb) >> (8 - bmp_180.mode);
    return (long) UP;
}

long read_temperature() {
    long UT;
    uint8_t msb,lsb;
    
    TWI_Start();
    TWI_Send_Addr(bmp180_write_addr);
    TWI_Send_Data(control_reg_addr);
	TWI_Send_Data(temperature_sel);
	TWI_Stop();
	_delay_ms(wait_temperature);
	
    TWI_Start();
    TWI_Send_Addr(bmp180_write_addr);
    TWI_Send_Data(out_msb);
    TWI_Send_Data(out_lsb);
    TWI_Restart();
    TWI_Send_Addr(bmp180_read_addr);
    msb = TWI_Read_Data(1);
    lsb = TWI_Read_Data(0);
    TWI_Stop();
    
    UT = (unsigned long) msb << 8 | (unsigned long) lsb;
    return (long) UT;
}

void read_coeffs() {
    
    uint16_t coeffs[11];
    
    for (short i = 0; i < 11; i++) {
        TWI_Start();
        TWI_Send_Addr(bmp180_write_addr);
        TWI_Send_Data((bmp180_coeff_addr[i] & 0xFF00) >> 8);
        TWI_Send_Data(bmp180_coeff_addr[i] & 0x00FF);
        TWI_Restart();
        TWI_Send_Addr(bmp180_read_addr);
        coeffs[i] = TWI_Read_Data(1) << 8;
        coeffs[i] |= TWI_Read_Data(0);
        TWI_Stop();
    }
    
    bmp_180.calib_coeffs.AC1 = (short) coeffs[0];
    bmp_180.calib_coeffs.AC2 = (short) coeffs[1];
    bmp_180.calib_coeffs.AC3 = (short) coeffs[2];
    bmp_180.calib_coeffs.AC4 = (uint16_t) coeffs[3];
    bmp_180.calib_coeffs.AC5 = (uint16_t) coeffs[4];
    bmp_180.calib_coeffs.AC6 = (uint16_t) coeffs[5];
    bmp_180.calib_coeffs.B1 = (short) coeffs[6];
    bmp_180.calib_coeffs.B2 = (short) coeffs[7];
    bmp_180.calib_coeffs.MB = (short) coeffs[8];
    bmp_180.calib_coeffs.MC = (short) coeffs[9];
    bmp_180.calib_coeffs.MD = (short) coeffs[10];
    
}


void bmp180_pressure(void){
	long a;
	char d[10];
	init_sensor(bmp180_mode_0);
	a= calculate_pressure();
	uart_init();
	lcdinit();										
	lcd_clear();
	lcd_print_xy(0,0,"Pressure=");
	ltoa(a,d,10);
	lcd_print_xy(1,0,d);
	lcd_print_xy(1,7,"Pa");
	send_string(d);
	send_string("Pa");
	send_string("\t");
}

void bmp180_altitude(void){
	long b;
	char d[10];
	b= calculate_altitude();
	uart_init();
	lcdinit();										
	lcd_clear();
	lcd_print_xy(0,0,"Altitude=");
	ltoa(b,d,10);
	lcd_print_xy(1,0,d);
	lcd_print_xy(1,4,"m");
	send_string(d);
	send_string("m");
	send_string("\n");
}


void detection(void){
	long a;
	unsigned int b;
	init_sensor(bmp180_mode_0);
	a= calculate_pressure();
	ADC_Init();	
	b = ADC_Read();
	DDRC|=(1<<PC4)|(1<<PC5)|(1<<PC6);
	if(b<300 || a<97000 || a>105000){
		PORTC  |= (1<<PC5)|(1<<PC6);
		PORTC &= ~(1<<PC4);
		_delay_ms(3000);
		PORTC &= ~(1<<PC5);
		PORTC &= ~(1<<PC6);
		
		
	}else{
		PORTC  |= (1<<PC4);
		PORTC &= ~(1<<PC5);
		PORTC &= ~(1<<PC6);
		_delay_ms(3000);
		PORTC &= ~(1<<PC4);
	}

}
