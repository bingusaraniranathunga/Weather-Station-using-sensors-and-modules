#ifndef TWI_MASTER_H_
#define TWI_MASTER_H_
#define  F_CPU 2000000UL
#include <avr/io.h>
#include <stdio.h>

#define TWI_START_SENT      0x08
#define TWI_R_START_SENT    0x10
#define TWI_SLAVE_W_ACK     0x18
#define TWI_SLAVE_W_NACK    0x20
#define TWI_DATA_SENT_ACK   0x28
#define TWI_DATA_SENT_NACK  0x30
#define TWI_ARBIT_LOST      0x38

#define TWI_SLAVE_R_ACK     0x40
#define TWI_SLAVE_R_NACK    0x48
#define TWI_DATA_RECV_ACK   0x50
#define TWI_DATA_RECV_NACK  0x58

void TWI_Init(void);
void TWI_Start(void);
void TWI_Stop(void);
void TWI_Send_Addr(uint8_t addr);
void TWI_Send_Data(uint8_t data);
uint8_t TWI_Read_Data(short ack);
void TWI_Restart(void);


uint8_t i2cdebug = 0;

void TWI_Init(void){
    
    TWSR = 0x00; // Prescaler 1
    TWBR = 0x00; // Decimal 32
}

void TWI_Start(void){
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // Enable TWI, generate start condition and clear interrupt flag
    while(!(TWCR & (1<<TWINT)));                // Wait until TWI finish its start
    while((TWSR & 0xF8)!= TWI_START_SENT);      // Check for the acknoledgement
}

void TWI_Stop(void){
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);    // Enable TWI, generate stop condition and clear interrupt flag
    while(TWCR & (1<<TWSTO));                   // Wait until stop condition execution
}

void TWI_Send_Addr(uint8_t addr){
	
    TWDR = addr;                                 //  write SLA+R in TWI data register
    TWCR = (1<<TWINT) | (1<<TWEN);              // Enable TWI and clear interrupt flag 
    while(!(TWCR & (1<<TWINT)));                // Wait until TWI finish its current job (Write operation)
    
}

void TWI_Send_Data(uint8_t data){
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while(!(TWCR & (1<<TWINT)));
       
}

uint8_t TWI_Read_Data(short ack){
    if (!ack)
        TWCR = (1<<TWINT) | (1<<TWEN);
    else
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    
    while(!(TWCR & (1<<TWINT)));
    return TWDR;  
}

void TWI_Restart(void){
    // Clear TWI interrupt flag, Put start condition on SDA, Enable TWI
    TWCR= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while(!(TWCR & (1<<TWINT))); // wait till restart condition is transmitted
    while((TWSR & 0xF8)!= TWI_R_START_SENT); // Check for the acknoledgement
}
#endif