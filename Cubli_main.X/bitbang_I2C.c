#include "bitbang_I2C.h"
#include <stdbool.h>
#include <xc.h>

bool I2C_WriteRegisters(unsigned char address, unsigned char *data, unsigned int num) {
    bool ret = true;
    
    address <<= 1;

    I2C_Start();
    I2C_Send(address & 0xFE); 
    if(!I2C_GetAck()) {
        ret = false;
    }
    while(num--) {
        I2C_Send(*data);
        if(!I2C_GetAck()) {
            ret = false;
        }
        data++;
    }
    I2C_Stop();
    return ret;
}

bool I2C_ReadRegisters(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num) {
    bool ret = true;
    
    address <<= 1;

    I2C_Start();
    I2C_Send(address & 0xFE); 
    if(!I2C_GetAck()) {
        ret = false;
    }
    I2C_Send(start_adr);
    if(!I2C_GetAck()) {
        ret = false;
    }
    I2C_Start();
    I2C_Send(address | 0x01); 
    if(!I2C_GetAck()) {
        ret = false;
    }
    
    if(num > 1) {
        while(num--) {
            *data = I2C_Read();
            data++;
            if(num > 0) I2C_SendAck();
        }
        
    } else {
        *data = I2C_Read();
    }
    
    I2C_SendNak();
    I2C_Stop();
    
    return ret;
}

bool I2C_CheckAddress(unsigned char addr) {
    bool ret = true;
    
    addr <<= 1;
    
    I2C_Start();
    I2C_Send(addr & 0xFE); 
    if(!I2C_GetAck()) {
        ret = false;
    }
    I2C_Stop();
    return ret;
}

void I2C_Send(unsigned char byte) {
    unsigned char count;
    SDA_LAT = 0;
    SCL_LAT = 0;
    I2C_DelayFull();    
    for(count = 8; count > 0; count--) {
        if((byte & 0x80) == 0) {      
            SDA_LAT = 0;                
            SDA_TRIS = 0;            
        } else {
            SDA_TRIS = 1;          
        }
        byte = byte << 1;            
        I2C_Clock();              
    }
    SDA_TRIS = 1;  
}

unsigned char I2C_Read() {
    unsigned char count, byte = 0;
    SDA_LAT = 0;
    SCL_LAT = 0;
    I2C_DelayFull();
    for(count = 8; count > 0; count--) {
        byte = byte << 1;
        SDA_TRIS = 1;     
        if(I2C_ReadBit()) {
            byte += 1;
        }
    }
    return byte;
}

void I2C_Start() {
    SDA_TRIS = 1;
    SCL_TRIS = 1;
    SCL_LAT = 0;
    SDA_LAT = 0;
    I2C_DelayHalf();  
    SDA_TRIS = 0;
    SDA_LAT = 0;
    I2C_DelayHalf();  
    SCL_TRIS = 0;
    I2C_DelayFull();    
}

void I2C_Stop() {
    SDA_TRIS = 0;
    SCL_TRIS = 1;
    I2C_DelayHalf();  
    SDA_TRIS = 1;
}

void I2C_Clock() {
    I2C_DelaySettle();    
    SCL_TRIS = 1;            
    I2C_DelayFull();     
    SCL_TRIS = 0;            
    I2C_DelayFull();         
}

unsigned char I2C_ReadBit() {
    unsigned char data = 0;
    I2C_DelaySettle(); 
    SCL_TRIS = 1;
    I2C_DelayHalf();
    if(SDA_PORT != 0) data = 1;  
    I2C_DelayHalf();  
    SCL_TRIS = 0;
    I2C_DelayFull();
    return data;
}

bool I2C_GetAck() {
    bool ack = true;
    
    SDA_LAT = 0;
    SCL_LAT = 0;
    SCL_TRIS = 0;                 
    SDA_TRIS = 1;  
    SCL_TRIS = 1; 
    I2C_DelayFull();
    
    if(SDA_PORT)
        ack = false;
    
    I2C_DelayHalf(); 
    SCL_TRIS = 0;
    I2C_DelayFull();
    I2C_DelayFull();
    
    return ack;
}

void I2C_SendAck() {
    SDA_LAT = 0;
    SDA_TRIS = 0; 
    I2C_DelaySettle();   
    I2C_Clock();       
    SDA_TRIS = 1;        
    I2C_DelaySettle();  
}

void I2C_SendNak() {
    SDA_TRIS = 1; 
    SDA_LAT = 0;
    I2C_DelaySettle();   
    I2C_Clock();         
    I2C_DelaySettle();  
}

void inline I2C_DelayFull() {
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
}

void inline I2C_DelayHalf() {
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
}

void inline I2C_DelaySettle() {
    Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();
}