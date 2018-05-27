#include "bitbang_I2C.h"
#include <stdbool.h>
#include <xc.h>

void I2C3_WriteRegisters(unsigned char address, unsigned char *data, unsigned int num) {
    I2C3_Start();
    I2C3_Send(address & 0xFE); 
    if(!I2C3_GetAck()) {
        I2C3_Stop();
        return;
    }
    while(num--) {
        I2C3_Send(*data);
        if(!I2C3_GetAck()) {
            I2C3_Stop();
            return;
        }
        data++;
    }
    I2C3_Stop();
}
unsigned char I2C3_ReadRegisters(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num) {
    I2C3_Start();
    I2C3_Send(address & 0xFE); 
    if(!I2C3_GetAck()) {
        I2C3_Stop();
        return 0;
    }
    I2C3_Send(start_adr);
    if(!I2C3_GetAck()) {
        I2C3_Stop();
        return 0;
    }
    I2C3_Start();
    I2C3_Send(address | 0x01);   //-- Lowest bit = 1 => READ
    if(!I2C3_GetAck()) {
        I2C3_Stop();
        return 0;
    }
    while(num--) {
        *data = I2C3_read();
        data++;
        if(num > 0) {
            I2C3_SendAck();
        }
    }
    I2C3_Stop();
    return 1;
}
void I2C3_Send(unsigned char byte) {
    unsigned char count;
    SDA3 = 0;
    SCL3 = 0;
    I2C_DelayFull();    
    for(count = 8; count > 0; count--){
        if((byte & 0x80) == 0) {      
            SDA3 = 0;                
            SDA3_TRIS = 0;            
        } else {
            SDA3_TRIS = 1;          
        }
        byte = byte << 1;            
        I2C3_Clock();              
    }
    SDA3_TRIS = 1;  
}
unsigned char I2C3_read() {
    unsigned char count, byte=0;
    SDA3 = 0;
    SCL3 = 0;
    I2C_DelayFull();
    for(count = 8; count > 0; count--) {
        byte = byte << 1;
        SDA3_TRIS = 1;     
        if(I2C3_ReadBit()) {
            byte +=1;
        }
    }
    return byte;
}
void I2C3_Start() {
    SDA3_TRIS = 1;
    SCL3_TRIS = 1;
    SCL3 = 0;
    SDA3 = 0;
    I2C_DelayHalf();  
    SDA3_TRIS = 0;
    SDA3 = 0;
    I2C_DelayHalf();  
    SCL3_TRIS = 0;
    I2C_DelayFull();    
}
void I2C3_Stop() {
    SDA3_TRIS = 0;
    SCL3_TRIS = 1;
    I2C_DelayHalf();  
    SDA3_TRIS = 1;
}
void I2C3_Clock() {
    I2C_DelaySettle();    
    SCL3_TRIS = 1;            
    I2C_DelayFull();     
    SCL3_TRIS = 0;            
    I2C_DelayFull();         
}
unsigned char I2C3_ReadBit() {
    unsigned char data = 0;
    I2C_DelaySettle(); 
    SCL3_TRIS = 1;
    I2C_DelayHalf();
    if(SDA3 != 0) data = 1;  
    I2C_DelayHalf();  
    SCL3_TRIS = 0;
    I2C_DelayFull();
    return data;
}
bool I2C3_GetAck() {
    SDA3 = 0;
    SCL3 = 0;
    SCL3_TRIS = 0;                 
    SDA3_TRIS = 1;  
    SCL3_TRIS = 1; 
    I2C_DelayFull();
    if(SDA3){
        return 0;  
    }
    I2C_DelayHalf(); 
    SCL3_TRIS = 0;
    I2C_DelayFull();
    I2C_DelayFull();
    return 1;
}
void I2C3_SendAck() {
    SDA3 = 0;
    SDA3_TRIS = 0; 
    I2C_DelaySettle();   
    I2C3_Clock();       
    SDA3_TRIS = 1;        
    I2C_DelaySettle();  
}

void I2C5_WriteRegisters(unsigned char address, unsigned char *data, unsigned int num) {
    I2C5_Start();
    I2C5_Send(address & 0xFE); 
    if(!I2C5_GetAck()) {
        I2C5_Stop();
        return;
    }
    while(num--) {
        I2C5_Send(*data);
        if(!I2C5_GetAck()) {
            I2C5_Stop();
            return;
        }
        data++;
    }
    I2C5_Stop();
}

void I2C5_ReadRegisters(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num) {
    I2C5_Start();
    I2C5_Send(address & 0xFE); 
    I2C5_GetAck();
    I2C5_Send(start_adr);
    I2C5_GetAck();
    I2C5_Start();
    I2C5_Send(address | 0x01); 
    I2C5_GetAck();
    while(num--) {
        *data = I2C5_Read();
        data++;
        if(num > 0) I2C5_SendAck();
    }
    I2C5_SendNak();
    I2C5_Stop();
}

void I2C5_Send(unsigned char byte) {
    unsigned char count;
    SDA5 = 0;
    SCL5 = 0;
    I2C_DelayFull();    
    for(count = 8; count > 0; count--) {
        if((byte & 0x80) == 0) {      
            SDA5 = 0;                
            SDA5_TRIS = 0;            
        } else {
            SDA5_TRIS = 1;          
        }
        byte = byte << 1;            
        I2C5_Clock();              
    }
    SDA5_TRIS = 1;  
}

unsigned char I2C5_Read() {
    unsigned char count, byte = 0;
    SDA5 = 0;
    SCL5 = 0;
    I2C_DelayFull();
    for(count = 8; count > 0; count--) {
        byte = byte << 1;
        SDA5_TRIS = 1;     
        if(I2C5_ReadBit()) {
            byte += 1;
        }
    }
    return byte;
}

void I2C5_Start() {
    SDA5_TRIS = 1;
    SCL5_TRIS = 1;
    SCL5 = 0;
    SDA5 = 0;
    I2C_DelayHalf();  
    SDA5_TRIS = 0;
    SDA5 = 0;
    I2C_DelayHalf();  
    SCL5_TRIS = 0;
    I2C_DelayFull();    
}

void I2C5_Stop() {
    SDA5_TRIS = 0;
    SCL5_TRIS = 1;
    I2C_DelayHalf();  
    SDA5_TRIS = 1;
}

void I2C5_Clock() {
    I2C_DelaySettle();    
    SCL5_TRIS = 1;            
    I2C_DelayFull();     
    SCL5_TRIS = 0;            
    I2C_DelayFull();         
}

unsigned char I2C5_ReadBit() {
    unsigned char data = 0;
    I2C_DelaySettle(); 
    SCL5_TRIS = 1;
    I2C_DelayHalf();
    if(SDA5 != 0) data = 1;  
    I2C_DelayHalf();  
    SCL5_TRIS = 0;
    I2C_DelayFull();
    return data;
}

bool I2C5_GetAck() {
    SDA5 = 0;
    SCL5 = 0;
    SCL5_TRIS = 0;                 
    SDA5_TRIS = 1;  
    SCL5_TRIS = 1; 
    I2C_DelayFull();
    //if(SDA5){
     //   return 0;  
    //}
    I2C_DelayHalf(); 
    SCL5_TRIS = 0;
    I2C_DelayFull();
    I2C_DelayFull();
    return 1;
}

void I2C5_SendAck() {
    SDA5 = 0;
    SDA5_TRIS = 0; 
    I2C_DelaySettle();   
    I2C5_Clock();       
    SDA5_TRIS = 1;        
    I2C_DelaySettle();  
}

void I2C5_SendNak() {
    SDA5_TRIS = 1; 
    I2C_DelaySettle();   
    I2C5_Clock();         
    I2C_DelaySettle();  
}

inline I2C_DelayFull() {
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    
}

inline I2C_DelayHalf() {
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
}

inline I2C_DelaySettle() {
    Nop();Nop();Nop();Nop();Nop();
}