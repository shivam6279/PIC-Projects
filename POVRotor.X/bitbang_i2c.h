#ifndef _bitbang_i2c_H_
#define _bitbang_i2c_H_

#define SCL3 PORTDbits.RD3 // PORTFbits.RF5
#define SCL3_TRIS TRISDbits.TRISD3 // TRISFbits.TRISF5
#define SDA3 PORTDbits.RD2 // PORTFbits.RF4  
#define SDA3_TRIS TRISDbits.TRISD2 // TRISFbits.TRISF4

#define SCL5 PORTFbits.RF5
#define SCL5_TRIS TRISFbits.TRISF5
#define SDA5 PORTFbits.RF4  
#define SDA5_TRIS TRISFbits.TRISF4

void i2c3_write_registers(unsigned char address, unsigned char *data, unsigned int num);
unsigned char i2c3_read_registers(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num);
void i2c3_send(unsigned char byte);
unsigned char i2c3_read();
bool i2c3_getack();
void i2c3_sendack();
void i2c3_start();
void i2c3_stop();
void i2c3_clock();
unsigned char i2c3_readbit();

void i2c5_write_registers(unsigned char address, unsigned char *data, unsigned int num);
void i2c5_read_registers(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num);
void i2c5_send(unsigned char byte);
unsigned char i2c5_read();
bool i2c5_getack();
void i2c5_sendack();
void i2c5_sendnak();
void i2c5_start();
void i2c5_stop();
void i2c5_clock();
unsigned char i2c5_readbit();

inline void i2c_delayhalf();
inline void i2c_delayfull();
inline void i2c_delaysettle();

void i2c3_write_registers(unsigned char address, unsigned char *data, unsigned int num){
    i2c3_start();
    i2c3_send(address & 0xFE); 
    if(!i2c3_getack()){
        i2c3_stop();
        return;
    }
    while(num--){
        i2c3_send(*data);
        if(!i2c3_getack()){
            i2c3_stop();
            return;
        }
        data++;
    }
    i2c3_stop();
}
unsigned char i2c3_read_registers(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num){
    i2c3_start();
    i2c3_send(address & 0xFE); 
    if(!i2c3_getack()){
        i2c3_stop();
        return 0;
    }
    i2c3_send(start_adr);
    if(!i2c3_getack()){
        i2c3_stop();
        return 0;
    }
    i2c3_start();
    i2c3_send(address | 0x01);   //-- Lowest bit = 1 => READ
    if(!i2c3_getack()){
        i2c3_stop();
        return 0;
    }
    while(num--){
        *data = i2c3_read();
        data++;

        if(num > 0){
            i2c3_sendack();
        }
    }
    i2c3_stop();
    return 1;
}
void i2c3_send(unsigned char byte){
    unsigned char count;
    SDA3 = 0;
    SCL3 = 0;
    i2c_delayfull();    
    for(count = 8; count > 0; count--){
        if((byte & 0x80) == 0){      
            SDA3 = 0;                
            SDA3_TRIS = 0;            
        }
        else{
            SDA3_TRIS = 1;          
        }
        byte = byte << 1;            
        i2c3_clock();              
    }
    SDA3_TRIS = 1;  
}
unsigned char i2c3_read(){
    unsigned char count, byte=0;
    SDA3 = 0;
    SCL3 = 0;
    i2c_delayfull();
    for(count = 8; count > 0; count--){
        byte = byte << 1;
        SDA3_TRIS = 1;     
        if(i2c3_readbit()){
            byte +=1;
        }
    }
    return byte;
}
void i2c3_start(){
    SDA3_TRIS = 1;
    SCL3_TRIS = 1;
    SCL3 = 0;
    SDA3 = 0;
    i2c_delayhalf();  
    SDA3_TRIS = 0;
    SDA3 = 0;
    i2c_delayhalf();  
    SCL3_TRIS = 0;
    i2c_delayfull();    
}
void i2c3_stop(){
    SDA3_TRIS = 0;
    SCL3_TRIS = 1;
    i2c_delayhalf();  
    SDA3_TRIS = 1;
}
void i2c3_clock(){
    i2c_delaysettle();    
    SCL3_TRIS = 1;            
    i2c_delayfull();     
    SCL3_TRIS = 0;            
    i2c_delayfull();         
}
unsigned char i2c3_readbit(){
    unsigned char data = 0;
    i2c_delaysettle(); 
    SCL3_TRIS = 1;
    i2c_delayhalf();
    if(SDA3 != 0) data = 1;  
    i2c_delayhalf();  
    SCL3_TRIS = 0;
    i2c_delayfull();
    return data;
}
bool i2c3_getack(){
    SDA3 = 0;
    SCL3 = 0;
    SCL3_TRIS = 0;                 
    SDA3_TRIS = 1;  
    SCL3_TRIS = 1; 
    i2c_delayfull();
    if(SDA3){
        return 0;  
    }
    i2c_delayhalf(); 
    SCL3_TRIS = 0;
    i2c_delayfull();
    i2c_delayfull();
    return 1;
}
void i2c3_sendack(){
    SDA3 = 0;
    SDA3_TRIS = 0; 
    i2c_delaysettle();   
    i2c3_clock();       
    SDA3_TRIS = 1;        
    i2c_delaysettle();  
}

void i2c5_write_registers(unsigned char address, unsigned char *data, unsigned int num){
    i2c5_start();
    i2c5_send(address & 0xFE); 
    if(!i2c5_getack()){
        i2c5_stop();
        return;
    }
    while(num--){
        i2c5_send(*data);
        if(!i2c5_getack()){
            i2c5_stop();
            return;
        }
        data++;
    }
    i2c5_stop();
}
void i2c5_read_registers(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num){
    i2c5_start();
    i2c5_send(address & 0xFE); 
    i2c5_getack();
    i2c5_send(start_adr);
    i2c5_getack();
    i2c5_start();
    i2c5_send(address | 0x01); 
    i2c5_getack();
    while(num--){
        *data = i2c5_read();
        data++;
        if(num > 0) i2c5_sendack();
    }
    i2c5_sendnak();
    i2c5_stop();
}
void i2c5_send(unsigned char byte){
    unsigned char count;
    SDA5 = 0;
    SCL5 = 0;
    i2c_delayfull();    
    for(count = 8; count > 0; count--){
        if((byte & 0x80) == 0){      
            SDA5 = 0;                
            SDA5_TRIS = 0;            
        }
        else{
            SDA5_TRIS = 1;          
        }
        byte = byte << 1;            
        i2c5_clock();              
    }
    SDA5_TRIS = 1;  
}
unsigned char i2c5_read(){
    unsigned char count, byte = 0;
    SDA5 = 0;
    SCL5 = 0;
    i2c_delayfull();
    for(count = 8; count > 0; count--){
        byte = byte << 1;
        SDA5_TRIS = 1;     
        if(i2c5_readbit()){
            byte += 1;
        }
    }
    return byte;
}
void i2c5_start(){
    SDA5_TRIS = 1;
    SCL5_TRIS = 1;
    SCL5 = 0;
    SDA5 = 0;
    i2c_delayhalf();  
    SDA5_TRIS = 0;
    SDA5 = 0;
    i2c_delayhalf();  
    SCL5_TRIS = 0;
    i2c_delayfull();    
}
void i2c5_stop(){
    SDA5_TRIS = 0;
    SCL5_TRIS = 1;
    i2c_delayhalf();  
    SDA5_TRIS = 1;
}
void i2c5_clock(){
    i2c_delaysettle();    
    SCL5_TRIS = 1;            
    i2c_delayfull();     
    SCL5_TRIS = 0;            
    i2c_delayfull();         
}
unsigned char i2c5_readbit(){
    unsigned char data = 0;
    i2c_delaysettle(); 
    SCL5_TRIS = 1;
    i2c_delayhalf();
    if(SDA5 != 0) data = 1;  
    i2c_delayhalf();  
    SCL5_TRIS = 0;
    i2c_delayfull();
    return data;
}
bool i2c5_getack(){
    SDA5 = 0;
    SCL5 = 0;
    SCL5_TRIS = 0;                 
    SDA5_TRIS = 1;  
    SCL5_TRIS = 1; 
    i2c_delayfull();
    //if(SDA5){
     //   return 0;  
    //}
    i2c_delayhalf(); 
    SCL5_TRIS = 0;
    i2c_delayfull();
    i2c_delayfull();
    return 1;
}
void i2c5_sendack(){
    SDA5 = 0;
    SDA5_TRIS = 0; 
    i2c_delaysettle();   
    i2c5_clock();       
    SDA5_TRIS = 1;        
    i2c_delaysettle();  
}
void i2c5_sendnak(){
    SDA5_TRIS = 1; 
    i2c_delaysettle();   
    i2c5_clock();         
    i2c_delaysettle();  
}

inline void i2c_delayfull(){
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
    
}
inline void i2c_delayhalf(){
    Nop();Nop();Nop();Nop();Nop();Nop();
    Nop();Nop();Nop();Nop();Nop();Nop();
}
inline void i2c_delaysettle(){
    Nop();Nop();Nop();Nop();Nop();
}

#endif