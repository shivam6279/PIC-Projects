//i2c library

#ifndef _I2C_H_
#define _I2C_H_

void i2c3_init();
void i2c3_write_registers(unsigned char address, unsigned char *data, unsigned int num);
unsigned char i2c3_read_registers(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num);
bool i2c3_start();
void i2c3_stop();
void i2c3_ack();
void i2c3_nak();
unsigned char i2c3_read();
void i2c3_restart();
bool i2c3_send(unsigned char dat);
void i2c3_wait();

void i2c5_init();
void i2c5_write_registers(unsigned char address, unsigned char *data, unsigned int num);
unsigned char i2c5_read_registers(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num);
bool i2c5_start();
void i2c5_stop();
void i2c5_ack();
void i2c5_nak();
unsigned char i2c5_read();
void i2c5_restart();
bool i2c5_send(unsigned char dat);
void i2c5_wait();

void i2c3_init(){   
    I2C3CONbits.SDAHT = 1;
    I2C3CONbits.SIDL = 0;
    I2C3CONbits.STRICT = 0;
    I2C3CONbits.DISSLW = 0;
    I2C3CONbits.SMEN = 0;
    I2C3BRG = 123;
    I2C3CONbits.ON = 1;
}
void i2c3_write_registers(unsigned char address, unsigned char *data, unsigned int num){
    i2c3_start();
    i2c3_send(address & 0xFE);
    while(num--){
        i2c3_send(*data);
        data++;
    }
    i2c3_stop();
}
unsigned char i2c3_read_registers(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num){
    i2c3_start();
    i2c3_send(address & 0xFE); 
    i2c3_send(start_adr);
    i2c3_start();
    i2c3_send(address | 0x01);
    while(num--){
        *data = i2c3_read();
        data++;

        if(num > 0){
            i2c3_ack();
        }
        else{
            i2c3_nak();
        }
    }
    i2c3_stop();
    return 1;
}
bool i2c3_start(){
    I2C3CONbits.ACKDT = 0;
    i2c3_wait();
    I2C3CONbits.SEN = 1;     // Start condition enabled
    if(I2C3STATbits.BCL) return false;
    else{
        i2c3_wait();
        return true;
    }
}
void i2c3_stop(){
    unsigned int t = 2000;
    I2C3CONbits.PEN = 1;     // Stop condition enabled
    while(I2C3CONbits.PEN){
        if(t-- == 1) break;
    }
    I2C3CONbits.PEN = 0;
    I2C3CONbits.RCEN = 0; 
    I2C3STATbits.IWCOL = 1; 
    I2C3STATbits.BCL = 1; 
}
void i2c3_restart(){
    unsigned int t = 2000;
    I2C3CONbits.RSEN = 1;
    while(I2C3CONbits.RSEN){
        if(t-- == 1) break;
    }
    I2C3CONbits.RSEN = 0;
}
void i2c3_ack(){
    I2C3CONbits.ACKDT = 0;      // Acknowledge data bit, 0 = ACK
    I2C3CONbits.ACKEN = 1;      // Ack data enabled
    while(I2C3CONbits.ACKEN);   // wait for ack data to send on bus
}
void i2c3_nak(){
    I2C3CONbits.ACKDT = 1;      // Acknowledge data bit, 1 = NAK
    I2C3CONbits.ACKEN = 1;      // Ack data enabled
    while(I2C3CONbits.ACKEN);   // wait for ack data to send on bus
}
void i2c3_wait(){
    unsigned int t = 2000;
    while(I2C3CONbits.SEN || I2C3CONbits.PEN || I2C3CONbits.RCEN || I2C3CONbits.RSEN || I2C3CONbits.ACKEN || I2C3STATbits.TRSTAT){
        if(t-- == 1) break;
    }
    if(t == 0){
        I2C3CON &= 0xFFE0;
        I2C3STATbits.TRSTAT = 0;
    }
    I2C3STATbits.BCL = 0;
}
bool i2c3_send(unsigned char dat){
    unsigned int t = 2000;
    while(I2C3STATbits.TBF){
        if(t-- == 0) break;
    }
    I2C3TRN = dat;    // Move data to I2CBUF
    if(I2C3STATbits.IWCOL) return false;
    else{
        while(I2C3STATbits.TRSTAT);
        if(I2C3STATbits.BCL) return false;
        else{
            i2c3_wait();// wait for any pending transfer
            return true;
        }
    }
}
unsigned char i2c3_read(){
    unsigned char temp;      // Reception works if transfer is initiated in read mode
    I2C3STATbits.I2COV = 0;
    I2C3CONbits.RCEN = 1;    // Enable data reception
    while(!I2C3STATbits.RBF);// wait for buffer full
    temp = I2C3RCV;          // Read serial buffer and store in temp register
    i2c3_wait();              // wait to check any pending transfer
    return temp;             // Return the read data from bus
} 

void i2c5_init(){
    I2C5CONbits.SDAHT = 1;
    I2C5CONbits.SIDL = 0;
    I2C5CONbits.STRICT = 0;
    I2C5CONbits.DISSLW = 0;
    I2C5CONbits.SMEN = 0;
    I2C5BRG = 123;
    I2C5CONbits.ON = 1;
}
void i2c5_write_registers(unsigned char address, unsigned char *data, unsigned int num){
    i2c5_start();
    i2c5_send(address & 0xFE);
    while(num--){
        i2c5_send(*data);
        data++;
    }
    i2c5_stop();
}
unsigned char i2c5_read_registers(unsigned char address, unsigned char start_adr, unsigned char *data, unsigned int num){
    i2c5_start();
    i2c5_send(address & 0xFE); 
    i2c5_send(start_adr);
    i2c5_start();
    i2c5_send(address | 0x01);
    while(num--){
        *data = i2c5_read();
        data++;

        if(num > 0){
            i2c5_ack();
        }
        else{
            i2c5_nak();
        }
    }
    i2c5_stop();
    return 1;
}
bool i2c5_start(){
    I2C5CONbits.ACKDT = 0;
    i2c5_wait();
    I2C5CONbits.SEN = 1;     // Start condition enabled
    if(I2C5STATbits.BCL) return false;
    else{
        i2c5_wait();
        return true;
    }
}
void i2c5_stop(){
    unsigned int t = 2000;
    I2C5CONbits.PEN = 1;     // Stop condition enabled
    while(I2C5CONbits.PEN){
        if(t-- == 1) break;
    }
    I2C5CONbits.PEN = 0;
    I2C5CONbits.RCEN = 0; 
    I2C5STATbits.IWCOL = 1; 
    I2C5STATbits.BCL = 1; 
}
void i2c5_restart(){
    unsigned int t = 2000;
    I2C5CONbits.RSEN = 1;
    while(I2C5CONbits.RSEN){
        if(t-- == 1) break;
    }
    I2C5CONbits.RSEN = 0;
}
void i2c5_ack(){
    I2C5CONbits.ACKDT = 0;      // Acknowledge data bit, 0 = ACK
    I2C5CONbits.ACKEN = 1;      // Ack data enabled
    while(I2C5CONbits.ACKEN);   // wait for ack data to send on bus
}
void i2c5_nak(){
    I2C5CONbits.ACKDT = 1;      // Acknowledge data bit, 1 = NAK
    I2C5CONbits.ACKEN = 1;      // Ack data enabled
    while(I2C5CONbits.ACKEN);   // wait for ack data to send on bus
}
void i2c5_wait(){
    unsigned int t = 2000;
    while(I2C5CONbits.SEN || I2C5CONbits.PEN || I2C5CONbits.RCEN || I2C5CONbits.RSEN || I2C5CONbits.ACKEN || I2C5STATbits.TRSTAT){
        if(t-- == 1) break;
    }
    if(t == 0){
        I2C5CON &= 0xFFE0;
        I2C5STATbits.TRSTAT = 0;
    }
    I2C5STATbits.BCL = 0;
}
bool i2c5_send(unsigned char dat){
    I2C5TRN = dat;    // Move data to I2CBUF
    if(I2C5STATbits.IWCOL) return false;
    else{
        while(I2C5STATbits.TRSTAT);
        if(I2C5STATbits.BCL) return false;
        else{
            i2c5_wait();// wait for any pending transfer
            return true;
        }
    }
}
unsigned char i2c5_read(){
    unsigned char temp;      // Reception works if transfer is initiated in read mode
    I2C5STATbits.I2COV = 0;
    I2C5CONbits.RCEN = 1;    // Enable data reception
    while(I2C5CONbits.RCEN);
    i2c5_wait();
    I2C5STATbits.I2COV = 0;
    temp = I2C5RCV & 0xFF; // Read serial buffer and store in temp register           // wait to check any pending transfer
    return temp;        // Return the read data from bus
}  

#endif