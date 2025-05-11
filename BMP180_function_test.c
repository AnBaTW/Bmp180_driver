#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>

#define BMP180_address  0x77
#define Measure_control 0xF4
#define Temperature     0x2E
#define Pressure        0x34
#define MSB_reg         0xF6
#define LSB_reg         0xF7
#define XLSB_reg        0xF8
#define press_data      1
#define Temp_data       0

int fd;
short AC1;
short AC2;
short AC3;
unsigned long AC4;
unsigned long AC5;
unsigned long AC6;
short B1;
short B2;
short MB;
short MC;
short MD;
long X1, X2, X3;
long B5, B6, B3;
unsigned long B4, B7;
double T,P;

void BMP180_init(){
    uint8_t MSB;
    uint8_t LSB;
    MSB = wiringPiI2CReadReg8(fd,0xAA);
    LSB =wiringPiI2CReadReg8(fd,0xAB);
    AC1 = (MSB<<8) + LSB;
    //--------------------------------
    MSB = wiringPiI2CReadReg8(fd,0xAC);
    LSB =wiringPiI2CReadReg8(fd,0xAD);
    AC2 = (MSB<<8) + LSB;
    //--------------------------------
    MSB = wiringPiI2CReadReg8(fd,0xAE);
    LSB =wiringPiI2CReadReg8(fd,0xAF);
    AC3 = (MSB<<8) + LSB;
    //--------------------------------
    MSB = wiringPiI2CReadReg8(fd,0xB0);
    LSB =wiringPiI2CReadReg8(fd,0xB1);
    AC4 = (MSB<<8) + LSB;
    //--------------------------------
    MSB = wiringPiI2CReadReg8(fd,0xB2);
    LSB =wiringPiI2CReadReg8(fd,0xB3);
    AC5 = (MSB<<8) + LSB;
    //-------------------------------
    MSB = wiringPiI2CReadReg8(fd,0xB4);
    LSB =wiringPiI2CReadReg8(fd,0xB5);
    AC6 = (MSB<<8) + LSB;
    //-------------------------------
    MSB = wiringPiI2CReadReg8(fd,0xB6);
    LSB =wiringPiI2CReadReg8(fd,0xB7);
    B1 = (MSB<<8) + LSB;
    //--------------------------------
    MSB = wiringPiI2CReadReg8(fd,0xB8);
    LSB =wiringPiI2CReadReg8(fd,0xB9);
    B2 = (MSB<<8) + LSB;
    //--------------------------------
    MSB = wiringPiI2CReadReg8(fd,0xBA);
    LSB =wiringPiI2CReadReg8(fd,0xBB);
    MB = (MSB<<8) + LSB;
    //--------------------------------
    MSB = wiringPiI2CReadReg8(fd,0xBC);
    LSB =wiringPiI2CReadReg8(fd,0xBD);
    MC = (MSB<<8) + LSB;
    //--------------------------------
    MSB = wiringPiI2CReadReg8(fd,0xBE);
    LSB =wiringPiI2CReadReg8(fd,0xBF);
    MD = (MSB<<8) + LSB;
    //-------------------------------
}

 void BMP180_read(char sampling_mode){
    uint8_t MSB;
    uint8_t LSB;
    uint8_t XLSB;
    uint16_t UT;
    uint32_t UP;
    //Write data into MC reg (Read temperature)
    wiringPiI2CWriteReg8(fd,Measure_control,Temperature);
    usleep(4500); //wait 4.5 ms
    //Read data
    MSB = wiringPiI2CReadReg8(fd,MSB_reg);
    LSB = wiringPiI2CReadReg8(fd,LSB_reg);
    UT = (MSB<<8) + LSB;
    //----------------------------------------
    int press_hex = Pressure + (sampling_mode<<6);
    //Write data into MC reg (Read temperature)
    wiringPiI2CWriteReg8(fd,Measure_control,press_hex);
    //Waitting time depend on oss
    switch (sampling_mode) { 
        case 0:
            usleep(4500);
            break;
        case 1:
            usleep(7500);
            break;
        case 2:
            usleep(13500);
            break;
        case 3:
            usleep(25500);
            break;
    }
    //Read data
    MSB = wiringPiI2CReadReg8(fd,MSB_reg);
    LSB = wiringPiI2CReadReg8(fd,LSB_reg);
    XLSB = wiringPiI2CReadReg8(fd,XLSB_reg);
    UP = ((MSB<<16) + (LSB<<8) +XLSB)>>(8-sampling_mode);
    //Calulate True value (Temp)
    X1 = (UT - AC6) * AC5 >> 15; //Chia cho 2^15
    X2 = (MC << 11) / (X1 + MD);
    B5 = X1 + X2;
    T = ((B5 + 8)/pow(2,4))/10; //chia 2^4 (đơn vị 1 0C)
    //Calculate true value (pressure)
    B6 = B5 - 4000;
    X1 = (B2 * ((B6 * B6)>> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((long)AC1 * 4 + X3) << sampling_mode) + 2) >> 2;

    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (AC4 * (unsigned long)(X3 + 32768)) >> 15;
    B7 = ((unsigned long)UP - B3) * (50000 >> sampling_mode);
    if (B7 < 0x80000000)
        P = (B7 << 1) / B4;
    else
        P = (B7 / B4) << 1;

    X1 = (P/pow(2,8)) * (P/pow(2,8));
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * P)/pow(2,16);
    P = P + ((X1 + X2 + 3791)/pow(2,4)); // đơn vị: Pa
    //OUtPUT
}

//Code demo
int main(){
    fd = wiringPiI2CSetup(BMP180_address);
    BMP180_init();
    double A;
    while(1){
    BMP180_read(2);
    //calculate the alitude
    A = 44330*(1-pow((P/(101325)),1/5.255));
    printf("\rPress %lf Pa, Temp: %lf C, alitute: %lf ",P,T,A);
    fflush(stdout);
    usleep(100000);
    }
    return 0;
}