// #include <wiringPi.h>
// #include <wiringPiI2C.h>

#include "includes/wiringPi.h"
#include "includes/wiringPiI2C.h"

#include <iostream>
#include <cmath>
#include <math.h>

//AK8963 registers
#define AK8963_ADDR  0x0C
#define AK8963_ST1   0x02
#define HXH          0x04
#define HYH          0x06
#define HZH          0x08
#define AK8963_ST2   0x09
#define AK8963_CNTL  0x0A

float mag_sens = 4900.0; // magnetometer sensitivity: 4800 uT
int fd;
int16_t magX, magY, magZ, mag_angle;
float pi = 3.14159265359 ; 
float resX, resY, resZ;


void AK8963_start(){
    wiringPiI2CWriteReg8(AK8963_ADDR,AK8963_CNTL,0x00);
    delay(500);
    int AK8963_bit_res = 0x01;  //0b0001 = 16-bit
    int AK8963_samp_rate = 0x06; //0b0010 = 8 Hz, 0b0110 = 100 Hz
    int AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate; //bit conversion
    wiringPiI2CWriteReg8(AK8963_ADDR,AK8963_CNTL,AK8963_mode);
    delay(500);
}

short AK8963_reader(int reg){
     short high_byte, low_byte, value;
    //read magnetometer values
    low_byte = wiringPiI2CReadReg8(AK8963_ADDR, reg - 1 );
    high_byte = wiringPiI2CReadReg8(AK8963_ADDR, reg  );
    //combine higha and low for unsigned bit value
    value = ((high_byte << 8) | low_byte);
    return value;
}

int main(){

    AK8963_start();

    while (1){
        magX = AK8963_reader(HXH);
        magY = AK8963_reader(HYH);
        magZ = AK8963_reader(HZH);

        std::cout << "X: " <<magX << std::endl;
        std::cout << "Y: " <<magY << std::endl;
        std::cout << "Z: " <<magZ << std::endl;
    }

}