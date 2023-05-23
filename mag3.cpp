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
#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1c
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47


float mag_sens = 4900.0; // magnetometer sensitivity: 4800 uT
int fd;
int16_t magX, magY, magZ, mag_angle;
float pi = 3.14159265359 ; 
float resX, resY, resZ;


void AK8963_start(){
    wiringPiI2CWriteReg8(AK8963_ADDR,AK8963_CNTL,0x00);
    delay(500);
    int AK8963_bit_res = 0b0001;  //0b0001 = 16-bit
    int AK8963_samp_rate = 0b0010; //0b0010 = 8 Hz, 0b0110 = 100 Hz
    int AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate; //bit conversion
    wiringPiI2CWriteReg8(AK8963_ADDR,AK8963_CNTL,AK8963_mode);
    delay(500);

    wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
    wiringPiI2CWriteReg8 (fd, CONFIG, 0x00);		/* Write to Configuration register */
    wiringPiI2CWriteReg8 (fd, GYRO_CONFIG, 24);	/* Write to Gyro Configuration register */
    // ^^^ was 24
    // wiringPiI2CWriteReg8 (fd, ACCEL_CONFIG,0x00); 
    wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01);	/* Write to power management register */
    wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01);
}

short read_raw_data(int addr){
	short high_byte,low_byte,value;
	high_byte = wiringPiI2CReadReg8(fd, addr);
	low_byte = wiringPiI2CReadReg8(fd, addr+1);
	value = (high_byte << 8) | low_byte;
	return value;
}

int main(){

    AK8963_start();

    while (1){
        magX = read_raw_data(HXH);
        magY = read_raw_data(HYH);
        magZ = read_raw_data(HZH);

        std::cout << "X: " <<magX << std::endl;
        std::cout << "Y: " <<magY << std::endl;
        std::cout << "Z: " <<magZ << std::endl;
        delay(1000);
    }

}