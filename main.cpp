//git branch 
//git checkout 
#include "includes/wiringPiI2C.h"
#include "includes/wiringPi.h"

#include <iostream>
#include <cmath>
#include <math.h>

#define _USE_MATH_DEFINES



#define Device_Address      0x68	/*Device Address/Identifier for MPU6050*/
#define PWR_MGMT_1          0x6B
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define INT_ENABLE          0x38
////////////////////////////////
#define AK8963_DEVICE_ADDR  0x0C 
#define AK8963_DEVICE_ID    0x00 
#define AK8963_INFORMATION  0x01
////////////////////////////////
#define AK8963_HXL          0x03
#define AK8963_HXH          0x04
#define AK8963_HYL          0x05
#define AK8963_HYH          0x06
#define AK8963_HZL          0x07
#define AK8963_HZH          0x08
////////////////////////////////
#define AK8963_STATUS_1     0x02
#define AK8963_STATUS_2     0x09
////////////////////////////////
#define AK8963_CONTROL_1    0x0A
#define AK8963_CONTROL_2    0x0B
////////////////////////////////
#define AK8963_SELFTEST     0x0C
#define AK8963_TEST_1       0x0D
#define AK8963_TEST_2       0x0E
////////////////////////////////
#define AK8963_I2CDIS       0x0F
////////////////////////////////
#define AK8963_ASAX         0x10
#define AK8963_ASAY         0x11
#define AK8963_ASAZ         0x12
///////////////////////////////
#define Magnetometer_Sensitivity_Scale_Factor ((float)0.15f)
//////////////////////////////


int MPU_addr,  AK_addr ;

int16_t AK8963_bit_res, AK8963_samp_rate, AK8963_mode; 
float interval;
long preInterval;
float  x,y,z ;



void init_MPU () {


    wiringPiI2CWriteReg8 (MPU_addr, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
    wiringPiI2CWriteReg8 (MPU_addr, CONFIG, 0x00);		/* Write to Configuration register */
    wiringPiI2CWriteReg8 (MPU_addr, PWR_MGMT_1, 0x01);	/* Write to power management register */
    wiringPiI2CWriteReg8 (MPU_addr, INT_ENABLE, 0x01);	/*Write to interrupt enable register ???*/


    wiringPiI2CWriteReg8( AK_addr  ,AK8963_CONTROL_1 ,0x00);
    delay(100) ;
    AK8963_bit_res = 0b0001; // 0b0001 = 16-bit
    AK8963_samp_rate = 0b0010; // 0b0010 = 8 Hz, 0b0110 = 100 Hz
    AK8963_mode = (AK8963_bit_res <<4) + AK8963_samp_rate ;// bit conversion
    wiringPiI2CWriteReg8( AK_addr  ,AK8963_CONTROL_1,AK8963_mode);
    delay(100);
    

}




short read_raw_data(int addr)

{
	short high_byte,low_byte,value;
	high_byte = wiringPiI2CReadReg8(MPU_addr, addr);
	low_byte = wiringPiI2CReadReg8(MPU_addr, addr-1);
	value = (high_byte << 8) | low_byte;
	return value;
}




void update()
{
    millis() ;
    interval = (millis() - preInterval) * 0.001;
    
    int16_t rawMagX, rawMagY, rawMagZ;
    rawMagX = read_raw_data(AK8963_HXH);
	rawMagY = read_raw_data(AK8963_HYH);
	rawMagZ = read_raw_data(AK8963_HZH);	
    x += ((float)rawMagX) * interval ;
    y += ((float)rawMagY)* interval ;
    z += ((float)rawMagZ)* interval ;
    
    
    preInterval = millis();

}


int main () {

  MPU_addr = wiringPiI2CSetup(Device_Address); 
AK_addr = wiringPiI2CSetup(AK8963_DEVICE_ADDR); 
    
    init_MPU () ;
   
    while(1) {
    
        update();
        std::cout<<z<<std::endl ;
    
    }
    return 0;
}




