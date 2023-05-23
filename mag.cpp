#include <wiringPi.h>
#include <wiringPiI2C.h>

// //include <includes/wiringPi.h>
// //include <includes/wiringPiI2C.h>

#include <iostream>
#include <cmath>
#include <math.h>

//AK8963 registers
#define AK8963_ADDR   = 0x0C
#define AK8963_ST1    = 0x02
#define HXH          = 0x04
#define HYH          = 0x06
#define HZH          = 0x08
#define AK8963_ST2   = 0x09
#define AK8963_CNTL  = 0x0A
float mag_sens = 4900.0 // magnetometer sensitivity: 4800 uT

void AK8963_start(){
    wiringPiI2CWriteReg8(AK8963_ADDR,AK8963_CNTL,0x00);
    delay(500);
    AK8963_bit_res = 0x01;  //0b0001 = 16-bit
    AK8963_samp_rate = 0x06; //0b0010 = 8 Hz, 0b0110 = 100 Hz
    AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate; //bit conversion
    wiringPiI2CWriteReg8(AK8963_ADDR,AK8963_CNTL,AK8963_mode);
    delay(500);
}

short AK8963_reader(register){
    //read magnetometer values
    low = wiringPiI2CWriteReg8(AK8963_ADDR, register-1);
    high = wiringPiI2CWriteReg8(AK8963_ADDR, register);
    //combine higha and low for unsigned bit value
    value = ((high << 8) | low);
    return value;
}

float AK8963_conv(){
    //raw magnetometer bits

    loop_count = 0;
    while (1){
        mag_x = AK8963_reader(HXH);
        mag_y = AK8963_reader(HYH);
        mag_z = AK8963_reader(HZH);

        //the next line is needed for AK8963
        if bin(wiringPiI2CWriteReg8(AK8963_ADDR,AK8963_ST2))=='0b10000'{
            break;
        }
        loop_count+=1;
    }
        
    //convert to acceleration in g and gyro dps
    m_x = (mag_x/(2.0**15.0))*mag_sens;
    m_y = (mag_y/(2.0**15.0))*mag_sens;
    m_z = (mag_z/(2.0**15.0))*mag_sens;

    return m_x,m_y,m_z;
}

int main(){

    AK8963_start();
    

}