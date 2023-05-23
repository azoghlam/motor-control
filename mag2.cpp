// #include <wiringPi.h>
// #include <wiringPiI2C.h>

#include "includes/wiringPi.h"
#include "includes/wiringPiI2C.h"

#include <iostream>
#include <cmath>
#include <math.h>

#define _USE_MATH_DEFINES

#define HMC5883L_ADDRESS              0x0C

#define HMC5883L_REG_STATUS_A         0x02
#define HMC5883L_REG_STATUS_B         0x09
#define HMC5883L_REG_CNTL             0x0A
#define HMC5883L_REG_OUT_X_M          0x04
//define HMC5883L_REG_OUT_X_L          0x04
#define HMC5883L_REG_OUT_Y_M          0x06
//define HMC5883L_REG_OUT_Y_L          0x08
#define HMC5883L_REG_OUT_Z_M          0x08
//define HMC5883L_REG_OUT_Z_L          0x06
#define HMC5883L_REG_STATUS           0x09
#define HMC5883L_REG_IDENT_A          0x0A
#define HMC5883L_REG_IDENT_B          0x0B
#define HMC5883L_REG_IDENT_C          0x0C

#define HMC5883L_CRA_AVERAGING_BIT       6
#define HMC5883L_CRA_AVERAGING_LENGTH    2
#define HMC5883L_CRA_RATE_BIT            4
#define HMC5883L_CRA_RATE_LENGTH         3
#define HMC5883L_CRA_BIAS_BIT            1
#define HMC5883L_CRA_BIAS_LENGTH         2

#define HMC5883L_AVERAGING_1          0x00
#define HMC5883L_AVERAGING_2          0x01
#define HMC5883L_AVERAGING_4          0x10
#define HMC5883L_AVERAGING_8          0x11

#define HMC5883L_RATING_0C75          0x00
#define HMC5883L_RATING_1C5           0x01
#define HMC5883L_RATING_3             0x02 
#define HMC5883L_RATING_7C5           0x03
#define HMC5883L_RATING_15            0x04
#define HMC5883L_RATING_30            0x05
#define HMC5883L_RATING_75            0x06

#define HMC5883L_MEASURMENT_DEF       0x00
#define HMC5883L_MEASURMENT_POS       0x01
#define HMC5883L_MEASURMENT_NEG       0x02

#define HMC5883L_GAIN_BIT             7
#define HMC5883L_GAIN_LENGTH          3
#define HMC5883L_GAIN_1370            0x00
#define HMC5883L_GAIN_1090            0x01
#define HMC5883L_GAIN_820             0x02
#define HMC5883L_GAIN_660             0x03
#define HMC5883L_GAIN_440             0x04
#define HMC5883L_GAIN_390             0x05
#define HMC5883L_GAIN_330             0x06
#define HMC5883L_GAIN_230             0x07

#define HMC5883L_MODEREG_BIT        1
#define HMC5883L_MODEREG_LENGTH     2

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0

#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_STATUS  0x1B
#define ACCEL_STATUS 0x1c
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47



int fd;
float mag_sens = 4900.0;
int16_t magX, magY, magZ, mag_angle;
float pi = 3.14159265359 ; 
// float elapsedtime, time, timeprev ;

void AK8963_start(){

    wiringPiI2CWriteReg8(HMC5883L_ADDRESS,HMC5883L_REG_CNTL,0x00);
    delay(500);
    int AK8963_bit_res = 0b0001; // 0b0001 = 16-bit
    int AK8963_samp_rate = 0b0110; // 0b0010 = 8 Hz, 0b0110 = 100 Hz
    int AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate; // bit conversion
    wiringPiI2CWriteReg8(HMC5883L_ADDRESS,HMC5883L_REG_CNTL,AK8963_mode);
    delay(500);
}

short AK8963_reader(int addr) {

    short high_byte, low_byte, value ;
    high_byte = wiringPiI2CReadReg8(fd, addr) ;
    low_byte = wiringPiI2CReadReg8(fd, addr-1) ;
    value = (high_byte << 8) | low_byte ;
    return value ;
}

int AK8963_conv(){
    int loop_count = 0;
    while (1){
        magX = AK8963_reader(HMC5883L_REG_OUT_X_M);
        magY = AK8963_reader(HMC5883L_REG_OUT_Y_M);
        magZ = AK8963_reader(HMC5883L_REG_OUT_Z_M);

        // the next line is needed for AK8963
        if (binary(wiringPiI2CWriteReg8(HMC5883L_ADDRESS,HMC5883L_REG_STATUS_B))=='0b10000'){
            break;
        }
        loop_count+=1;
    }
        
    //convert to acceleration in g and gyro dps
    int16_t m_x = (magX/(2.0*15.0))*mag_sens;
    int16_t m_y = (magY/(2.0*15.0))*mag_sens;
    int16_t m_z = (magZ/(2.0*15.0))*mag_sens;

    return m_x,m_y,m_z;
}

void init() {
    int error;

    error = wiringPiI2CWriteReg8 (fd, HMC5883L_REG_STATUS_A , 0x70);	 //write to_STATUSuration Register A
    if(error == -1) std::cout<<"error_STATUS A";
    error = wiringPiI2CWriteReg8 (fd, HMC5883L_REG_STATUS_B , 0xa0);	//Write to_STATUSuration Register B for gain
    if(error == -1) std::cout<<"error_STATUS B";
    error = wiringPiI2CWriteReg8 (fd, HMC5883L_REG_CNTL , 0);	       //Write to mode Register for selecting mode
    if(error == -1) std::cout<<"error_STATUS select mode";

}

void init_MPU () {
    wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
    wiringPiI2CWriteReg8 (fd, CONFIG, 0x00);		/* Write to_STATUSuration register */
    wiringPiI2CWriteReg8 (fd, GYRO_STATUS, 24);	/* Write to Gyro_STATUSuration register */
    // ^^^ was 24
    // wiringPiI2CWriteReg8 (fd, ACCEL_STATUS,0x00); 
    wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01);	/* Write to power management register */
    wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01);	/*Write to interrupt enable register ???*/
    // angleGyroX = 0;
    // angleGyroY = 0;
    // angleX = angleAccX;
    // angleY = angleAccY;
    // preInterval = millis();

    // update();
}


int main()
{
        fd = wiringPiI2CSetup(HMC5883L_ADDRESS); 
//     init_MPU();
//     float time;
//     long timeprev;
//     float elapsedtime;
//     // initialization function
//     init();
//     delay(3000);
    AK8963_start();


    while(1){

        // elapsedtime = (millis()- timeprev) / 1000 ; 
        // // Read Accelerometer raw value
        // magX = read_raw_data(HMC5883L_REG_OUT_X_M);
        // magY = read_raw_data(HMC5883L_REG_OUT_Y_M);
        // magZ = read_raw_data(HMC5883L_REG_OUT_Z_M);
        // mag_angle =  (atan2(magX, magY)  * 180 / pi)* elapsedtime ; 
        // printf("%f \r",  mag_angle);
        // fflush(stdout);
        // timeprev = millis();
    
        float mx,my,mz = AK8963_conv();
        printf("X = %f | Y = %f | Z = %f \n",mx,my,mz);


    
    }
}







