#include <wiringPi.h>
#include <wiringPiI2C.h>

// #include <includes/wiringPi.h>
// #include <includes/wiringPiI2C.h>

#include <iostream>
#include <cmath>
#include <math.h>

#define _USE_MATH_DEFINES

#define HMC5883L_ADDRESS              0x1E

#define HMC5883L_REG_CONFIG_A         0x00
#define HMC5883L_REG_CONFIG_B         0x01
#define HMC5883L_REG_MODE             0x02
#define HMC5883L_REG_OUT_X_M          0x03
#define HMC5883L_REG_OUT_X_L          0x04
#define HMC5883L_REG_OUT_Z_M          0x05
#define HMC5883L_REG_OUT_Z_L          0x06
#define HMC5883L_REG_OUT_Y_M          0x07
#define HMC5883L_REG_OUT_Y_L          0x08
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

int fd;

int16_t magX, magY, magZ, mag_angle;
float pi = 3.14159265359 ; 
// float elapsedtime, time, timeprev ;



void init() {

    wiringPiI2CWriteReg8 (fd, HMC5883L_REG_CONFIG_A , 0x70);	 //write to Configuration Register A
    wiringPiI2CWriteReg8 (fd, HMC5883L_REG_CONFIG_B , 0xa0);	//Write to Configuration Register B for gain
    wiringPiI2CWriteReg8 (fd, HMC5883L_REG_MODE , 0);	        //Write to mode Register for selecting mode

}

short read_raw_data(int addr) {

    short high_byte, low_byte, value ;
    high_byte = wiringPiI2CReadReg8(fd, addr) ;
    low_byte = wiringPiI2CReadReg8(fd, addr+1) ;
    value = (high_byte << 8) | low_byte ;
    return value ;

}


int main()
{
    fd = wiringPiI2CSetup(HMC5883L_ADDRESS);

    unsigned int time;
 float elapsedtime, timeprev ;
  // initialization function
        init();

    while(1){


        timeprev = time ;
        time = millis() ;
        elapsedtime = (time - timeprev) / 1000 ; 

      
        
        // Read Accelerometer raw value
        magX = read_raw_data(HMC5883L_REG_OUT_X_M);
        magY = read_raw_data(HMC5883L_REG_OUT_Y_M);
        magZ = read_raw_data(HMC5883L_REG_OUT_Z_M);

        mag_angle =  (atan2(magX, magY)  * 180 / pi)* elapsedtime ; 

        printf("%f \r",  mag_angle);
        
        
        
      
    
    
    }
}







