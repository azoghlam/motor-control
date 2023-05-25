#include <wiringPiI2C.h>
#include <wiringPi.h>
// #include "includes/wiringPiI2C.h"
// #include "includes/wiringPi.h"

#define _USE_MATH_DEFINES
 
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <math.h>

#define Device_Address 0x68	/*Device Address/Identifier for MPU6050*/

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

    int16_t rawMagX, rawMagY, rawMagZ;

int fd;

int16_t rawAccX, rawAccY, rawAccZ, rawTemp,
rawGyroX, rawGyroY, rawGyroZ;
 
float gyroXoffset= 1.45;
float gyroYoffset= 1.23;
float gyroZoffset= -1.32;

float sX,sY,sZ, vX, vY, vZ,dvX, dvY, dvZ;
float magX,magY,magZ;

float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;

float angleGyroX, angleGyroY, angleGyroZ,
angleAccX, angleAccY, angleAccZ;

float angleX, angleY, angleZ;

float interval;
long preInterval;

float accCoef = 0.02f;
float gyroCoef = 0.98f;

short read_raw_data(int addr){
	short high_byte,low_byte,value;
	high_byte = wiringPiI2CReadReg8(fd, addr);
	low_byte = wiringPiI2CReadReg8(fd, addr+1);
	value = (high_byte << 8) | low_byte;
	return value;
}

void calcOffset(bool console, uint16_t delayBefore, uint16_t delayAfter) {
    float x = 0, y = 0, z = 0;
	int16_t rx, ry, rz;

    delay(delayBefore);
    if(console){
        std::cout << "Calculating gyro offsets" << std::endl;
    }
  for(int i = 0; i < 3000; i++){
    if(console && i % 1000 == 0){
      printf(".");
    }
    
    // wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x43);
    // wire->beginTransmission(MPU6050_ADDR);
    // wire->write(0x43);
    // wire->endTransmission(false);
    // wire->requestFrom((int)MPU6050_ADDR, 6);

    rx = read_raw_data(GYRO_XOUT_H);
    ry = read_raw_data(GYRO_YOUT_H);
    rz = read_raw_data(GYRO_ZOUT_H);

    x += ((float)rx) / 65.5;
    y += ((float)ry) / 65.5;
    z += ((float)rz) / 65.5;
  }
  gyroXoffset = x / 3000;
  gyroYoffset = y / 3000;
  gyroZoffset = z / 3000;

  if(console){
	  std::cout << "Calculating gyro offsets" << std::endl;
	  std::cout << "X: " << gyroXoffset << std::endl;
	  std::cout << "Y: " << gyroXoffset << std::endl;
	  std::cout << "Z: " << gyroXoffset << std::endl;
	delay(delayAfter);
	}
}

void update(){
    // wire->beginTransmission(MPU6050_ADDR);
	// wire->write(0x3B);
	// wire->endTransmission(false);
	// wire->requestFrom((int)MPU6050_ADDR, 14);

    // WRIETE FN????????????????????????
	rawAccX = read_raw_data(ACCEL_XOUT_H);
	rawAccY = read_raw_data(ACCEL_YOUT_H);
	rawAccZ = read_raw_data(ACCEL_ZOUT_H);
	rawGyroX = read_raw_data(GYRO_XOUT_H);
	rawGyroY = read_raw_data(GYRO_YOUT_H);
	rawGyroZ = read_raw_data(GYRO_ZOUT_H);
    rawMagX = read_raw_data(AK8963_HXL);
	rawMagY = read_raw_data(AK8963_HYL);
	rawMagZ = read_raw_data(AK8963_HZL);	
	
    temp = (rawTemp + 12412.0) / 340.0;

	accX = ((float)rawAccX) / 16384.0;
	accY = ((float)rawAccY) / 16384.0;
	accZ = ((float)rawAccZ) / 16384.0;

	angleAccX = atan2(accY, sqrt(accZ * accZ + accX * accX)) * 360 / 2.0 / M_PI;
	angleAccY = atan2(accX, sqrt(accZ * accZ + accY * accY)) * 360 / -2.0 / M_PI;

	gyroX = ((float)rawGyroX) / 65.5;
	gyroY = ((float)rawGyroY) / 65.5;
	gyroZ = ((float)rawGyroZ) / 65.5;

	gyroX -= gyroXoffset;
	gyroY -= gyroYoffset;
	gyroZ -= gyroZoffset;

	interval = (millis() - preInterval) * 0.001;

	angleGyroX += gyroX * interval;
	angleGyroY += gyroY * interval;
	angleGyroZ += gyroZ * interval;

	

	vX = dvX + accX*interval;
	vY = dvX + accX*interval;
	vZ = dvX + accX*interval;

	sX = (sX*interval) + (0.5 * accX * (interval*interval));
	sY = (sY*interval) + (0.5 * accY * (interval*interval));
	sZ = (sZ*interval) + (0.5 * accZ * (interval*interval));

	angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
	angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
	angleZ = angleGyroZ;

	preInterval = millis();
}

void init_MPU () {
    wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
    wiringPiI2CWriteReg8 (fd, CONFIG, 0x00);		/* Write to Configuration register */
    wiringPiI2CWriteReg8 (fd, GYRO_CONFIG, 24);	/* Write to Gyro Configuration register */
    // ^^^ was 24
    // wiringPiI2CWriteReg8 (fd, ACCEL_CONFIG,0x00); 
    wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01);	/* Write to power management register */
    wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01);	/*Write to interrupt enable register ???*/
    angleGyroX = 0;
    angleGyroY = 0;
    angleX = angleAccX;
    angleY = angleAccY;
    preInterval = millis();

    update();
}

int main() {
    fd = wiringPiI2CSetup(Device_Address); 
    calcOffset(false, 0, 0);
    init_MPU();
	
	while(1){
		update();
		printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\r", angleX, angleY, angleZ,sX, sY, sZ,rawMagX ,rawMagY,rawMagZ);
		fflush(stdout);
	}
	
    return 0;
}