from mpu9250_i2c import *

time.sleep(1) # delay necessary to allow mpu9250 to settle

print('recording data')
while 1:
    start_processing = MPU9250()

    print(start_processing)

    time.sleep(1)