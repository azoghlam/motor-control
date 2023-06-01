from mpu9250_i2c import *

time.sleep(1) # delay necessary to allow mpu9250 to settle

MPU6050_start()
AK8963_start()

print('recording data')
while 1:
    # print('mpu')
    ax,ay,az,wx,wy,wz= mpu6050_conv() # read and convert mpu6050 data
    # print("ak")
    mx,my,mz,mt = AK8963_conv() # read and convert AK8963 magnetometer data
    # try:
        
    # except:
    #     continue
    
    print('{}'.format('-'*30))
    print('accel [g]: x = {0:2.2f}, y = {1:2.2f}, z = {2:2.2f} '.format(ax ,ay ,az))
    # print('accel [g]: x = {0:2.2f}, y = {1:2.2f}, z {2:2.2f}= '.format(ax,ay,az))
    print('gyro [dps]:  x = {0:2.2f}, y = {1:2.2f}, z = {2:2.2f}'.format(wx,wy,wz))
    print('mag [uT]:   x = {0:2.2f}, y = {1:2.2f}, z = {2:2.2f}'.format(mx,my,mz))
    print('heading [uT]:   x = {0:2.2f}'.format(mt))
    print('{}'.format('-'*30))
    time.sleep(1)