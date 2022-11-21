import re

with open ("vr_log.txt", "r") as myfile:
    str_data = myfile.read().replace('\n', '')


## splitting individual readings in the log file
str_values = str_data.split("Printing Raw Sensor HMD values")

sf = 1000

## creating a list for each raw value parameter coordinates
Angular_acceleration = []
Linear_acceleration = []
Angular_velocity = []
Linear_velocity = []
Device_rotation = []
Device_position = []
Time = []
## need linear acceleration from the accelerometer, and the angular velocity from the Gyro 
Acc_Reading = []
Gyro_Reading = []


temp = 6.193533
temp_list = []
driftTemp = 0
for line in str_values: 
    X_values = re.findall('X=\S*', line)
    Y_values = re.findall('Y=\S*', line)
    Z_values = re.findall('Z=\S*', line)
    Pitch_value = re.findall('P=\S*', line)
    Yaw_value = re.findall('Y=\S*', line)
    Roll_value = re.findall('R=\S*', line)
    Time_value = re.findall('\s.....\s.....\s', line)


    ## Making sure we are reading the correct values from the log file 
    if (X_values and Y_values and Z_values and Pitch_value and Pitch_value and Yaw_value and Roll_value and Time_value): 
        ang_acc = [X_values[0], Y_values[0], Z_values[0][:-3]]
        lin_acc = [X_values[1], Y_values[1], Z_values[1][:-3]]
        ang_vel = [X_values[2], Y_values[2], Z_values[2][:-3]]
        lin_vel = [X_values[3], Y_values[3], Z_values[3][:-3]]
        time = Time_value[0][1:6]

        ## Removing the trailing -22 in the last param of each reading using [:-3]
        device_position = [X_values[4], Y_values[4], Z_values[4][:-3]]
        device_rotation = [Pitch_value[0], Yaw_value[0], Roll_value[0][:-3]]
        Angular_acceleration.append(ang_acc)
        Linear_acceleration.append(lin_acc)
        Linear_velocity.append(lin_vel)
        Angular_velocity.append(ang_vel)
        Device_rotation.append(device_rotation)
        Device_position.append(device_position)
        Time.append(time)
        ## need linear acceleration from the accelerometer, and the angular velocity from the Gyro 
        acc_read = [(float)(X_values[1][2:]) * sf, (float)(Y_values[1][2:]) * sf, (float)(Z_values[1][2:-3]) * sf]
        Acc_Reading.append(acc_read)
        gyro_read = [(float)(X_values[2][2:]) * 1000, (float)(Y_values[2][2:]) * 1000, (float)(Z_values[2][2:-3]) * 1000]
        Gyro_Reading.append(gyro_read)
        




SensorDataset = []
for i in range(0,len(Time)): 
    SensorDataset.append([Time[i], Acc_Reading[i][0], Acc_Reading[i][1], Acc_Reading[i][2], Gyro_Reading[i][0], Gyro_Reading[i][1], Gyro_Reading[i][2]])


import csv

with open("vrSensorRawDataset.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(SensorDataset)





from Kalman import KalmanAngle
import math


kalmanX = KalmanAngle()
kalmanY = KalmanAngle()

RestrictPitch = False	#Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
radToDeg = 57.2957786 # 180/pi
kalAngleX = 0
kalAngleY = 0

gyroXAngle = 0
gyroYAngle = 0

counter = 0

kalman_roll = []

gyroXAngle_roll = []
acc_roll = []



if (RestrictPitch):
    roll = math.atan2(Acc_Reading[0][1],Acc_Reading[0][2]) * radToDeg
    pitch = math.atan(-Acc_Reading[0][0]/math.sqrt((Acc_Reading[0][1]**2)+(Acc_Reading[0][2]**2))) * radToDeg
else:
    roll = math.atan((math.sqrt(sf)/sf)*(Acc_Reading[0][1]/math.sqrt((Acc_Reading[0][0]**2)+(Acc_Reading[0][2]**2)))) * radToDeg
    pitch = math.atan2(-Acc_Reading[0][0],Acc_Reading[0][2]) * radToDeg

kalmanX.setAngle(roll)
kalmanY.setAngle(pitch)
gyroXAngle = roll;
gyroYAngle = pitch;
compAngleX = roll;
compAngleY = pitch;

temp = []
temp_angle = 0
while counter < (len(Acc_Reading)-1):
    counter = counter+1   

    #Read Accelerometer raw value
    accX = Acc_Reading[counter][0]
    accY = Acc_Reading[counter][1]
    accZ = Acc_Reading[counter][2]

    #Read Gyroscope raw value
    gyroX = Gyro_Reading[counter][0] 
    gyroY = Gyro_Reading[counter][1] 
    gyroZ = Gyro_Reading[counter][2] 

    dt = 0.01
    

    if (RestrictPitch):
        roll = math.atan2(accY,accZ) * radToDeg
        pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
    else:
        roll = math.atan((math.sqrt(sf)/sf)*(accY/math.sqrt((accX**2)+(accZ**2)))) * radToDeg
        pitch = math.atan2(-accX,accZ) * radToDeg

    gyroXRate = gyroX
    gyroYRate = gyroY

    temp_angle += gyroXRate * dt * 57
    temp.append(temp_angle)

    if (RestrictPitch):

        if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
            kalmanX.setAngle(roll)
            complAngleX = roll
            kalAngleX   = roll
            gyroXAngle  = roll
        else:
            kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

        if(abs(kalAngleX)>90):
            gyroYRate  = -gyroYRate
            kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)
    else:

        if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
            kalmanY.setAngle(pitch)
            complAngleY = pitch
            kalAngleY   = pitch
            gyroYAngle  = pitch
        else:
            kalAngleY = kalmanY.getAngle(pitch,gyroYRate,dt)

        if(abs(kalAngleY)>90):
        #if(1):
            gyroXRate  = -gyroXRate
            kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

    #angle = (rate of change of angle) * change in time
    gyroXAngle = gyroXRate * dt
    gyroYAngle = gyroYAngle * dt
    
    if ((gyroXAngle < -180) or (gyroXAngle > 180)):
        gyroXAngle = kalAngleX
    if ((gyroYAngle < -180) or (gyroYAngle > 180)):
        gyroYAngle = kalAngleY
    
    kalman_roll.append(kalAngleX)    
    gyroXAngle_roll.append(gyroXAngle * 57)
    acc_roll.append(compAngleX)


import pprint


import matplotlib.pyplot as plt
overlapping = 0.550

#plt.plot(Gyro_Reading[0], c='red', alpha=overlapping, lw=5)
#plt.plot(Rotation, c='green', alpha=overlapping, lw=5)
#plt.plot(Oculus_Roll, c='blue', alpha=overlapping, lw=5)
plt.plot(kalman_roll, c='blue',  alpha=overlapping, lw=2)
#plt.plot(temp, c='red',  alpha=overlapping, lw=2)
#plt.plot(gyroXAngle_roll, c='green',  alpha=overlapping, lw=2)


plt.ylabel('Gyroscope Drift')
plt.show()
