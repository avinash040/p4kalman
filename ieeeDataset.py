import csv

rollGyro = []

counter = 0
angle = 0
xGyro = []
AX = []
AY = []
AZ = []

GX = []
GY = []
GZ = []

with open("raw_pitch_slow.txt", "r", newline="") as f:
    data_reader = csv.reader(f)
    for row in data_reader: 
        counter = counter + 1 
        if (counter > 1000): 
            break
        #print(row)
        #print(row[17],row[18],row[19] )
        #print(counter)
        temp = float(row[17]) * 0.01
        angle += temp
        xGyro.append(temp)  
        GX.append(float(row[16]))
        GY.append(float(row[17]))
        GZ.append(float(row[18]))
        AX.append(float(row[19]))
        AY.append(float(row[20]))
        AZ.append(float(row[21]) + 0.000001)



comp = []
kal = []
roll_list = []

import math
import Kalman

kalmanX = Kalman.KalmanAngle()
kalmanY = Kalman.KalmanAngle()

RestrictPitch = False	#Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
radToDeg = 1
kalAngleX = 0
kalAngleY = 0

print(AX[0], AY[0], AZ[0])


if (RestrictPitch):
    roll = math.atan2(AY[0],AZ[0]) 
    pitch = math.atan(-AX[0]/math.sqrt((AY[0]**2)+(AZ[0]**2)+(AX[0]**2)))
else:
    roll = math.atan(AY[0]/math.sqrt((AY[0]**2)+(AZ[0]**2)+(AX[0]**2))) 
    pitch = math.atan2(-AX[0],AZ[0]) 
print(roll)
kalmanX.setAngle(roll)
kalmanY.setAngle(pitch)
gyroXAngle = roll;
gyroYAngle = pitch;
compAngleX = roll;
compAngleY = pitch;

counter = 0
while counter < (len(AX)-1):
    counter = counter+1  
	
    #Read Accelerometer raw value
    accX = AX[counter]
    accY = AY[counter]
    accZ = AZ[counter]

    #Read Gyroscope raw value
    gyroX = GX[counter]
    gyroY = GY[counter]
    gyroZ = GZ[counter]

    dt = 0.01

    if (RestrictPitch):
        roll = math.atan2(AY[counter],AZ[counter]) 
        pitch = math.atan(-AX[counter]/math.sqrt((AY[counter]**2)+(AZ[counter]**2)+(AX[counter]**2)))
    else:
        roll = math.atan(AY[counter]/math.sqrt((AY[counter]**2)+(AZ[counter]**2)+(AX[counter]**2))) 
        pitch = math.atan2(-AX[counter],AZ[counter]) 

    gyroXRate = gyroX/131
    gyroYRate = gyroY/131

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
            gyroXRate  = -gyroXRate
            kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

    #angle = (rate of change of angle) * change in time
    gyroXAngle = gyroXRate * dt
    gyroYAngle = gyroYAngle * dt

    #compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained from accelerometer
    compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
    compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

    if ((gyroXAngle < -180) or (gyroXAngle > 180)):
        gyroXAngle = kalAngleX
    if ((gyroYAngle < -180) or (gyroYAngle > 180)):
        gyroYAngle = kalAngleY

    #print("Angle X: " + str(kalAngleX)+"   " +"Angle Y: " + str(kalAngleY))
    #print(str(roll)+"  "+str(gyroXAngle)+"  "+str(compAngleX)+"  "+str(kalAngleX)+"  "+str(pitch)+"  "+str(gyroYAngle)+"  "+str(compAngleY)+"  "+str(kalAngleY))

    comp.append(compAngleX)
    kal.append(kalAngleX)
    roll_list.append(roll)














import matplotlib.pyplot as plt
overlapping = 0.550

plt.plot(kal, c='green', alpha=overlapping, lw=2)
#plt.plot(kal, c='red', alpha=overlapping, lw=2)
plt.plot(comp, c='yellow', alpha=overlapping, lw=2)
#plt.plot(comp, c='blue',  alpha=overlapping, lw=2)




plt.ylabel('Gyroscope Drift')
plt.show()



#17,18,19) Raw Gyroscope Readings (deg/s) of IMU 1 about x,y,z (3 values) 
#20,21,22) Raw Accelerometer Readings (g - gravitational constant) of IMU 1 about x,y,z (3 values)