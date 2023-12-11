import serial
import time
import math
import numpy as np
import matplotlib.pyplot as plt
# from mpl_toolkits import mplot3d
import csv
import pandas as pd
from matplotlib.animation import FuncAnimation

### Create a csv file ###
header = ['Degree', 'XCoordinate', 'YCoordinate']
with open('storeData.csv', 'w', encoding='UTF8', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(header)

# Create a struct #
class my2DStruct:
    # value1: store yawDegree
    # value2: store distance 
    def __init__ (self, value1, value2):
        self.value1= value1
        self.value2= value2

# Create a two dimension array to store and implement filter with 3 row/ 5000 collumm # 
array_2d = [[my2DStruct(0.0, 0.0) for _ in range(8000)] for _ in range(5)]
xAxes=[]
yAxes=[]
posArray= []

# # Use for collect only 1 yawDegree #
# def checkInArray(value):
#     idx=0

#     # If posArray is empty -> Get the value to Array # 
#     if(len(posArray)==0):
#          posArray.append(value)
#     # Else if posArray already has -> return 0 to not add to Array # 
#     else:
#         for idx in range(len(posArray)):
#             if(value== posArray[idx]):
#                 return 0
#         return 1



# Process Array to get posArray that is an array to store only one value for each yawDegree # 
def processArray():
    idx=0
    for idx in range(8000):                                    
        if(checkInArray(array_2d[0][idx].value1)==1):
            posArray.append(array_2d[0][idx].value1)

# Function to calculate rootMeanSquare of each yawDegree # 
def calculateRootMeanSquare(value):
    idxRow=0
    idxCollum=0
    count=1
    RMS=0

    # idxRow here need to adjust ( Because now is setting 3 rounds -> range 2 is enough )
    # This function will scan for all -> Find the yawDegree to calculate the mean of value # 
    for idxRow in range(5):
        for idxCollum in range(8000):
            if(array_2d[idxRow][idxCollum].value1== value and array_2d[idxRow][idxCollum].value2 !=0 and array_2d[idxRow][idxCollum].value2 <500):
                RMS= RMS + array_2d[idxRow][idxCollum].value2* array_2d[idxRow][idxCollum].value2
                count= count+1
                break
    return math.sqrt(RMS/count)

#UPDATE

import numpy as np
from filterpy.kalman import KalmanFilter

# Initialize Kalman Filter
kf = KalmanFilter(dim_x=2, dim_z=1)  # 2 states, 1 measurement

# Define transition matrix (state transition model)
kf.F = np.array([[1, 1],
                 [0, 1]])

# Define measurement function
kf.H = np.array([[1, 0]])

# Define measurement noise covariance
kf.R = 0.1

# Initialize state and covariance
kf.x = np.array([0, 0])  # Initial state [position, velocity]
kf.P = np.eye(2)  # Initial covariance

def rootMeanSquare():
    # Your existing code
    for idx in range(len(posArray)):
        RMSResult = calculateRootMeanSquare(posArray[idx])
        
        # Measurement from sensor
        z = RMSResult
        
        # Kalman prediction step
        kf.predict()
        
        # Kalman update step
        kf.update(z)
        
        # Get the filtered position estimation
        filtered_position = kf.x[0]
        
        # Calculate xAxesValue and yAxesValue to draw 
        xAxesValue = filtered_position * (np.cos(posArray[idx] * np.pi / 180))
        yAxesValue = filtered_position * (np.sin(posArray[idx] * np.pi / 180))
        xAxes.append(xAxesValue)
        yAxes.append(yAxesValue)
        
        # This writeList is to store in file
        writeList = [posArray[idx], xAxesValue, yAxesValue]
        with open('storeData.csv', 'a', encoding='UTF8', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(writeList)
                    

# # Function to call each yawDegree to calculate rootMeanSquare       
# def rootMeanSquare():
#     idx=0
#     RMSResult=0

#     # For idx of posArray to calculate the mean of its yawDegree | For example: at yawDegree: 0.8 -> We have 0.7; 1.0 -> Get the mean of its distance #
#     for idx in range(len(posArray)):
#         RMSResult=calculateRootMeanSquare(posArray[idx])

#         # Calculate xAxesValue and yAxesValue to draw 
#         xAxesValue= RMSResult*(np.cos(posArray[idx]*3.14/180))
#         yAxesValue= RMSResult*(np.sin(posArray[idx]*3.14/180))
#         xAxes.append(xAxesValue)
#         yAxes.append(yAxesValue)

#         # This writeList is to store in file # 
#         writeList = [posArray[idx], xAxesValue, yAxesValue]
#         with open('storeData.csv', 'a', encoding='UTF8', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow(writeList)        
    

# Define function drawing 2D Mapping
def mode2D():

    # Create a new figure #
    fig, ax = plt.subplots()

    # Variable to check limit of round to measure # 
    limit_of_data = True

    # Round of array to draw 2D # 
    rowOfArray=0

    # Collum of array to draw 3D # 
    # If collumofArray==0 -> Draw 2D #
    collumOfArray=0

    # Start while # 
    while limit_of_data:
        while arduino.in_waiting == 0:
            pass

        # Read line sent from arduino depending on Mode #
        # Process the dataPacket receiving from arduino # 
        dataPacket = arduino.readline()
        dataPacket  = str(dataPacket, 'utf-8')    
        dataPacket  = dataPacket.strip('\r\n')
        print(dataPacket)
        
        # If dataPacket == End Data -> Stop ! # 
        if (dataPacket == 'End Data'):
            limit_of_data = False
        # If dataPacket == Round up -> Increase size of array to store value -> Draw # 
        elif (dataPacket == 'Round Up'):
            rowOfArray= rowOfArray+1
            collumOfArray=0   
                     
        else:
            # Get the line from arduino # 
            splitData = dataPacket.split(',')
            # [0] : PosX 0-> 360 
            # [1] : PosY 0-> 360 
            # [2] : distance get from arduino 
            yawDegree= float(splitData[0])
            pitchDegree= float(splitData[1])
            dist    = float(splitData[2])

            # Set value yawDegree to PosX (value1) || Set value dist -> value2 #
            array_2d[rowOfArray][collumOfArray].value1= yawDegree
            array_2d[rowOfArray][collumOfArray].value2= dist
          
            # Each + 1 collumm -> up round +1 #
            collumOfArray= collumOfArray+1
            # Print to console #
            print(yawDegree,dist)

            ax.clear

    # End while # 

    # Process Array to get the quality of 2D mapping # 
    # Process to get an Array of each distance #
    # processArray()

    # Then when get the Array of each distance -> Process to get rootMeanSquare and get to xAxes, yAxes to draw # 
    rootMeanSquare()
    
    # Add to draw # 
    ax.scatter(xAxes,yAxes,s=0.1,c='green')
    
    for i in range(1600):
        ax.scatter(xAxes[i],yAxes[i],s=0.1,c='red')

    ax.scatter(0, 0, s=30, c = 'red')
    ax.set_xlim(-400, 400)
    ax.set_ylim(-400, 400)
    plt.show()


### Start main thread of programme ###

# Connect with arduino through serial 
arduino = serial.Serial(
    port='COM7',
    baudrate=115200
)

# Delay time
time.sleep(1)

# Variable to control program working or not #       
programWorking= True


while programWorking:
    # Three modes: 0: Stop Lidar || 1: 2D 5 rounds || 2: 3D 90 degree pitch (Test may be 20 degree )
    # Taking input from user

    print("Waiting for input number: ")
    modeWorking = input("Enter a mode working: ")

    # Send to arduino (change modeWorking to bytes)
    arduino.write(bytes(modeWorking, 'utf-8'))

    # Delay 0.05 s 
    time.sleep(0.05)

    # Depend on which mode to start 
    if(modeWorking== '0'):
        programWorking= False
    elif (modeWorking== '1'):
        mode2D()

#### End Main Thread Program ####
