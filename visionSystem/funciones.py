
##=======deteccion de circulos y envio por osc + ventana==============================================
import cv2
import numpy as np
#from pythonosc.udp_client import SimpleUDPClient
import serial
import time
import datetime
import math
import pandas as pd
# Creating Empty DataFrame and Storing it in variable df 
df = pd.read_csv("/home/andrex/ANDY/shared/CCEBA 2023/CCEBA_codes y material/etudes/visionSystem/database.csv") 
angles=[]
data=[0,0,0,0,0,0,0]


##==================FUNCIONES DE SCHEDULE, LOGGER Y OSC=====================
# DATALOGGER FUNCION ====================================================
def datalogger(event):
    now = datetime.datetime.now()
   # print (now.day, now.month, now.year, now.hour, now.minute, now.second)

    tlog=str(now.day)+str(now.month)+".txt"
    with open(tlog, "a") as myfile:
        myfile.write(str(now.day)+" "+str(now.month)+" "+str(now.year)+" "+str(now.hour)+" " + str(now.minute)+" "+str(now.second)+" " +"\n")
        myfile.write(str(event)+ "\n")
        myfile.close()

#=====================OSC SENDER=======================
def oscSender(data):
   print("oscSender")
   ip = "192.168.0.31"
   port  = 12000
   client = SimpleUDPClient(ip, port)  # Create client  
   client.send_message("/cv", data.tolist())  # Send message with int, float and string

##=======================FUNCIONES DE CAMARA Y ANALISIS VISUAL====================================
#=========circles deteccion=====================================

def circleDetection(frame):
    # convert image to grayscale
    try:
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    except:
        print("cvt error")
    # apply a blur using the median filter
    img = cv2.medianBlur(img, 5)    
    # find circles in grayscale image using the Hough transform
    circles = cv2.HoughCircles(image=img, method=cv2.HOUGH_GRADIENT, dp=0.009, 
                            minDist=0.01, param1=50, param2=10, minRadius=1, maxRadius=10)

#gray: Input image (grayscale).
#circles: A vector that stores sets of 3 values: xc,yc,r for each detected circle.
#HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV.
#dp = 1: The inverse ratio of resolution.
#min_dist = gray.rows/16: Minimum distance between detected centers.
#param_1 = 200: Upper threshold for the internal Canny edge detector.
#param_2 = 100*: Threshold for center detection.
#min_radius = 0: Minimum radius to be detected. If unknown, put zero as default.
#max_radius = 0: Maximum radius to be detected. If unknown, put zero as default
    if np.any(circles):
       # oscSender(circles) #acá en vez de mandarlo directo , hacer un merge de data con PANDAS
        for co, i in enumerate(circles[0, :]):
            # draw the outer circle with radius i[2] in green
            cv2.circle(frame,(int(i[0]),int(i[1])),int(i[2]),(0,255,0),1)
            
    try:
        cv2.imshow('Video', frame)
    except:
        print("frames")


##=========== DESDE ACÁ TODAS LAS FUNCIONES RELACIONADAS A COMUNICACION ARDUINO Y MOV ROBOT==================
#===================IK SOLVER===========================
def moveToPos(x, y, z):
    b = math.atan2(y, x) * (180 / 3.1415);  # base angle
    l = math.sqrt(x * x + y * y);  # x and y extension
    h = math.sqrt(l * l + z * z);
    phi = math.atan(z / l) * (180 / 3.1415);
    theta = math.acos((h / 2) / 75) * (180 / 3.1415);
    a1 = phi + theta;  # angle for first part of the arm
    a2 = phi - theta;  # angle for second part of the arm

    angles = [b, a1, a2]

    return(angles)
#=================connect a USB======================
def connect():
    global ser
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200)
        time.sleep(3)
    except Exception as e: 
        datalogger(e)
       # exit()

##  serial con return ========================
def readSerial():
    global ser
    global data
    try:
        msg = ser.read(ser.inWaiting()) # read everything in the input buffer
        data=msg.split(",")
        #print (msg)
    except Exception as e: 
        datalogger(e)

#==============send angles=========================
def moveToAngle(angles):
    print("movetoangle")
    global ser
    global data
    angles[0]=angles[0]-data[0]# hombro
    angles[1]=angles[1]-data[1]+data[0] #codo 
    angles[2] = angles[2]-data[2] #base - No tiene sensor!!!
    try:
        ser.write(str(angles)+"\n")
    except Exception as e:
        datalogger(e)
        print(e)

#====================sends==============
def sends(data): #envia otra data, como luces, sleep
    global ser
    try:
        ser.write(data +"\n")
    except Exception as e:
        datalogger(e)
        print(e)

def save(q,w,e,r,t,y,u,i):
    df.loc[-1] = q,w,e,r,t,y,u,i  # adding a row
    df.to_csv("database.csv", encoding='utf-8')


