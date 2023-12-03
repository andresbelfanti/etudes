
##=======deteccion de circulos y envio por osc + ventana==============================================
import cv2
import numpy as np
from pythonosc.udp_client import SimpleUDPClient
from pythonosc import osc_message_builder
from pythonosc import udp_client
import serial
import time
import datetime
import math
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib.transforms import Affine2D
import mpl_toolkits.axisartist.floating_axes as floating_axes
from rich.console import Console
from rich.text import Text
from rich.columns import Columns

console = Console()
# falta funcion que abre el database previo y agrega datos

df = pd.DataFrame(columns= ['angleX', 'angleY', 'angleZ', 'posX', 'posY', 'posZ', 'gyro0', 'gyro1', 'lidar', 'accel', 'deccel', 'stops', 'works']) 
print(df.shape)

    # Creating figure
fig = plt.figure(figsize = (10, 7))
ax = plt.axes(projection ="3d")
plt.title("clasificacion posiciones")


##==================FUNCIONES DE SCHEDULE, LOGGER Y OSC=====================

###-plotter of positions -----------------------
def plotter():
    # Creating plot
    ax.scatter3D(df["posX"][0:], df["posY"][0:], df["posZ"][0:], c = df["works"][0:])
   
    # show plot
    plt.ion()
    plt.pause(0.001)
    plt.show()

## -- save database-------------
def save(angles, pos, data, speed, n):
    stops = 0
    if any(x == 1 for x in data[4:8]):
        stops = 1

    df.loc[len(df)] = angles[0], angles[1], angles[2], pos[0], pos[1], pos[2], data[0], data[1], data[2], speed[0], speed[1], stops, n
    df.to_csv("database.csv", encoding='utf-8')
    print("df len: "+ str(len(df)))
    print(df.shape)
    plotter()

# DATALOGGER FUNCION ====================================================
def datalogger(event):
    now = datetime.datetime.now()
   # print (now.day, now.month, now.year, now.hour, now.minute, now.second)

    tlog=str(now.day)+str(now.month)+".txt"
    with open(tlog, "a") as myfile:
        myfile.write(str(now.day)+"/"+str(now.month)+"/"+str(now.year)+" "+str(now.hour)+":" + str(now.minute)+":"+str(now.second)+" " +"\n")
        myfile.write(str(event)+ "\n")
        myfile.close()

#=====================OSC SENDER=======================
def oscSender(data):
    ip = "127.0.0.1"
    port  = 12000
    client = SimpleUDPClient(ip, port)  # Create client  
    
    try:
        msg = osc_message_builder.OscMessageBuilder(address = '/cv')
        msg.add_arg(len(data[0]))
        for n in data[0]:
            for x in n:
                msg.add_arg(x)

        #msg format [/cv, arraylength,[[x0],[x1]...]]
            
        msg = msg.build()
        client.send(msg)

    except:
        client.send_message("/prueba", "error")  # Send message with int, float and string
        #console.print("osc send error", style="red")()

##=======================FUNCIONES DE CAMARA Y ANALISIS VISUAL====================================
#=========circles deteccion=====================================

def circleDetection(frame, treshold1, treshold2):
    # convert image to grayscale
    try:
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    except:
        print("cvt error")
    # apply a blur using the median filter
    img = cv2.medianBlur(img, 5)    
    # find circles in grayscale image using the Hough transform
    circles = cv2.HoughCircles(image=img, method=cv2.HOUGH_GRADIENT, dp=1, 
                            minDist=4, param1=treshold1, param2=treshold2, minRadius=1, maxRadius=10)
    
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
        data=circles.tolist()
       # console.print(data[0], style="black")
        oscSender(data)
        for co, i in enumerate(circles[0, :]):
            # draw the outer circle with radius i[2] in green
            cv2.circle(frame,(int(i[0]),int(i[1])),int(i[2]),(0,255,0),1)    
    try:
        cv2.imshow('Video', frame)
    except:
        console.print("frame lost", style="red on white")()



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
    conexion = True
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200)
        time.sleep(3)
    except Exception as e: 
        datalogger(e)
        conexion = False
       # exit()
    return(conexion)

##  serial con return ========================
def readSerial():
    msg = []
    global ser
    msg = ser.read(ser.inWaiting()) # read everything in the input buffer
    return(msg)


##=========== posicion de inicio del brazo

def initAngles():
    return([0,150, -180])
#==============send angles=========================
def moveToAngle(anglesObj, anglesAct): #reenvia speed también
    print("movetoangle")
    global ser
    global angles
    newAngles =[0,0,0]
    newAngles[0]=anglesObj[0]-anglesAct[0]# base
    newAngles[1]=(anglesObj[1]-anglesAct[1])*-1 #hombro
    newAngles[2] = (anglesObj[2]-anglesAct[2])*-1 #codo
    print("SENDING NEW ANGLES: ")
    print(newAngles)
    try:
        ser.write(str(str(newAngles[0])+','+str(newAngles[1])+','+str(newAngles[2])+','+str(anglesObj[3])+','+str(anglesObj[4])+','+str(anglesObj[5])+"\n").encode())

    except Exception as e:
        datalogger(e)
        console.print("moveAngle"+str(e), style="magenta")

    return(newAngles)
#    anglePlot(angles[0], angles[1], angles[2]) 
#====================sends==============
def sends(data): #envia otra data, como luces, sleep
    global ser
    try:
        ser.write((data +"\n").encode())
    except Exception as e:
        datalogger(e)
        console.print(e, style="magenta")



