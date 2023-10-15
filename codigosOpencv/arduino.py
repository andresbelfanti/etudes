
import serial
import random
import numpy as np
#import schedule
import time
import datetime
import unicodedata


x=np.array([0.0,0.0,0.0])
angle=np.array([0.0,0.0,0.0])
vel=np.array([0.0,0.0,0.0])
accel=np.array([0.0,0.0,0.0])
USB = "/dev/ttyUSB0"
ser = serial.Serial(USB, 9600)


#===========================================
def connect():
    try:
        ser = serial.Serial(USB, 9600)
        time.sleep(3)
    except Exception as e: 
        datalogger(e)
        exit()


def datalogger(event):
    now = datetime.datetime.now()
    print(event)
    print (now.day, now.month, now.year, now.hour, now.minute, now.second)

    tlog=str(now.day)+str(now.month)+".txt"
    with open(tlog, "a") as myfile:
        myfile.write(str(now.day)+" "+str(now.month)+" "+str(now.year)+" "+str(now.hour)+" " + str(now.minute)+" "+str(now.second)+" " +"\n")
        myfile.write(str(event)+ "\n")
        myfile.close()

def testserial():
    global ser
    try:
        msg = ser.read(ser.inWaiting()) # read everything in the input buffer
        #print (msg)
        return(msg) ## ACA MODIFICAR
    except Exception as e: 
        datalogger(e)
#=======================================================
def led(state):
    global ser
    try:
        ser.write((str(state)+"\n").encode())
    except Exception as e: 
        datalogger(e)
#=======================================================
def target():
    global x
    global vel
    #led("ledon")
    x=np.random.randint(10,30,3)
    vel= np.random.random_sample((3,))*0.01
    datalogger("new target" +str(x))

#===========================================================
def move():
    global angle
    global x
    global vel
    check=True

    if(np.all(angle==x)):
        
       # led("ledoff")
        try:
            if(check==True):
                #print("rest")
                check=False

        except Exception as e: 
            datalogger(e)

    else:
        for i in range(0,3):
            #accel[i]=abs(x[i]-angle[i])#PARA TRABAJAR LA ACELERACION
            #print(accel)
            if(angle[i]<x[i]):
                angle[i]=angle[i]+x[i]*(vel[i])
            if(angle[i]>x[i]):
                if(angle[i]<=0):
                    angle[i]=angle[i]+x[i]*(vel[i])
                else:
                    angle[i]=angle[i]-x[i]*(vel[i])

        angle=np.clip(angle, 0, 100)
        try:
            ser.write((str(angle[0])+", "+str(angle[1])+", "+str(angle[1]-angle[2]/2)+"\n").encode())
            print("angle: "+str(angle[0])+", "+str(angle[1])+", "+str(angle[1]-angle[2]/2)+"\n")
        
        except Exception as e: 
            datalogger(e)
#================================================
def shut():
    global angle
    global x
    global vel
    global ser
    x=np.array([0,0,0])
    #led("ledoff")
    for i in range(0,100):
        angle=angle-1
        angle=np.clip(angle, 0, 100)
        time.sleep(0.2)
        ser.write((str(angle[0])+", "+str(angle[1])+", "+str(angle[1]-angle[2]/2)+"\n").encode())
     
    datalogger("apagando")

#==================================================
def sleep():
   # led("ledoff")
    global angle
    global x
    global vel
    global ser
    x=np.array([0,0,0])
    #print("sleep")
    vel=np.array([1,1,1])
    try:
       # datalogger("sleep")
        for i in range(0,100):
            angle=angle-1
            angle=np.clip(angle, 0, 100)
            print("down: " +str(angle))
            ser.write((str(angle[0])+", "+str(angle[1])+", "+str(angle[1]-angle[2]/2)+"\n").encode())
            time.sleep(0.2)
    except:
        datalogger("sleep not")
    ser.write(("sleep"+"\n").encode())


#==================================================================
def wakeup():
    print("wake")
    #led("ledon")
    global ser
    try:
        ser.write(("wakeup"+"\n").encode())
        target()
    except Exception as e: 
        print("func wakeup:")
        datalogger(e)




#============================================|