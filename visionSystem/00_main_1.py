
#import unicodedata
#import schedule
import time
import os
#import subprocess
#import unidecode
import datetime
import numpy as np
import cv2
from pythonosc.udp_client import SimpleUDPClient
import tkinter as tk
import serial
  

data = [0,0,0,0,0,0,0]
pos = [1,1,1]
angles = [0,0,0]
speed = [0,0]

from funciones import  datalogger, circleDetection, moveToPos, moveToAngle, connect, readSerial, sends, save

##===============GUI============================================
def draw(event):# este evento que sucede por el mouse - despues en automatico
    global pos
    # Get the current mouse position
    pos= [event.x, event.y, Slider1.get()]
    zn=pos[2]*0.02
    # Draw a dot on the canvas at the current mouse position 
    canvas.create_oval(pos[0]-zn, pos[1]-zn, pos[0]+zn, pos[1]+zn, fill='black')
    #text.insert('1.0', 'x: '+ str(pos[0]) + 'y: ' + str(pos[1])+ 'z: ' + str(pos[2])+ '\n')
    #text.insert('1.0', 'lidar: '+ str(data[0]) + 'gyrohombro: ' + str(data[1])+ 'gyrocodo: ' + str(data[2])+ '\n')

def moveYprint():
    global pos
    global angles
    global speed
    global data
    speed =[Slider2.get(), Slider3.get()]
    angles= moveToPos(pos[0]*0.01, pos[1]*0.01, pos[2]*0.1)
    yes=True

    sends("speed:"+str(speed)+'\n')
    moveToAngle(angles, data)
    
    text.insert('1.0', 'pos:  '+ str(pos)+'\n')
    text.insert('1.0', "speed,"+str(speed)+'\n')
    text.insert('1.0', 'Angles,  '+ str(angles)+'\n')
   
   # save([pos,angles, speed, yes])
##---------------------------------------------------

# Create a Tkinter window
window = tk.Tk()
# Create a Tkinter canvas
canvas = tk.Canvas(window, width=600, height=400, bg='green')
# Draw the cartesian axis (or grid) on the canvas
canvas.create_line(0, 200, 600, 200, width=1)  # x-axis
canvas.create_line(300, 0, 300, 600, width=1)  # y-axis
## elementos dentro de la ventana
text = tk.Text(window, height=8)
text.grid(row=1, column=0)
text.insert('1.0', 'INFO')
move_button = tk.Button(window, text='move',  command= lambda: moveYprint()) 
move_button.grid(row=0, column=1)
stop_button = tk.Button(window, text='stop',  command=lambda: sends("stop"))
stop_button.grid(row=1, column=1)
off_button = tk.Button(window, text='OFF',  command=lambda: sends("off"))
off_button.grid(row=2, column=1)
on_button = tk.Button(window, text='ON',  command=lambda: sends("on"))

Slider1 = tk.Scale(window, from_=600, to=0, orient=tk.VERTICAL, length=200)
Slider1.grid(row=0, column=2)
Slider2 = tk.Scale(window, from_=200, to=0, orient=tk.VERTICAL, length=200)
Slider2.grid(row=0, column=3)
Slider3 = tk.Scale(window, from_=200, to=0, orient=tk.VERTICAL, length=200)
Slider3.grid(row=0, column=4)
ok_button = tk.Button(
    window,  
    text='SI', 
    command=lambda: save(angles, pos, data, speed, 1)
)
ok_button.grid(row=1, column=4)
not_button = tk.Button(
    window,  
    text='NO', 
    command=lambda: save(angles, pos, data, speed, 0)
)
not_button.grid(row=2, column=4)


##==========INIT============================
datalogger("started")
connect()
print("connected")
time.sleep(1)
#schedule.every(10).seconds.do() ## ejemplos de schedule para realizar una secuencia
#schedule.every().hour.at(":45").do(sleep)

#en schedule
#orden de movimiento
#chek de posicion

## start Serial
print("startin")
connect()

## video start 
video_capture = cv2.VideoCapture(0)

#main loop
while True:
    ret, frame = video_capture.read()
    circleDetection(frame) # deteccion de circulos
    try:
        data=readSerial()
    except:
        data=data

    #oscsender()
    canvas.bind('<Button-1>', draw)
    canvas.grid(row=0, column=0)
    window.update_idletasks()
    window.update()


#break
    if cv2.waitKey(1) & 0xFF == ord('&'):
        break
# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()

