
## sudo python3 install schedule
## sudo pip3 install --upgrade google-api-python-client
##sudo pip install --upgrade google-cloud-speech
##sudo pip3 install google-speech
##sudo apt-get install python-pyaudio
## sudo apt-get install python wave o sudo pip3 install wave
## sudo install python3-pydub o sudo pip3 install pydub
## sudo pip3 install unidecode (o por synaptics)

## si algunas cosas no funcionan instaladas desde consola se puede revisar si estan desde synaptics

import unicodedata
#import schedule
import time
import os
#import subprocess
#import unidecode
import datetime
import numpy as np
import cv2
from pythonosc.udp_client import SimpleUDPClient

from funciones import testserial, datalogger, connect, circleDetection, readSerial, moveToPos, moveToAngle 

#datalogger("started")
#connect()
#print("connected")
#time.sleep(2)


#schedule.every(10).seconds.do() ## ejemplos de schedule para realizar una secuencia
#schedule.every().hour.at(":45").do(sleep)

#en schedule
#orden de movimiento
#chek de posicion


#============================================
## start Serial
connect()

## video start 
video_capture = cv2.VideoCapture(0)

#main loop
while True:
    ret, frame = video_capture.read()
    circleDetection(frame) # deteccion de circulos
    dataClass(readSerial())



#break
    if cv2.waitKey(1) & 0xFF == ord('&'):
        break
# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()

