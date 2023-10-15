
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
import schedule
from arduino import move, testserial, shut, target, sleep, wakeup, led, datalogger, connect
import time

##================================================================
#NOTAS
#EL CODIGO DE ARDUINO NO DEBE ARRANCAR CON UN ATTACH, SINO QUE DEBE ESPERAR AL CODIGO DE PYTHON
#REVISAR TAMBIEN LAS FUNCIONES Y SU COINCIDENCIA CON EL CoDIGO

datalogger("started")
connect()
print("connected")
time.sleep(2)
wakeup()
time.sleep(2)
target()


schedule.every(10).seconds.do(target)
#schedule.every().hour.at(":45").do(sleep)


while True:
    try:
        schedule.run_pending()
        move()
        nen = testserial()
        if(nen==True):
                ## testear esto!!
                shut()
        time.sleep(0.01)
       # print("running")

    except KeyboardInterrupt:
        datalogger("ERROR GENERAL")
