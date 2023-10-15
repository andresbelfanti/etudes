import os
import subprocess
import schedule
os.environ["GOOGLE_APPLICATION_CREDENTIALS"]="credencial_print.json"
import pyaudio
import wave
import random
import unidecode
from pydub import AudioSegment
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import datetime
import time
def datalogger(event):
    now = datetime.datetime.now()
    print (now.day, now.month, now.year, now.hour, now.minute, now.second)

    tlog=str(now.day)+str(now.month)+".txt"
    with open(tlog, "a") as myfile:
        myfile.write(str(now.day)+" "+str(now.month)+" "+str(now.year)+" "+str(now.hour)+" " + str(now.minute)+" "+str(now.second)+" " +"\n")
        myfile.write(str(event)+ "\n")
        myfile.close()


##API GOOGLE===========================================
def transcribe_file(speech_file):
    client = speech.SpeechClient()
    with open(speech_file, 'rb') as audio_file:
        content = audio_file.read()
    audio = types.RecognitionAudio(content=content)
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.FLAC, ##CODIFICACION FLAC
        language_code='es-AR')
    try:
        response = client.recognize(config, audio)
    except:
        print("no internet")

    for result in response.results:

        try:
            if "locura" in u'{}'.format(result.alternatives[0].transcript):
                subprocess.call("shutdown now" ,shell=True)
            else:
                return(u'{}'.format(result.alternatives[0].transcript))
        except:
            print("no data")



##FUNCION SAVE TXT======================================
def saveTxt (strings):
    try:
        with open('/home/pi/printpong/texto.txt', 'a') as file:
            strings=unidecode.unidecode(strings)
            print(strings)
            file.write(strings+str("\n"))
        file.close()
        print("texto guardado")
    except:
        print("nofile to append")

##FUNCION READ TXT======================================
def readTxt():

    try:
        with open('texto.txt', 'r') as file:#ACOMODAR ESTO DE NUEVO
            data = file.read().strip().split('\n')
            select=data[random.randint(0, len(data)-1)]
            print(select)
            subprocess.call(str("espeak -ves "+'"'+ str(select)+'"'+" --stdout | aplay  -D 'sysdefault:CARD=Device'") ,shell=True)
    except:
        print("no txt file or not lines")
        escritura()
## escritura=============================================
def escritura():
    subprocess.call(str("espeak -ves "+'"'+ "grabaaando"+'"'+" --stdout | aplay  -D 'sysdefault:CARD=Device'") ,shell=True)
    time.sleep(2)
    os.system("pkill espeak")
#    schedule.clear
    record()
    try:
        saveTxt(transcribe_file("output.flac"))
    except:
        print("no data")


##RECORD AUDIO==========================================
def record():
    CHUNK = 4096
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 44100
    RECORD_SECONDS =10
    WAVE_OUTPUT_FILENAME = "output.wav"

    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                        input_device_index = 1,
                    input=True,
                    frames_per_buffer=CHUNK)

    print("* recording")
    frames = []
    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)
    print("* done recording")
    stream.stop_stream()
    stream.close()
    p.terminate()
    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()
    print("convirtiendo a FLAC")
    try:
        song = AudioSegment.from_wav("output.wav")
        song.export("output.flac",format = "flac")
    except:
        print("no wav?")
