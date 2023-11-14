import serial
ser = serial.Serial('/dev/ttyUSB0')  # open serial port
print(ser.name)         # check which port was really used
ser.close()             # close port
