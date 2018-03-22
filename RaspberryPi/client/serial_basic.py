import serial
# import numpy as np

ser = serial.Serial('/dev/ttyUSB0',115200) # open serial port
print(ser.name)                     # check which port was really opened

NSAMPLES = int(ser.readline());

get = [0]*NSAMPLES

for i in range(NSAMPLES):
    line = ser.readline()
    get[i] = line
    print get[i]

ser.close();
