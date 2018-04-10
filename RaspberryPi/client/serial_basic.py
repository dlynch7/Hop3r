import serial
import time

timeout = 10
timeoutOccurred = 0

ser = serial.Serial(port='/dev/ttyUSB0',baudrate=115200,timeout=timeout) # open serial port
print(ser.name)                     # check which port was really opened

now = time.time()
NSAMPLES = ser.readline();
deltat = time.time() - now;

if deltat < timeout:
    NSAMPLES = int(NSAMPLES);
    print "Expecting %d samples." % NSAMPLES
    get = [0]*NSAMPLES # create array to store received data
    received = 0;

    ser.write('1'); # '1' gives the other device permission to write

    for i in range(NSAMPLES):
        now = time.time()

        line = ser.readline()
        get[i] = line
        print get[i]

        deltat = time.time() - now;

        if deltat < timeout:
            received += 1
        else:
            timeoutOccurred = 1
            print "Breaking out of read loop..."
            break

    if timeoutOccurred:
        print "Serial read timeout occurred. Expected %d samples, received %d samples." % (NSAMPLES, received)
    else:
        if received < NSAMPLES:
            print "Serial read complete, but number of received samples does not match expected number of samples."
            print "Expected %d samples, received %d samples." % (NSAMPLES, received)
        else:
            print "Serial read complete. Expected %d samples, received %d samples." % (NSAMPLES, received)

else:
    timeoutOccurred = 1
    print "Serial read timeout occurred before the number of samples was sent. Check connection."


ser.close()
