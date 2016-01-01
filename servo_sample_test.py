from time import sleep
import serial
ser = serial.Serial('/dev/ttyACM0', 115200,timeout =0.1) # Establish the connection on a specific port
ser.flush()
rtn = 0
angl = 230;
while angl > 50:
     
     
     ##turn the base
     ser.write('g') 
     ser.write(chr(angl))
     while rtn ==0:
        rtn = ser.readline() # Read the newest output from the Arduino
        print 'waiting'
     print 'base done:',rtn
     rtn =0
     
     ###turn the head
     ser.write('h') 
     ser.write(chr(angl))
     while rtn ==0:
        rtn = ser.readline() # Read the newest output from the Arduino
        print 'waiting'
     print 'head done:',rtn
     rtn =0
     
     angl = angl-5
     sleep(1) # Delay for one tenth of a second
     
    
