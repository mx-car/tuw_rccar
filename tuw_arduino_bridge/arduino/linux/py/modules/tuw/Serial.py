import serial
import sys
import time
import struct
import numpy as np

class MsgHeader:
    def __init__(self, size = 0, type = 0, seq = 0, sec = 0, nsec = 0):
        self.size = np.uint16(size)
        self.type = np.uint16(type)
        self.seq = np.uint32(seq)
        self.sec = np.uint32(sec)
        self.nsec = np.uint32(nsec)
        self.struct = struct.Struct("HHI")    
    
      
class MySerial:
    
    def __init__(self, port , baudrate = 115200, timeout = 1.0, xonxoff = True):
        self.com = serial.Serial()
        self.com.port = port
        self.com.baudrate = baudrate
        self.com.timeout = timeout
        self.com.xonxoff = xonxoff
        self.quit = False
        
    def close(self):     
        try:
            self.com.close()
        except serial.serialutil.SerialException:
            print "Could close port: " +  self.com.port
        
    def open(self):   
        try:
            self.com.open()
            return True
        except serial.serialutil.SerialException:
            print "Could not open port: " +  self.com.port
            self.close()
            return False
        
    def readline(self):
        try:
            line = self.com.readline()
        except :
            while (self.open() == False) and (self.quit == False):
                time.sleep(2)
            if self.quit == False :
                line = self.readline()           
        return line
    
    def writeline(self, command, value):
        line = '#{:s} {:+f} \n'.format(command, value)
        try:
            self.com.writelines(line)
            print line
        except :
            while self.open() == False:
                time.sleep(2)
            self.com.writelines(line)         