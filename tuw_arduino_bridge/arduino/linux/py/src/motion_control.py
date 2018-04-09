from tuw.Serial import *
from tuw.Figure import *
from matplotlib import pyplot as plt
import string
import struct
from _dbus_bindings import Struct

default_port='/dev/ttyACM0'
ser = MySerial(default_port)

def readPort():
    while True :           
        try:            
            line = ser.readline()
        except :
            print "tx"
        pose = Struct('@hhh')
        if(len(line) == pose.size) :
            x, y, theta, = pose.unpack(line)
            print x
        else:
            print 'bytes receive {:d}, expected {:d} {:s}'.format(len(line), pose.size, line.rstrip('\n'))
        #print line
 
if __name__ == '__main__':
    readPort()