## Demo program to demonstrate a serial link between a PC and an Arduino board
#!/usr/bin/env python

import time
from datetime import datetime
import argparse
import struct
import numpy as np
from tuw.Arduino import ComMessage
from tuw.ArduinoMessages import Pose
from tuw.ArduinoMessages import Text
from tuw.ArduinoMessages import Acckerman
from tuw.ArduinoMessages import Actors
from tuw.Gamepad import *

parser = argparse.ArgumentParser()
parser.add_argument('-p', dest="port", type=str, default="/dev/ttyACM0", help="port such as /dev/ttyACM0")
parser.add_argument('-b', dest="baudrate", type=int, default="115200", help="baudrate such as 115200")
parser.add_argument('-o', dest="timeout", type=float, default="3", help="timeout such as 1.0")
args = parser.parse_args()
print 'port: {:s}, baudrate {:d}, timeout {:f} '.format(args.port, args.baudrate, args.timeout)

   
if __name__ == '__main__':
    com = ComMessage(args.port, args.baudrate, args.timeout)
    loop = True    
    actors = Actors()    
    gamepad = Gamepad(0)
    while loop:
        actors.clear()
        com.receive()
        com.clear_tx()
        if( com.rx ) :
            print "rx " + str(com)
            while (com.rx and (len(com.data_rx) > 0)):
                type_msg = com.pop_type()              
                if(type_msg == ComMessage.TYPE_SYNC_REQUEST) :  
                    print ' sync request' 
                    com.push_sync()         
                elif(type_msg == Text.TYPE) :  
                    print " " + str(com.pop_object(Text()))        
                elif(type_msg == ComMessage.TYPE_ERROR) :  
                    print ' problem in message'
                    com.rx = False
                elif(type_msg == ComMessage.TYPE_EMPTY) :  
                    print ' empty'
                    com.rx = False
                else :  
                    print ' unkown type: {:d} '.format(type_msg)
                    
        gamepad.read()
        if(gamepad.button[4] > 0):
            actors.rps = gamepad.axis[1]*100
            actors.rad = gamepad.axis[3]*0.5
            com.push_object(actors)
        print "tx " + str(com) + ": " + str(actors)
        com.send()           
    print "exit"