#!/usr/bin/env python

import time
import serial
from datetime import datetime
import argparse
import struct
import numpy as np
from tuw.Arduino import ComMessage
from tuw.Arduino import Pose

parser = argparse.ArgumentParser()
parser.add_argument('-p', dest="port", type=str, default="/dev/ttyACM0", help="port such as /dev/ttyACM0")
parser.add_argument('-b', dest="baudrate", type=int, default="115200", help="baudrate such as 115200")
parser.add_argument('-o', dest="timeout", type=float, default="1", help="timeout such as 1.0")
args = parser.parse_args()
print 'port: {:s}, baudrate {:d}, timeout {:f} '.format(args.port, args.baudrate, args.timeout)

   
if __name__ == '__main__':
    com = ComMessage(args.port, args.baudrate, args.timeout)
    loop = True    
    while loop:
        com.receive()
        if( com.rx ) :
            if(com.type == Pose.TYPE) :
                p = Pose()
                p.unpack(com.data)
                print 'rx {:d} :{:f}, {:f}, {:f}'.format(com.seq,p.x,p.y,p.theta)
                p.y = p.y + 1
                com.send(p.pack())
                                
        time.sleep(0.001)           
    print "exit"