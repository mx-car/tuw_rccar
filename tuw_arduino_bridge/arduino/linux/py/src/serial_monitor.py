#!/usr/bin/env python

import time
import serial
from datetime import datetime
import argparse
import struct
import numpy as np
from serial import SerialException

parser = argparse.ArgumentParser()
parser.add_argument('-p', dest="port", type=str, default="/dev/ttyACM0", help="port such as /dev/ttyACM0")
parser.add_argument('-b', dest="baudrate", type=int, default="115200", help="baudrate such as 115200")
parser.add_argument('-o', dest="timeout", type=float, default="1", help="timeout such as 1.0")
parser.add_argument('-t', dest="msg_type", type=int, default=1, help="message type to capture")
parser.add_argument('-s', dest="msg_struct", type=str, default="fff", help="struct to expect if non string is expected")
args = parser.parse_args()
print 'port: {:s}, baudrate {:d}, timeout {:f} '.format(args.port, args.baudrate, args.timeout)
if args.msg_struct is not None:
    print 'the following struct is expected: {:s}, size of {:d} bytes with the type id {:d} '.format(args.msg_struct, struct.Struct(args.msg_struct).size, args.msg_type)
else : 
    print 'strings are expected'


class MsgHeader:
    def __init__(self, size = 0, type = 0, seq = 0, sec = 0, nsec = 0):
        self.rx = False
        self.max_data_size = 0xFF
        self.size = np.uint16(size)
        self.type = np.uint16(type)
        self.seq = np.uint32(seq)
        self.sec = np.uint32(sec)
        self.nsec = np.uint32(nsec)
        self.data = []
        self.struct = struct.Struct("HHIII")

    def unpack(self, msg):
        self.size, self.type, self.seq, self.sec, self.nsec, = self.struct.unpack(msg)
        
    def pack(self):
        msg = self.struct.pack(len(self.data), self.type, self.seq, self.sec, self.nsec)
        return msg
        
    def size_header(self):
        return self.struct.size
    
        
def com_close():     
    """ open serial interface   """
    try:
        com.close()
    except serial.serialutil.SerialException:
        print "Could close port: " +  com.port  

def com_open():   
    """ close serial interface   """
    try:
        com.open()
        return True
    except (serial.serialutil.SerialException, serial.SerialTimeoutException) as e:
        print e
        com_close()
        return False

def com_readline():
    """ read serial interface and opens/reconnects if needed """
    try:
        line = com.readline()
    except (serial.serialutil.SerialException, serial.SerialTimeoutException) as e:
        print e
        while (com_open() == False) :
            time.sleep(2)
        line = com_readline()           
    return line
        
def com_read(header):
    """ read serial interface and opens/reconnects if needed """
    try:
        header.rx = False
        msg = com.read(header.size_header())
        if(len(msg) != header.size_header()) :
            print "incorrect header received"
            com.reset_input_buffer()
            return header
        header.unpack(msg)
        if(header.size >= header.max_data_size):
            print "incorrect header received, data size = " + str(header.size)
            com.reset_input_buffer()
            return header            
        header.data = com.read(header.size)
        if(len(header.data) != header.size) :
            print "incorrect data received"
            com.reset_input_buffer()
            return header
        header.rx = True
    except struct.error as e:
        print "struct.error"        
        print e        
    except serial.SerialTimeoutException as e:
        print "serial.SerialTimeoutException = " + e
    except serial.serialutil.SerialException as e:
        print "serial.serialutil.SerialException = " + e
    return header


def com_send(header):
    """ read serial interface and opens/reconnects if needed """
    try:
        com.write(header.pack())
        com.write(header.data)
    except serial.SerialTimeoutException as e:
        print "serial.SerialTimeoutException = " + e
    except serial.serialutil.SerialException as e:
        print "serial.serialutil.SerialException = " + e

    
if __name__ == '__main__':
    com = serial.Serial()
    com.port = args.port
    com.baudrate = args.baudrate
    com.timeout = args.timeout
    #com.xonxoff = True
    com.open()
    loop = True
    while loop :       
        if args.msg_struct is not None:
            header = MsgHeader()
            pose = struct.Struct(args.msg_struct)
            try:            
                header = com_read(header)
                if(header.rx == True) : 
                    if (header.type == args.msg_type) and (len(header.data) == pose.size):
                        try:
                            x,y,theata, = pose.unpack(header.data)
                        except struct.error as e:
                            print "error: unpack data = " + e   
                        print 'rx {:d} :{:f}, {:f}, {:f}'.format(header.seq,x,y,theata)
                        try:
                            header.data = pose.pack(x,x,theata)
                        except struct.error as e:
                            print "error: pack data = " + e   
                        com_send(header)
                        # x,y,theata, = pose.unpack(header.data)
                        # print 'tx {:d} :{:f}, {:f}, {:f}'.format(header.seq,x,y,theata)
                    
            except :
                print 'rx error struct ' + str(header.seq)
        else:    
            try:            
                line = com_readline()
                print '{:s}: {:s}'.format(str(datetime.now()), line.rstrip('\n'))
            except :
                print 'rx error'
        time.sleep(0.001)
    print "exit"