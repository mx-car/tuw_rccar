## @package Arduino
#  module to simplify the serial communcation with an arduino using structs


import serial
import time
import struct
import numpy as np
import datetime as dt
    
## class to handle time stamps in a ROS fashion (www.ros.org)
class Time:
    ## constructor
    def __init__(self, sec = 0, nsec = 0):
        ## timestamp secound
        self.sec = np.int32(sec)
        ## timestamp nanosecound
        self.nsec = np.int32(nsec)
    
    ## sets the timestamp to now
    def set_to_now(self):
        t = time.time();
        self.sec = int(t)
        self.nsec = int( (t - self.sec) * 1000 * 1000 * 1000)
        
    ## returns the time stamp a datetime object
    def time(self):
        t = dt.datetime.fromtimestamp(self.sec)
        t = t + dt.timedelta( microseconds = self.nsec/1000)
        return t
        
    def __str__(self):
        return self.time().strftime('%Y-%m-%d %H:%M:%S.%f')

    
## header class to handle message for serial communication
class ComHeader:
    
    ## unique type id to identify received objects
    struct_header = struct.Struct("=HIii")
    
    ## unique type id to identify received objects
    TYPE_ERROR   = 0
    TYPE_EMPTY   = 1
    TYPE_SYNC_REQUEST = 10
    TYPE_SYNC = 11
    
    ## constructor
    def __init__(self, size = 0, seq = 0, sec = 0, nsec = 0):
        ## set on true on successful received messages
        self.rx = False
        ## maximum object size to receive
        self.max_data_size = 0xFF
        ## total size of message
        self.size_rx = np.uint16(size)
        ## total size of message
        self.size_tx = np.uint16(size)
        ## unique message id
        self.seq = np.uint32(seq)
        ## timestamp 
        self.stamp = Time(sec, nsec)
        ## serialized data
        self.data_rx = []
        ## serialized data
        self.data_tx = []

    ## de-serialized a received string into the class variables
    def unpack(self, msg):
        self.size_rx, self.seq, self.stamp.sec, self.stamp.nsec, = self.struct_header.unpack(msg)
        
    ## serializes the class variables into a buffer 
    # @return serialized buffer
    def pack(self):
        msg = self.struct_header.pack(len(self.data_tx), self.seq, self.stamp.sec, self.stamp.nsec)
        return msg
              
    ## overload str fnc
    def __str__(self):
        return 'seq={:04d}, size={:d}, stamp={:s}'.format(self.seq, self.size_rx, str(self.stamp) )
     
## header class send messages via serial link
class ComMessage(ComHeader):
    
    struct_type = struct.Struct("=Hh")
    
    ## constructor
    def __init__(self, port , baudrate = 115200, timeout = 1.0, xonxoff = False):
        ComHeader.__init__(self)
        self.com = serial.Serial()
        self.com.port = port
        self.com.baudrate = baudrate
        self.com.timeout = timeout
        self.com.xonxoff = xonxoff
        self.quit = False
        self.count_msg = 0   
        
        
    ## closes serial link
    def close(self):     
        try:
            self.com.close()
        except serial.serialutil.SerialException:
            print "Could close port: " +  self.com.port
        
    ## opens serial link
    def open(self):   
        try:
            self.com.open()
            self.com.flushInput()
            self.com.flushOutput()
            return True
        except serial.serialutil.SerialException:
            print "Could not open port: " +  self.com.port
            self.close()
            return False
    
    ## deserialize allways the frist object type after the header
    ## @remark for simple messages with one object
    ## @pre receive()
    ## @return the object type 
    def get_object_header(self):
        if  (len(self.data_rx) >= self.struct_type.size):
            msg = self.data_rx[0 : self.struct_type.size]
            type, size, = self.struct_type.unpack(msg )
            return type, size
        else :
            return ComHeader.TYPE_ERROR, 0
        
    ## deserialize allways the frist object after the header and object type
    ## @remark for simple messages with one object
    ## @pre get_type()
    ## @return object
    def get_object(self, object):
        object_size = object.struct.size + self.struct_type.size
        if  (len(self.data_rx) >= object_size):
            msg = self.data_rx[self.struct_type.size : ]
            object.unpack(msg)
        return object
    
    ## pushes an object for sending into the message buffer as first object
    ## @remark for simple messages with one object
    ## @post send()
    def set_object(self, object):
        self.data_tx = self.struct_type.pack(object.TYPE, object.struct.size ) + object.pack()
        self.size_tx = len(self.data_tx)
        return self
        
    ## deserialize and pop's the object type from the buffer
    ## @return object type
    def pop_object_header(self):
        if  (len(self.data_rx) >= self.struct_type.size):
            type, size, = self.struct_type.unpack(self.data_rx[0:self.struct_type.size] )
            self.data_rx = self.data_rx[self.struct_type.size:]
            return type, size
        elif (len(self.data_rx) == 0):
            return ComHeader.TYPE_EMPTY, -1
        else :
            return ComHeader.TYPE_ERROR, -1
    
    ## serializes a object into the message buffer
    ## @post send()
    def push_object(self, object):
        objet_header_tx = self.struct_type.pack(object.TYPE, object.struct.size)
        self.data_tx = self.data_tx + objet_header_tx + object.pack()
        self.size_tx = len(self.data_tx)
        
    ## deserialize and pop's a object from the buffer
    ## @return the object
    def pop_object(self, object):
        if  (len(self.data_rx) >= object.struct.size):
            msg = self.data_rx[0:object.struct.size]
            self.data_rx = self.data_rx[object.struct.size:]
            object.unpack(msg)
        return object
        
    ## places a sync request into the message buffer
    ## @post send()
    def push_sync(self):
        self.stamp.set_to_now()
        self.data_tx = self.data_tx + self.struct_type.pack(self.TYPE_SYNC, 0)
        self.size_tx = len(self.data_tx)
    
    ## clears the message buffer
    def clear_rx(self):
        self.size_rx = 0
        self.data_rx = ''
        
    ## clears the message buffer
    def clear_tx(self):
        self.data_tx = ''
        
    def ready(self):
        return (self.com.inWaiting() >= self.struct_header.size)
     
    ## receives message
    # @see check the rx variable to see if the receiving was successful 
    def receive(self, blocking = False):            
        """ read serial interface and opens/reconnects if needed """
        try:
            self.rx = False
            if(self.com.isOpen() == False):
                self.open()
            msg = self.com.read(self.struct_header.size)
            if(len(msg) != self.struct_header.size) :
                if(len(msg) == 0):
                    print "timeout"
                self.com.flushInput()
                return self.rx
            self.unpack(msg)
            if(self.size_rx >= self.max_data_size):
                print "incorrect header received, data size = " + str(self.size_rx)
                self.com.flushInput()
                return self.rx
            self.data_rx = self.com.read(self.size_rx)
            if(len(self.data_rx) != self.size_rx) :
                print "incorrect data received"
                self.com.flushInput()
                return self.rx
            self.rx = True
        except struct.error as e:
            print "struct.error " + e    
        except serial.SerialTimeoutException as e:
            print "serial.SerialTimeoutException " + e
        except serial.serialutil.SerialException as e:
            print "serial.serialutil.SerialException " + e
        return self.rx
    
    ## sends message
    # @param msg serialized object
    def send(self):
        """ read serial interface and opens/reconnects if needed """
        try:
            self.seq = self.count_msg
            self.com.write(self.pack())
            if(len(self.data_tx) > 0):
                self.com.write(self.data_tx)
            self.count_msg = self.count_msg + 1
        except serial.SerialTimeoutException as e:
            print "serial.SerialTimeoutException = " + e
        except serial.serialutil.SerialException as e:
            print "serial.serialutil.SerialException = " + e
            
            