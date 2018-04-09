## @package ArduinoMessages
#  defines messages for serial communication


import struct
import copy
    
## class to receive and send via serial link
class Pose:
    
    ## unique type id to identify received objects
    TYPE = 100
    struct = struct.Struct("=fff")
    
    
    ## constructor
    def __init__(self, x = 0., y = 0., theta = 0.):
        self.x = x
        self.y = y
        self.theta = theta

    ## de-serialized a received string into the class variables
    def unpack(self, msg):
        self.x, self.y, self.theta,  = self.struct.unpack(msg)
        
    ## serializes the class variables into a buffer 
    # @return serialized buffer
    def pack(self):
        msg = self.struct.pack(self.x, self.y, self.theta)
        return msg
    
    def __str__(self):
        return '{:f}, {:f}, {:f}'.format(self.x, self.y, self.theta)
        

           
## class to receive and send via serial link
class Actors:
    
    ## unique type id to identify received objects
    TYPE = 1000
    struct = struct.Struct("=ff")
    
    
    ## constructor
    def __init__(self, rps = 0., rad = 0.):
        self.rps = rps
        self.rad = rad

    ## de-serialized a received string into the class variables
    def unpack(self, msg):
        self.rps, self.rad,  = self.struct.unpack(msg)
        
    ## serializes the class variables into a buffer 
    # @return serialized buffer
    def pack(self):
        msg = self.struct.pack(self.rps, self.rad)
        return msg
    
    def __str__(self):
        return '{:f}, {:f}'.format(self.rps, self.rad)
    
    def clear(self):
        self.rps = 0
        self.rad = 0
        
## class to receive and send via serial link
class Acckerman:
    
    ## unique type id to identify received objects
    TYPE = 1001
    struct = struct.Struct("=ff")
    
    
    ## constructor
    def __init__(self, v = 0., alpha = 0.):
        self.v = v
        self.alpha = alpha

    ## de-serialized a received string into the class variables
    def unpack(self, msg):
        self.v, self.alpha,  = self.struct.unpack(msg)
        
    ## serializes the class variables into a buffer 
    # @return serialized buffer
    def pack(self):
        msg = self.struct.pack(self.v, self.alpha)
        return msg
    
    def __str__(self):
        return '{:f}, {:f}'.format(self.v, self.alpha)
        
## class to receive and send via serial link
class Text:
    
    ## unique type id to identify received objects
    TYPE = 200
    struct = struct.Struct("=32s")
    
    
    ## constructor
    def __init__(self, text = ''):
        self.text = copy.copy(text)

    ## de-serialized a received string into the class variables
    def unpack(self, msg):
        self.text = copy.copy(msg)
        
    ## serializes the class variables into a buffer 
    # @return serialized buffer
    def pack(self):
        return copy.copy(self.text) 
    def __str__(self):
        return self.text[0: self.text.find('\0')]
        