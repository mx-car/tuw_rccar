from tuw.Gamepad import *
import time 

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('-p', dest="port", type=str, default="/dev/ttyACM0", help="port such as /dev/ttyACM0")
parser.add_argument('-b', dest="baudrate", type=int, default="115200", help="baudrate such as 115200")
parser.add_argument('-o', dest="timeout", type=float, default="1", help="timeout such as 1.0")
args = parser.parse_args()
print 'port: {:s}, baudrate {:d}, timeout {:f} '.format(args.port, args.baudrate, args.timeout)


if __name__ == '__main__':
    com = serial.Serial()
    com.port = args.port
    com.baudrate = args.baudrate
    com.timeout = args.timeout
    com.xonxoff = True
    com.open()
    gamepad = Gamepad(0)
    while not gamepad.done :
        gamepad.read()
        print "axis: " + str(gamepad.axis)
        print "hat: " + str(gamepad.hat)
        print "button: " + str(gamepad.button)
        time.sleep(0.5)
        