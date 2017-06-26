from tuw.Serial import *
from tuw.Figure import *
from matplotlib import pyplot as plt
import string

default_port='/dev/ttyACM0'
ser = MySerial(default_port)
def readPort(fig):
    last = MyParam()
    last.target = np.NaN
    last.steering = 0
    last.kp = np.NaN
    last.ki = np.NaN
    last.kd = np.NaN
    while True :
           
        line = ser.readline()
        try:
            if (line[0] == '$'):
                if (line[1] == 's'):
                    values = line[2:]
                    words = string.split(values,",")    # Fields split   
                    if len(words) == 3:     
                        current_pwm = float(words[0])
                        current_rps = float(words[1])  
                        current_rps_target = float(words[2])   
                        fig.set(current_pwm, current_rps, current_rps_target)
                    if(last.target != fig.param.target) :
                        last.target = fig.param.target
                        ser.writeline('t',last.target)
                    if(last.kp != fig.param.kp) :
                        last.kp = fig.param.kp
                        ser.writeline('p',last.param.kp)
                    if(last.ki != fig.param.ki) :
                        last.ki = fig.param.ki
                        ser.writeline('i',last.param.ki)
                    if(last.kd != fig.param.kd) :
                        last.kd = fig.param.kd
                        ser.writeline('d',last.kd)
                    if(last.steering != fig.param.steering) :
                        last.steering = fig.param.steering
                        ser.writeline('s',last.steering)
                else :
                    print "cmd: " + line
            else :
                print "msg: " + line
        except :
            print "no msg" + line
 
def update(fig):
    while True:
        fig.plot()
        time.sleep(0.01)   

if __name__ == '__main__':
    fig = plt.figure(FigureClass=MyFigure, figtitle='my title')
    fig.init_gui()
    t1 = threading.Thread(target=readPort,args=(fig, )) 
    t1.start()
    t2 = threading.Thread(target=update,args=(fig, )) 
    t2.start()
    plt.show()