import threading
import time
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.figure import Figure
from matplotlib.widgets import Slider, Button

class MyParam:
    def __init__(self):
        self.steering = 0
        self.target = 0.0
        self.kp = 0.0000
        self.ki = 0.0001
        self.kd = 0.00
        self.div = 100000.
    
    
class MyFigure(Figure):
    def __init__(self, *args, **kwargs):
        """
        custom kwarg figtitle is a figure title
        """
        figtitle = kwargs.pop('figtitle', 'hi mom')
        Figure.__init__(self, *args, **kwargs)
        self.text(0.5, 0.95, figtitle, ha='center')
        self.history_pwm = np.zeros(100, dtype=float)
        self.history_rps = np.zeros(100, dtype=float)
        self.history_rps_target = np.zeros(100, dtype=float)
        self.t = range(100)   
        self.subplots_adjust(left=0.25, bottom=0.35)
        self.lock = threading.Lock()
        self.param = MyParam()

        
    def init_gui(self):
        self.ax_pwm = self.add_subplot(211)
        self.ax_rps = self.add_subplot(212)
        self.ax_btn_stop = plt.axes([0.1, 0.15, 0.1, 0.04])
        # self.ax_btn_hold = plt.axes([0.1, 0.15, 0.1, 0.04])
        self.ax_sld_rps = plt.axes([0.25, 0.25, 0.65, 0.03])
        self.ax_sld_kp  = plt.axes([0.25, 0.20, 0.65, 0.03])
        self.ax_sld_ki  = plt.axes([0.25, 0.15, 0.65, 0.03])
        self.ax_sld_kd  = plt.axes([0.25, 0.10, 0.65, 0.03])
        self.ax_sld_steering  = plt.axes([0.25, 0.05, 0.65, 0.03])
        self.btn_stop = Button(self.ax_btn_stop, 'Stop')
        # self.btn_hold = Button(self.ax_btn_hold, 'Hold')
        self.btn_stop.on_clicked(self.rand)
        # self.btn_hold.on_clicked(self.rand)
        self.sld_rps = Slider(self.ax_sld_rps, 'rps', -250., 250.0, valinit=self.param.target) 
        self.sld_kp = Slider(self.ax_sld_kp, 'kp', 0., 1000., valinit=self.param.kp*self.param.div) 
        self.sld_ki = Slider(self.ax_sld_ki, 'ki', 0., 1000., valinit=self.param.ki*self.param.div) 
        self.sld_kd = Slider(self.ax_sld_kd, 'kd', 0., 1000., valinit=self.param.kd*self.param.div) 
        self.sld_steering = Slider(self.ax_sld_steering, 'steering', -20., 20., valinit=0) 
        self.sld_rps.on_changed(self.set_rps)  
        self.sld_kp.on_changed(self.set_kp)  
        self.sld_ki.on_changed(self.set_ki)  
        self.sld_kd.on_changed(self.set_kd)  
        self.sld_steering.on_changed(self.set_steering)  
        
    def rand(self, event):
        self.history_pwm = np.append(np.delete(self.history_pwm, 0), [np.random.random_sample()])
        self.ax_pwm.cla()
        self.ax_pwm.set_ylim([0,1])
        self.ax_rps.set_ylim([0,100])
        self.ax_pwm.plot(self.t, self.history_pwm, 'b')
        self.canvas.draw_idle()
        
    def set_rps(self, value):
        self.param.target = value
        return 0
    
    def set_kp(self, value):
        self.param.kp = value/self.div
        return 0
    
    def set_ki(self, value):
        self.param.ki = value/self.div
        return 0
    
    def set_kd(self, value):
        self.param.kd = value/self.div
        return 0
    
    def set_steering(self, value):
        self.param.steering = value
        return 0
    
    def set(self, pwm, rps, rps_target):
        self.history_pwm = np.append(np.delete(self.history_pwm, 0), [float(pwm)])
        self.history_rps = np.append(np.delete(self.history_rps, 0), [float(rps)])
        self.history_rps_target = np.append(np.delete(self.history_rps_target, 0), [float(rps_target)])
        
        
    def async_event(self ):
        while True:
            self.rand(0)
            time.sleep(0.1)
    
    def plot(self ):
        self.ax_pwm.cla()
        self.ax_rps.cla()
        self.ax_pwm.set_ylim([-1.1,1.1])
        self.ax_pwm.grid(True)
        self.ax_rps.set_ylim([-250,250])
        self.ax_rps.grid(True)
        ln_pwm,        = self.ax_pwm.plot(self.t, self.history_pwm, 'b')
        ln_rps,        = self.ax_rps.plot(self.t, self.history_rps, 'b')
        ln_rps_target, = self.ax_rps.plot(self.t, self.history_rps_target, 'g')
        self.canvas.draw()
