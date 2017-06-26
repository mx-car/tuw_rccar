import threading
import time
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.figure import Figure
from matplotlib.widgets import Slider, Button
import logging

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-9s) %(message)s',)

class MyFigure(Figure):
    def __init__(self, *args, **kwargs):
        """
        custom kwarg figtitle is a figure title
        """
        figtitle = kwargs.pop('figtitle', 'hi mom')
        Figure.__init__(self, *args, **kwargs)
        self.text(0.5, 0.95, figtitle, ha='center')
        self.y = np.zeros(100, dtype=float)
        self.x = range(100)   
        self.subplots_adjust(left=0.25, bottom=0.25)
        self.lock = threading.Lock()

        
    def init_gui(self):
        self.ax_top = fig.add_subplot(111)
        self.ax_top.set_ylim([0,1])
        self.ax_btn_update = plt.axes([0.8, 0.025, 0.1, 0.04])
        self.ax_sld_value = plt.axes([0.25, 0.1, 0.65, 0.03])
        self.btn_update = Button(self.ax_btn_update, 'Update')
        self.btn_update.on_clicked(self.rand)
        self.sld_value = Slider(self.ax_sld_value, 'Value', 0., 1.0, valinit=0.5) 
        self.sld_value.on_changed(self.value)  
        
    def rand(self, event):
        self.y = np.delete(self.y, 0)
        self.y = np.append(self.y, [np.random.random_sample()])
        self.ax_top.cla()
        self.ax_top.set_ylim([0,1])
        self.ax_top.plot(self.x, self.y, 'b')
        self.canvas.draw_idle()
        
    def value(self, val):
        self.y = np.delete(self.y, 0)
        self.y = np.append(self.y, [float(val)])
        self.ax_top.cla()
        self.ax_top.set_ylim([0,1])
        self.ax_top.plot(self.x, self.y, 'b')
        self.canvas.draw_idle()
        
    def async_event(self ):
        while True:
            self.rand(0)
            time.sleep(0.1)
    
if __name__ == '__main__':
    fig = plt.figure(FigureClass=MyFigure, figtitle='my title')
    fig.init_gui()
    t = threading.Thread(target=fig.async_event,args=()) 
    t.start()
    plt.show()
