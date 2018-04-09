import time
import numpy as np
import thread
from matplotlib import pyplot as plt
from matplotlib.figure import Figure
from matplotlib.widgets import  Button
from multiprocessing import Process

class MyFigure(Figure):
    def __init__(self, *args, **kwargs):
        figtitle = kwargs.pop('figtitle', 'hi')
        Figure.__init__(self, *args, **kwargs)
        self.text(0.5, 0.95, figtitle, ha='center')
        self.y = np.zeros(100, dtype=float)
        self.x = range(100)   
        self.subplots_adjust(left=0.25, bottom=0.25)

        
    def init_gui(self):
        self.ax_top = fig.add_subplot(111)
        self.ax_top.set_ylim([0,1])
        self.ax_btn_generate = plt.axes([0.8, 0.1, 0.1, 0.04])
        self.btn_generate = Button(self.ax_btn_generate, 'Generate')
        self.btn_generate.on_clicked(self.generate)
        
    def generate(self, event):
        self.ax_top.cla()
        self.ax_top.set_ylim([0,1])
        self.y = np.delete(self.y, 0)
        self.y = np.append(self.y, [np.random.random_sample()])
        self.ax_top.plot(self.x, self.y, 'b')
        self.canvas.draw_idle()
        
        
    def async_event(self ):
        i = 0
        while True:
            self.generate(0)
            time.sleep(0.1)
            print 'async_event: %d'%(i)
            i = i + 1
    
if __name__ == '__main__':
    fig = plt.figure(FigureClass=MyFigure, figtitle='my title')
    fig.init_gui()
    thread.start_new_thread(fig.async_event,()) 
    
    plt.show()
