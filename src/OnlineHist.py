from PlotWindow import PlotWindow

from time_sync.msg import Event

import rospy
import sys, random
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import numpy
from std_msgs.msg import Int8




class OnlineHist(PlotWindow):
  def __init__(self):
    PlotWindow.__init__(self)

    self.window_size=200
    self.values=numpy.zeros((self.window_size))
    self.index=0

    rospy.init_node('visualizer', anonymous=True)
    self.subscriber = rospy.Subscriber("event", Event, self.plotResults, queue_size = 1 )


  def plotResults(self, data): 
    self.axes.clear()        
    self.axes.set_autoscaley_on(True)

    if self.index==self.window_size-1:
      self.index=0
    else:
      self.index=self.index+1

    device_time          = data.device_time.secs          + data.device_time.nsecs          /1.0e9 
    local_receive_time   = data.local_receive_time.secs   + data.local_receive_time.nsecs   /1.0e9 
    corrected_local_time = data.corrected_local_time.secs + data.corrected_local_time.nsecs /1.0e9 

  
    self.values[self.index]=(1e3 * (local_receive_time - corrected_local_time) )

    print self.values
    n, bins, patches = self.axes.hist(self.values, 30, (0, 1), normed=True, facecolor='green', alpha=0.75)

    output= "Data index "+str(data.local_receive_time.secs)
    min_x, max_x=self.axes.get_xlim()
    min_y, max_y=self.axes.get_ylim()   
    self.axes.text(max_x*0.6,max_y*0.7,output,horizontalalignment='left',verticalalignment='center')

    self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OnlineHist()
    window.show()
    app.exec_()



#!/usr/bin/env python
import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import matplotlib
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as  NavigationToolbar
from matplotlib.figure import Figure

class PlotWindow(QMainWindow):
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle('Sliding histogramm')
        self.create_main_frame()
        self.on_draw()

    def save_plot(self):
        pass

    def on_about(self):
        pass

    def on_pick(self, event):
        pass

    def on_draw(self):
        self.axes.clear()        
        self.axes.grid(True)
        self.canvas.draw()

    def create_main_frame(self):
        self.main_frame = QWidget()
        self.dpi = 100
        self.fig = Figure((5.0, 4.0), dpi=self.dpi)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self.main_frame)
        self.axes = self.fig.add_subplot(111)
        self.canvas.mpl_connect('pick_event', self.on_pick)
        self.mpl_toolbar = NavigationToolbar(self.canvas, self.main_frame)     
        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        vbox.addWidget(self.mpl_toolbar)
        self.main_frame.setLayout(vbox)
        self.setCentralWidget(self.main_frame)
