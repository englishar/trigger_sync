 #!/usr/bin/env python


from trigger_sync.msg import Event
import rospy
import sys, random
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import numpy
from std_msgs.msg import Int8
from threading import Lock
import matplotlib.pyplot as plt

ub_points = []
ub_points_matched = []
ub_times  = []
ub_times_matched = []
lb_points = []
lb_times = []
corrected_points = []
corrected_points_times = []
first_time = True
device_clock_id=sys.argv[1]
local_clock_id=sys.argv[2]
init_time = 0
num_points = 500

first = False

mutex = Lock()

def draw_window():
    mutex.acquire()
    plt.clf()
    ax = plt.subplot(1,1,1)
    ax.plot(ub_times, ub_points, label = "Upper Bound"  )
    ax.plot(lb_times, lb_points, label = "Lower Bound"  )
    ax.plot(corrected_points_times, corrected_points, label="Estimate")
    ax.plot(ub_times_matched, ub_points_matched, label="Matched Upper Bound")


    p_ub = plt.Rectangle((0, 1), 1, 10, fc="b")
    p_lb = plt.Rectangle((0, 0), 1, 1, fc="g")
    p_c  = plt.Rectangle((0, 0), 1, 1, fc="r")
    p_m  = plt.Rectangle((0, 0), 1, 1, fc="c")

    plt.legend([p_ub , p_lb, p_c, p_m ], ["Upper Bound","Lower Bound" , "Estimate","Mattched Upper Bound"],2)

    #l = plt.legend(bbox_to_anchor=(0, 0, 1, 1), bbox_transform=gcf().transFigure)
    #plt.legend([p_ub, p_lb, p_c], ["Upper Bound", "Lower Bound", "Estimate"])
    global device_clock_id, local_clock_id
    title = "Clock Mapping from %s to %s" % (device_clock_id, local_clock_id)
    plt.title( title)
    plt.ylabel('Offset (ms)')
    plt.xlabel('Device Time (s)')

    plt.draw()
    mutex.release()

def callback(data):
    #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.device_time.secs)

    global first, init_time, num_points
    if data.device_clock_id != device_clock_id or data.local_clock_id != local_clock_id:
        return
 
    device_time          = data.device_time.secs          + data.device_time.nsecs          /1.0e9 
    local_receive_time   = data.local_receive_time.secs   + data.local_receive_time.nsecs   /1.0e9 
    local_request_time   = data.local_request_time.secs   + data.local_request_time.nsecs   /1.0e9
    corrected_local_time = data.corrected_local_time.secs + data.corrected_local_time.nsecs /1.0e9 

    if device_time == 0:
        return

    if local_receive_time == 0:
        return

    if corrected_local_time == 0:
        return


    mutex.acquire()

    if first == False:
        first=True
        init_time = device_time

    ub_points.append(1e3 * (local_receive_time - device_time) )
    ub_times.append(device_time) # - init_time)

    corrected_points.append(1e3 * (corrected_local_time - device_time))
    corrected_points_times.append(device_time) # - init_time)

    if local_request_time != 0:
        lb_points.append(1e3 * (local_request_time - device_time) )
        lb_times.append(device_time) # - init_time)

    if len(ub_points)==num_points:
        ub_points.pop(0)
        ub_times.pop(0)
    if len(lb_points)==num_points:
        lb_points.pop(0)
        lb_times.pop(0)
    if len(corrected_points)==num_points:
        corrected_points.pop(0)
        corrected_points_times.pop(0)

    mutex.release()
    
def callback_matched(data):
    if data.device_clock_id != device_clock_id or data.local_clock_id != local_clock_id:
        return    

    device_time          = data.device_time.secs          + data.device_time.nsecs          /1.0e9 
    local_receive_time   = data.local_receive_time.secs   + data.local_receive_time.nsecs   /1.0e9 
    local_request_time   = data.local_request_time.secs   + data.local_request_time.nsecs   /1.0e9
    corrected_local_time = data.corrected_local_time.secs + data.corrected_local_time.nsecs /1.0e9 

    if device_time == 0:
        return

    if local_receive_time == 0:
        return

    if corrected_local_time == 0:
        return


    mutex.acquire()

 

    ub_points_matched.append(1e3 * (local_receive_time - device_time) )
    ub_times_matched.append(device_time) # - init_time)

  
    if len(ub_points_matched)==num_points:
        ub_points_matched.pop(0)
        ub_times_matched.pop(0)

    mutex.release()



if __name__ == '__main__':
    rospy.init_node('plotClockMapping', anonymous=True)

    rospy.Subscriber("event", Event, callback, queue_size=1)
    rospy.Subscriber("event_matched", Event, callback_matched, queue_size=1)


    #plt.autoscale(enable=True, axis='x', tight=True)
    plt.autoscale(enable=True, axis='both', tight=True)

    plt.show(block=False)

    r = rospy.Rate(5) # hz
    while not rospy.is_shutdown():
        draw_window()
        r.sleep()
