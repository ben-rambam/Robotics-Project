import rospy
from math import *
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
# define our robot dimensions and speed
r = 0.1
L = 0.2
v = 1.4
i = 0

Npts = 24
t = np.linspace(0, 2.0*pi,Npts)
xlocs = 10* np.sin(t) + 12
ylocs = 10* np.sin(2*t) + 12
pub = rospy.Publisher('robot0/kinematic_params', Float64MultiArray, queue_size=10)
msgPub = rospy.Publisher('message', String, queue_size=10)

def poseCallback(data):
    global i
    global r
    global L
    global v
    global Npts
    global xlocs
    global ylocs
    global pub
    global msgPub

    #datavec = data.data.split(":")
    robotx = data.x
    roboty = data.y
    robottheta = data.theta
    print(robottheta)
# grab the current target point
    xtarget = xlocs[i];
    ytarget = ylocs[i];
# calculate error
    thetatarget = atan2(ytarget-roboty,xtarget-robotx)
    err = -robottheta + thetatarget
# convert err to a value between +/-180
    while err > pi:
        err = err - 2*pi
    while err < -pi:
        err = err + 2*pi
# implement the controller
    vMax = 3.0
    minTurnRad = 0.1
    kp = vMax/pi/minTurnRad
    v = (pi-abs(err))/pi * vMax
    omega = kp*err;
# determine wheel speeds
    w1 = 1/r*(v+L*omega)
    w2 = 1/r*(v-L*omega)
# publish wheel speeds
    data = Float64MultiArray(data=[])
    data.layout = MultiArrayLayout()
    data.layout.dim = [MultiArrayDimension()]
    data.layout.dim[0].label = "Parameters"
    data.layout.dim[0].size = 4
    data.layout.dim[0].stride = 1
    data.data = [w1,w2,r,L]
    pub.publish(data)
# calculate distance to target
    targetDist = sqrt( (robotx-xtarget)**2 + (roboty-ytarget)**2 )
# if within 0.2 update target
    if targetDist < 0.2:
        i = (i+1)%(Npts)
        if i == 0:
            msgPub.publish("stop")
        #print(xlocs[0])
        #print(ylocs[0])

def myMain():
    global i
    global data
    #print("about to make sub")
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("robot0/pose2D", Pose2D, poseCallback)

    data = Float64MultiArray(data=[])
    data.layout = MultiArrayLayout()
    data.layout.dim = [MultiArrayDimension()]
    data.layout.dim[0].label = "Parameters"
    data.layout.dim[0].size = 4
    data.layout.dim[0].stride = 1
    #print(xlocs[0])
    #print(ylocs[0])
    i = 0
    print("finished main")
    while True:
        pass

if __name__ == '__main__':
        try:
            myMain()
        except rospy.ROSInterruptException:
            pass
