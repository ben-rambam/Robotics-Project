#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from math import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Ellipse
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension


#Float64 for wheel velocities
rw_vel = 0.0
lw_vel = 0.0
wcom1 = 0.0
wcom2 = 0.0
xreal = np.matrix([[0.0],[0.0],[0.0]])
dt = 0.1


r = 0.12
L = 0.35

#Pose2D (x, y, theta) for beacon location relative to robot location
beacon_found = False
beacon_pose = Pose2D()
beacon_pose.x = 0
beacon_pose.y = 0
beacon_pose.theta = -180

#LaserScan information on the robot laserscan taopic
laser_scan = LaserScan()



#Get the sensor reading on the left wheel velocity
def lw_callback( lw_data ):
	global lw_vel
	lw_vel = lw_data.data
	#print lw_vel


#Get the sensor reading on the right wheel velocity
def rw_callback( rw_data ):
	global rw_vel
	rw_vel = rw_data.data
	#print rw_vel

def omega_callback( omega_data ):
    global wcom1, wcom2
    wcom1 = omega_data.data[0]
    wcom2 = omega_data.data[1]


#Read the beacon sensor. If theta is not between -90, 90 degrees beacon is not found
def beacon_callback(beacon_data ):
	global beacon_found, beacon_pose
	#the beacon is not detected if theta is -180
	#print beacon_data.theta
	if( beacon_data.theta == -180 ):
		beacon_found = False
		return
	beacon_found = True
	beacon_pose.x = beacon_data.x
	beacon_pose.y = beacon_data.y
	#The correct value will be sent in radians from [-pi/2, pi/2]
	beacon_pose.theta = beacon_data.theta

def pose_callback( pose_data ):
    global xreal
    xreal[0] = pose_data.x
    xreal[1] = pose_data.y
    xreal[2] = pose_data.theta

def dt_callback(dt_data ):
    global dt
    dt = dt_data.data
# update state using physics



# Single scan from a planar laser range-finder
#
#
#	The following is the list of data fields associtated with the laser scan message object
#

#Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         
#float32 angle_min        # start angle of the scan [rad]
#float32 angle_max        # end angle of the scan [rad]
#float32 angle_increment  # angular distance between measurements [rad]

#float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
#float32 scan_time        # time between scans [seconds]

#float32 range_min        # minimum range value [m]
#float32 range_max        # maximum range value [m]

#float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
#float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.


#Reads in the laser message sent on robot0/laser_0 topic
def lidar_callback( lidar_data ):
	global laser_scan
	laser_scan = lidar_data
    #print laser_scan.header
	
#def f(x, u, r, L, dt):
#    print x
#
#    print u
#    f = np.matrix([[x[0,0] + x[3,0]*dt],
#                  [x[1,0] + x[4,0] * dt],
#                  [x[2,0] + x[5,0] * dt],
#                  [0.0],
#                  [0.0],
#                  [0.0]])
#    return f
#
#
#def F(x, u, r, L, dt):
#    F = np.array([  [1, 0, 0, dt, 0, 0],
#                    [0, 1, 0, 0, dt, 0],
#                    [0, 0, 1, 0, 0, dt],
#                    [0, 0, 0, 0, 0, 0],
#                    [0, 0, 0, 0, 0, 0],
#                    [0, 0, 0, 0, 0, 0]])
#    return F
#
#def H(x, u, r, L, dt):
#    H = np.array([[0, 0, 0, x[3]/r/sqrt(x[3]^2 + x[4]^2), x[4]/r/sqrt(x[3]^2 + x[4]^2), -L/r],
#        [0, 0, 0, x[3]/r/sqrt(x[3]^2 + x[4]^2), x[4]/r/sqrt(x[3]^2 + x[4]^2), L/r],
#        [0, 0, 0, x[3]/r/sqrt(x[3]^2 + x[4]^2), x[4]/r/sqrt(x[3]^2 + x[4]^2), -L/r],
#        [0, 0, 0, x[3]/r/sqrt(x[3]^2 + x[4]^2), x[4]/r/sqrt(x[3]^2 + x[4]^2), L/r]])
#    return H
#
#H = np.matrix([[1, 0, 0], [0, 1, 0]])
def f(x, u, r, L, dt):
    #print x

    #print u
    f = np.matrix([ [x[0,0] + r*dt/2.0*(x[3,0] + x[4,0])*cos(x[2])],
                    [x[1,0] + r*dt/2.0*(x[3,0] + x[4,0])*sin(x[2])],
                    [x[2,0] + r*dt/2.0/L*(x[3,0] - x[4,0])],
                    [x[3,0] ],
                    [x[4,0] ]])
    return f


def F(x, u, r, L, dt):
    F = np.array([  [1, 0, -r*dt/2*(x[3,0] + x[4,0])*sin(x[2]), r*dt/2*cos(x[2,0]), r*dt/2*cos(x[2,0])],
                    [0, 1, r*dt/2*(x[3,0] + x[4,0])*cos(x[2]), r*dt/2*sin(x[2,0]), r*dt/2*sin(x[2,0])],
                    [0, 0, 1, r*dt/2/L, -r*dt/2/L],
                    [0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 1]])
    return F

def h(x, u, r, L, dr):
    h = np.array([[x[3,0]],[x[4,0]],[x[3,0]],[x[4,0]]])
    return h

H = np.matrix([  [0,0,0,1,0],
                [0,0,0,0,1],
                [0,0,0,1,0],
                [0,0,0,0,1]])

    
def listener():
    global xreal
    global r, L, dt
    global rw_vel, lw_vel
    global wcom1, wcom2
	#create the node
	#create the node
    rospy.init_node('read_sensors', anonymous=True)
	#create the subscribers
    rospy.Subscriber("/robot0/left_wheel",Float64,rw_callback)
    rospy.Subscriber("/robot0/right_wheel",Float64,lw_callback)
    rospy.Subscriber("/robot0/beacon",Pose2D, beacon_callback)
    rospy.Subscriber("/robot0/laser_0",LaserScan, lidar_callback) 
    rospy.Subscriber("/robot0/dt",Float64, dt_callback) 
    rospy.Subscriber("/robot0/pose2D",Pose2D, pose_callback) 
    rospy.Subscriber("/robot0/kinematic_params", Float64MultiArray, omega_callback)

    #xvec = np.ndarray(shape=(1,3,1))
    #xrealvec = np.ndarray(shape=(1,3,1))
    xvec = []
    yvec = []

    thetavec = []
    xrealvec = []
    yrealvec = []
    thetarealvec = []

    #x = np.matrix([[44.0],[111.0],[4.0],[0.0],[0.0]])
    x = np.matrix([[1.0],[2.0],[0.0],[0.0],[0.0]])
    xvec.append(x[0,0])
    yvec.append(x[1,0])
    thetavec.append(x[2,0])
    #xprev = np.matrix([[44.0],[111.0],[4.0],[0.0],[0.0],[0.0]])
    xprev = np.matrix([[1.0],[2.0],[0.0],[0.0],[0.0]])

    #xreal = np.matrix([[44.0],[111.0],[4.0]])
    xreal = np.matrix([[1.0],[2.0],[0.0]])
    xrealvec.append(xreal[0,0])
    yrealvec.append(xreal[1,0])
    thetarealvec.append(xreal[2,0])

    #np.append(xvec,x,0)
    #np.append(xrealvec,xreal,0)
    uprev = np.matrix([[0.0]])
    Pprev = np.matrix([ [0,0,0,0,0],
                        [0,0,0,0,0],
                        [0,0,0,0,0],
                        [0,0,0,0,0],
                        [0,0,0,0,0] ])

    P = np.eye(5)
    #P = np.matrix([ [0,0,0,0,0],
    #                [0,0,0,0,0],
    #                [0,0,0,0,0],
    #                [0,0,0,0,0],
    #                [0,0,0,0,0]])

    proc_var = 10000.0
    V = ([  [proc_var ,0,0,0,0],
            [0,proc_var ,0,0,0],
            [0,0,proc_var ,0,0],
            [0,0,0,proc_var,0],
            [0,0,0,0,proc_var]])
            #[0,proc_var,0,0,0],
            #[0,0,proc_var,0,0],
            #[0,0,0,proc_var,0],
            #[0,0,0,0,proc_var]])

    W = ([  [0.1**2,0,0,0],
            [0,0.1**2,0,0],
            [0,0,0.05**2,0],
            [0,0,0,0.05**2]])



    count = 0
    while True:
        count = count+1
# project previous state forward one time step
        #print x[0].shape
        #print xprev.shape
        x = f(xprev,uprev,r,L,dt)
        P = np.dot(np.dot(F(x,uprev,r,L,dt),P),F(x,uprev,r,L,dt).T) + V
        K = np.linalg.solve((np.dot(H, np.dot(P, H.T)) + W).T, np.dot(H,P.T)).T
        #print K
        z = np.matrix([[wcom1],[wcom2],[rw_vel],[lw_vel]])
        print dt
        #print z
        #print h(x,uprev,r,L,dt)
        #print "right, left"
        #print rw_vel
        #print lw_vel
        x = x + np.dot(K, z - h(x,uprev,r,L,dt))
        temp = P
        P = np.dot(np.eye(5) - np.dot(K, H), temp)
        xprev = x

        #print x
        #print rw_vel
        #uprev[0] = rw_vel
        #uprev[1] = lw_vel
        #np.append(xvec,x,0)
        #xvec.append(x)
        #np.append(xrealvec,xreal,0)
        #xrealvec.append(xreal)

        if count % 10 == 0:
            print "saved point"
            xvec.append(x[0,0])
            yvec.append(x[1,0])
            thetavec.append(x[2,0])
            xrealvec.append(xreal[0,0])
            yrealvec.append(xreal[1,0])
            thetarealvec.append(xreal[2,0])

        if count % 1000 == 0:
            plt.plot(xvec,yvec,'go')
            plt.plot(xrealvec,yrealvec,'b')
            plt.show()
# 

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

	#use sleep instead and use a while ros ok loop if you want to grab everything then do a computation 
	#and publish a message based on all subscribers

if __name__ == '__main__':
    listener()
