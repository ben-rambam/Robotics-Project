#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from enum import Enum

State = Enum('State', 'go_away pivot_right go_to pivot_left')

#Float64 for wheel velocities
rw_vel = 0.0
lw_vel = 0.0

#Pose2D (x, y, theta) for beacon location relative to robot location
beacon_found = False
beacon_pose = Pose2D()
beacon_pose.x = 0
beacon_pose.y = 0
beacon_pose.theta = -180

front_right_dist = 2.0
rear_right_dist = 2.0
r = 0.12
L = 0.35

#LaserScan information on the robot laserscan taopic
laser_scan = LaserScan()

#creat the wheel-speed publisher
pub = rospy.Publisher(
    'robot0/kinematic_params', Float64MultiArray, queue_size=10)


#Get the sensor reading on the left wheel velocity
def lw_callback(lw_data):
    global lw_vel
    lw_vel = lw_data.data
    #print lw_vel

#Get the sensor reading on the right wheel velocity
def rw_callback(rw_data):
    global rw_vel
    rw_vel = rw_data.data
    #print rw_vel


#Read the beacon sensor. If theta is not between -90, 90 degrees beacon is not found
def beacon_callback(beacon_data):
    global beacon_found, beacon_pose
    #the beacon is not detected if theta is -180
    #print beacon_data.theta
    if (beacon_data.theta == -180):
        beacon_found = False
        return
    beacon_found = True
    beacon_pose.x = beacon_data.x
    beacon_pose.y = beacon_data.y
    #The correct value will be sent in radians from [-pi/2, pi/2]
    beacon_pose.theta = beacon_data.theta


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
def lidar_callback(lidar_data):
    global laser_scan
    global rear_right_dist, front_right_dist
    laser_scan = lidar_data
    rear_right_dist = laser_scan.ranges[45]
    front_right_dist = laser_scan.ranges[135]


#print laser_scan.header


def hit_wall():
    global laser_scan
    if laser_scan.angle_increment != 0:
        zero_index = int(
            (0 - laser_scan.angle_min) / laser_scan.angle_increment)
        left_index = 160
        right_index = 200
        if laser_scan.ranges[left_index] < laser_scan.range_max/2.0 or\
             laser_scan.ranges[right_index] < laser_scan.range_max/2.0:
            return True
        else:
            return False

def follow_wall_right():
    global laser_scan
    if laser_scan.angle_increment !=0:
        zero_index = int((0 - laser_scan.angle_min) / laser_scan.angle_increment)
        rear_index_right = 315
        forward_index_right = 225

def drive_straight(speed):
        global r, L
        w1 = speed
        w2 = speed
        data = Float64MultiArray(data=[])
        data.layout = MultiArrayLayout()
        data.layout.dim = [MultiArrayDimension()]
        data.layout.dim[0].label = "Parameters"
        data.layout.dim[0].size = 4
        data.layout.dim[0].stride = 1
        data.data = [w1, w2, r, L]
        pub.publish(data)

def pivot_left(speed):
        global r, L
        w1 = speed
        w2 = -speed
        data = Float64MultiArray(data=[])
        data.layout = MultiArrayLayout()
        data.layout.dim = [MultiArrayDimension()]
        data.layout.dim[0].label = "Parameters"
        data.layout.dim[0].size = 4
        data.layout.dim[0].stride = 1
        data.data = [w1, w2, r, L]
        pub.publish(data)

def pivot_right(speed):
        global r, L
        w1 = -speed
        w2 = speed
        data = Float64MultiArray(data=[])
        data.layout = MultiArrayLayout()
        data.layout.dim = [MultiArrayDimension()]
        data.layout.dim[0].label = "Parameters"
        data.layout.dim[0].size = 4
        data.layout.dim[0].stride = 1
        data.data = [w1, w2, r, L]
        pub.publish(data)

def listener():
    global pub
    #create the node
    rospy.init_node('read_sensors', anonymous=True)
    #create the subscribers
    rospy.Subscriber("/robot0/left_wheel", Float64, lw_callback)
    rospy.Subscriber("/robot0/right_wheel", Float64, rw_callback)
    rospy.Subscriber("/robot0/beacon", Pose2D, beacon_callback)
    rospy.Subscriber("/robot0/laser_0", LaserScan, lidar_callback)

    state = State.go_to
    rear_max = 1.5
    front_max = 1.5
    rear_min = 1.0
    front_min = 1.0
    speed = 1

    # infinite loop
    while True:
        
        if state == State.go_away:
            if rear_right_dist > rear_max:
                state = State.pivot_right
                print "go_away -> pivot_right"
                print "rear: " 
                print rear_right_dist
                print "front: " 
                print front_right_dist
            else:
                drive_straight(speed)
        elif state == State.pivot_right:
            if front_right_dist < front_max:
                state = State.go_to
                print "pivot_right -> go_to"
                print "rear: " 
                print rear_right_dist
                print "front: " 
                print front_right_dist
            else:
                pivot_right(speed)
        elif state == State.go_to:
            if front_right_dist < front_min:
                state = State.pivot_left
                print "go_to -> pivot_left"
                print "rear: " 
                print rear_right_dist
                print "front: " 
                print front_right_dist
            else:
                drive_straight(speed)
        elif state == State.pivot_left:
            if rear_right_dist < rear_min:
                state = State.go_away
                print "pivot_left -> go away"
                print "rear: " 
                print rear_right_dist
                print "front: " 
                print front_right_dist
            else:
                pivot_left(speed)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

#use sleep instead and use a while ros ok loop if you want to grab everything then do a computation
#and publish a message based on all subscribers

if __name__ == '__main__':
    listener()
