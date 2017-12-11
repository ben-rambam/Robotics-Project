#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension




#Float64 for wheel velocities
rw_vel = 0.0
lw_vel = 0.0

#Pose2D (x, y, theta) for beacon location relative to robot location
beacon_found = False
beacon_pose = Pose2D()
beacon_pose.x = 0
beacon_pose.y = 0
beacon_pose.theta = -180

#LaserScan information on the robot laserscan taopic
laser_scan = LaserScan()


#creat the wheel-speed publisher
pub = rospy.Publisher('robot0/kinematic_params', Float64MultiArray, queue_size=10)



#Get the sensor reading on the left wheel velocity
def lw_callback( lw_data ):
	global lw_vel
	lw_vel = lw_data.data
	print lw_vel


#Get the sensor reading on the right wheel velocity
def rw_callback( rw_data ):
	global rw_vel
	rw_vel = lw_data.data
	print rw_vel


#Read the beacon sensor. If theta is not between -90, 90 degrees beacon is not found
def beacon_callback(beacon_data ):
	global beacon_found, beacon_pose
	#the beacon is not detected if theta is -180
	print beacon_data.theta
	if( beacon_data.theta == -180 ):
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
def lidar_callback( lidar_data ):
	global laser_scan
	laser_scan = lidar_data
    #print laser_scan.header
	

def hit_wall():
    global laser_scan
    if laser_scan.angle_increment != 0:
        zero_index = int((0-laser_scan.angle_min)/laser_scan.angle_increment)
        left_index = 160
        right_index = 200
        if laser_scan.ranges[left_index] < laser_scan.range_max/2.0 or\
             laser_scan.ranges[right_index] < laser_scan.range_max/2.0:
            return True
        else:
            return False


    
def listener():
    global pub
	#create the node
    rospy.init_node('read_sensors', anonymous=True)
	#create the subscribers
    rospy.Subscriber("/robot0/left_wheel",Float64,lw_callback)
    rospy.Subscriber("/robot0/right_wheel",Float64,rw_callback)
    rospy.Subscriber("/robot0/beacon",Pose2D, beacon_callback)
    rospy.Subscriber("/robot0/laser_0",LaserScan, lidar_callback) 

# infinite loop
    while True:
        speed = 8
        r = 0.1
        L = 0.2
# while we haven't hit a wall, go straight
        if hit_wall() != True:
            w1 = speed
            w2 = speed

# while we are hitting a wall, turn left
        else:
            w1 = speed
            w2 = -speed
        data = Float64MultiArray(data=[])
        data.layout = MultiArrayLayout()
        data.layout.dim = [MultiArrayDimension()]
        data.layout.dim[0].label = "Parameters"
        data.layout.dim[0].size = 4
        data.layout.dim[0].stride = 1
        data.data = [w1,w2,r,L]
        pub.publish(data)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

	#use sleep instead and use a while ros ok loop if you want to grab everything then do a computation 
	#and publish a message based on all subscribers

if __name__ == '__main__':
    listener()
