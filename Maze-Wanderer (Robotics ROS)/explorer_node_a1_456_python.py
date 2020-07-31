#!/usr/bin/env python
## AK
## explorer_node_py.py
##
## BLG456E Assignment 1 skeleton
##
## Instructions: Change the laser_callback function to make the robot explore more
## intelligently, using its sensory data (the laser range array).
##
## Advanced: If you want to make use of the robot's mapping subsystem then you can
## make use of the map in the mapping_callback function.
##
## 

## Common ROS headers.
import rospy
## Required for some printing options
import sys

## This is needed for the data structure containing the motor command.
from geometry_msgs.msg import Twist
## This is needed for the data structure containing the laser scan
from sensor_msgs.msg import LaserScan
## This is needed for the data structure containing the map (which you may not use).
from nav_msgs.msg import OccupancyGrid
import math
import random
import collections 

## The following function is a "callback" function that is called back whenever a new laser scan is available.
## That is, this function will be called for every new laser scan.
##
## --------------------------------------------------------------------------------
## ----------CHANGE THIS FUNCTION TO MAKE THE ROBOT EXPLORE INTELLIGENTLY----------
## --------------------------------------------------------------------------------
##

# If stuck or bump detected this counter will increase and turtlebot will reverse
STUCK_COUNTER = 0

# Number of turns published so far, In each 10 turns flip a coin to decide next turn randomly
# To ensure turtlebot can explore new regions
ITERATOR = 0

# Left or Right Turn coeeficient
TURN = 1

# Memorize past 30 scans to act smarter
MEMORY = collections.deque(maxlen = 30)


#Standart Deviation calculation (this could be vectorized for efficiency)
def std(array):
	mean = sum(array) / len(array)
	_sum = 0
	for i in  array:
		_sum += (i - mean)**2
	_sum / (len(array) - 1)
	return _sum ** 0.5
	 


#Check if the turtlebot stucked or bumped
#This function calculates the standart deviation of memory tuples
#Small std on memory means turtlebot is stucked or bumbed
def get_state():
	global STUCK_COUNTER
	global MEMORY
 
	#Do not compute all at once since it is an kinda "expensive" operation
 	front_var = std([i[1] for i in MEMORY])
	print "FRONT_STD: ", front_var
	if front_var < 0.5:
		left_var  = std([i[0] for i in MEMORY])
		print "LEFT_STD: ",  left_var
		if left_var < 0.5:
			right_var = std([i[2] for i in MEMORY])
			print "RIGHT_STD: ",  right_var
			if right_var < 0.5:
				STUCK_COUNTER = 6
	

#Handle laser scan input
def laser_callback(data):
	global TURN
	global ITERATOR
	global MEMORY
	global STUCK_COUNTER
	global motor_command_publisher
	#threshold value for "turn right or left" decision
	Threshold = 1.5

	motor_command = Twist()

	#Turn around if stucked, Once stuck detected next "6" scans will be ignored
	if STUCK_COUNTER != 0:
		motor_command.linear.x = 0.0
		motor_command.angular.z = 3.0
		STUCK_COUNTER -= 1
		motor_command_publisher.publish(motor_command)
		print "I am stucked"
		return

	#Dont use exact values, intead scan an interval and compute min, otherwise Tiny objects may be overlooked
	front_scan = min(data.ranges[len(data.ranges)/2 - 20: len(data.ranges)/2 + 20])
	left_scan = min(data.ranges[0:10])
	right_scan = min(data.ranges[-10:-1])

	#Fix NaN problem
	if math.isnan(front_scan):
		front_scan = 10
	if math.isnan(left_scan):
		left_scan = 10
	if math.isnan(right_scan):
		right_scan = 10
	
	#Push new memory, double ended queue structure will throw away old memories
	MEMORY.append((left_scan, front_scan, right_scan))

	#Forward
	if front_scan > Threshold:
		motor_command.linear.x = 1.0
		motor_command.angular.z = 0
	#Turn
	else:
		#Change Turn side randomly for better exploration
		if (ITERATOR % 10 == 0):
			if random.randint(1,101) % 2 == 0:
				TURN = 1
			else:
				TURN = -1
			print "Next Turn Choice: ", TURN, " Iterator: ", ITERATOR

		motor_command.linear.x = 0.0
		motor_command.angular.z = 3 * TURN
		ITERATOR += 1

	## Lets publish that command so that the robot follows it
	motor_command_publisher.publish(motor_command)			
	
	print front_scan
	#Check if stucked or bumped
	if len(MEMORY) > 10:
		get_state()
	

	


## You can also make use of the map which is being built by the "gslam_mapping" subsystem
## There is some code here to help but you can understand the API also by looking up the OccupancyGrid message and its members (this is the API for the message)
## If you want me to explain the data structure, I will - just ask me in advance of class
def map_callback(data):
    chatty_map = False
    if chatty_map:
        print "-------MAP---------"
        ## Here x and y has been incremented with five to make it fit in the terminal
        ## Note that we have lost some map information by shrinking the data
        for x in range(0,data.info.width-1,5):
            for y in range(0,data.info.height-1,5):
                index = x+y*data.info.width
                if data.data[index] > 50:
                    ## This square is occupied
                    sys.stdout.write('X')
                elif data.data[index] >= 0:
                    ## This square is unoccupied
                    sys.stdout.write(' ')
                else:
                    sys.stdout.write('?')
            sys.stdout.write('\n')
        sys.stdout.flush()
        print "-------------------"
    
## This is the method we initilize everything
def explorer_node():
    ## We must always do this when starting a ROS node - and it should be the first thing to happen
    rospy.init_node('amble')
    
    ## Here we declare that we are going to publish "Twist" messages to the topic /cmd_vel_mux/navi. It is defined as global because we are going to use this publisher in the laser_callback.
    global motor_command_publisher
    motor_command_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
    
    ## Here we set the function laser_callback to recieve new laser messages when they arrive
    rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size = 1000)
    
    ## Here we set the function map_callback to recieve new map messages when they arrive from the mapping subsystem
    rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size = 1000)
    
    ## spin is an infinite loop but it lets callbacks to be called when a new data available. That means spin keeps this node not terminated and run the callback when nessessary. 
    rospy.spin()
    
if __name__ == '__main__':
    explorer_node()
