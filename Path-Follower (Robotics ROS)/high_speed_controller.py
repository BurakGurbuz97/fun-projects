#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
import tf
import math
from tf import transformations

#Global variables that manages the control
#Functions
ROUTE4 = False
waypoint = None
err_ang_prev = 0
K_LIN_P = 0.8
K_ANG_P = 0.7
K_ANG_D = 0.5
THRESHOLD = 0.3


def sign(x):
	return (1,-1)[x < 0]

def standart_control(err_lin, err_ang):
	#I dont want big forward jumps
	#truncate error since I loop frequenty it will be compensated
	if (abs(err_lin) > THRESHOLD):
		err_lin = THRESHOLD * sign(err_lin)

	#P control with icing(normalize with err_ang and add extra push)
	motor_x = ((K_LIN_P * err_lin)) * math.cos(abs(err_ang)) + 0.05 #give a small push
	#PD control for angular
	motor_z = K_ANG_P  * err_ang + K_ANG_D * (err_ang - err_ang_prev)

	return motor_x, motor_z


# Rotation transformation 
def rotate(x, y, degree):
	cos = math.cos(degree)
	sin = math.sin(degree)
	x_rot = cos*x + sin*y
	y_rot = -sin*x + cos*y
	return x_rot, y_rot

#waypoint callback
def waypoint_callback(msg): #  callback

    #***************************************
    #***          Obtain current destination
    #***************************************

    #save waypoint data for printing out in main loop
    global waypoint
    waypoint=msg;


if __name__ == '__main__':

    #setup ROS node, subscribe waypoint_cb to the topic /waypoint_cmd & publish motor commands
    rospy.init_node("crazy_driver_456")
    waypoint_subscriber = rospy.Subscriber("/waypoint_cmd", Transform, waypoint_callback) # <--- set up callback
    motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=100)
    #you could in principle also subscribe to the laser scan as is done in assignment 1.

    #setup transform cache manager
    listener = tf.TransformListener()

    
    #For faster looping
    delay = rospy.Rate(5);
    while not rospy.is_shutdown():

        #***************************************
        #***          Obtain current robot pose
        #***************************************
        
        try:
            #grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            
            (translation,orientation) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
        except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("EXCEPTION:",e)
            #if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            delay.sleep()
            continue
        

        #***************************************
        #***          Print current robot pose
        #***************************************

        #Print out the x,y coordinates of the transform
        print("Robot is believed to be at (x,y): (",translation[0],",",translation[1],")")

        #Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        r_xorient, r_yorient, r_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
        robot_theta = r_zorient  # only need the z axis
        print("Robot is believed to have orientation (theta): (",robot_theta,")\n")

        #***************************************
        #***          Print current destination
        #***************************************

        # the waypoint variable is filled in in the waypoint_callback function above, which comes from incoming messages
        # subscribed to in the .Subscriber call above.

        #Print out the x,y coordinates of the latest message
        print("Current waypoint (x,y): (",waypoint.translation.x,",",waypoint.translation.y,")")

        #Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        waypointrotq = [waypoint.rotation.x,waypoint.rotation.y,waypoint.rotation.z,waypoint.rotation.w]
        w_xorient, w_yorient, w_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(waypointrotq))
        waypoint_theta=w_zorient # only need the z axis
        print("Current waypoint (theta): (",waypoint_theta,")\n")

        #***************************************
        #***          DRIVE THE ROBOT HERE (same as with assignment 1 
        #**           - except you are driving towards a goal not away from an obstacle)
        #***************************************

        #Default control function and coefficents work well with
        #ROUTE1, ROUTE2 and ROUTE3 however ROUTE for needed little bit 
        #hardcoding 
        #First turtlebot detects that it is on the ROUTE4 by checking waypoint
        #It starts going backward
        #Then switchs back to P(speed) and PD(angular) control
        #Further, it converte PD(angular) to P(angular) control
        #When route became curly
        if ((not ROUTE4)  and (waypoint.translation.x == -0.32561117580808513)):
			ROUTE4 = True
			print("ROUTE4 Special control activated")
        if ((ROUTE4) and (waypoint.translation.x == -3.411882458861898)):
            ROUTE4 = False
            K_LIN_P = 0.8
            K_ANG_P = 0.5
            K_ANG_D = 0.4
            print("Swithing back to standart control (with tailor made coefficients)")
        if ((not ROUTE4) and (waypoint.translation.x == 0.48138858294916437)):
            K_LIN_P = 0.7
            K_ANG_P = 0.6
            K_ANG_D = 0.0
            print("Shutting down derivative control for angular")


        #for containing the motor commands to send to the robot
        motor_command=Twist()

        #Find the difference between waypoint and robot
        dif_x = waypoint.translation.x - translation[0]
        dif_y = waypoint.translation.y - translation[1]

        #Apply rotation to coordinates
        x, y = rotate(dif_x, dif_y, robot_theta) 

        #Compute the distance
        err_lin = math.hypot(x, y)
        #Compute the angular error
        err_ang = math.atan2(y, x)

        if not ROUTE4:
        	motor_command.linear.x, motor_command.angular.z = standart_control(err_lin, err_ang)
        	err_ang_prev = err_ang
        	motor_command_publisher.publish(motor_command)
        else:
        	motor_command.linear.x = -0.5
        	motor_command_publisher.publish(motor_command)

        
        delay.sleep()
        # we don't need to call spinOnce like in roscpp as callbacks happen in different threads
    
    
    print("ROS shutdown now I will also go to sleep. I hope I didn't crash. Night night.")
