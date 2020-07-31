#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray as S
import math
import time

calc = lambda x: (1.0/1000)*(9000 - x)

pub = None

DETECTED_SIGN = "n"
SIGN_SIZE = 0

MOST_LEFT = 0
HALF_LEFT = 0
VERTICAL = 0
HALF_RIGHT = 0
MOST_RIGHT = 0

err_prev = 0
k_err = 0.005
k_der_err = 0.01

def scan_callback(S):
	global MOST_LEFT
	global HALF_LEFT
	global VERTICAL
	global HALF_RIGHT	
	global MOST_RIGHT

	MOST_LEFT, HALF_LEFT, VERTICAL, HALF_RIGHT, MOST_RIGHT  = S.data


def sign_callback(msg):
	global DETECTED_SIGN
	global SIGN_SIZE
	msg = str(msg)
	DETECTED_SIGN = msg[7]
	SIGN_SIZE = int(msg[9:-1])


if __name__ == '__main__':
	rospy.init_node("motor_control")
	rospy.Subscriber('/sign/detector', String, sign_callback)
	rospy.Subscriber('/scan_node/scan_distances', S, scan_callback)

	pub = rospy.Publisher("/cmd_vel", Twist,queue_size=3)
	delay = rospy.Rate(10)
	while not rospy.is_shutdown():
		motor_command=Twist()
		detected = DETECTED_SIGN
		sign_size = SIGN_SIZE
		if (DETECTED_SIGN == "l" or DETECTED_SIGN == "r"):
			if SIGN_SIZE > 1000:
				print("Now I am going straight distance", sign_size)
				start = rospy.Time.now()
				while (rospy.Time.now() - start < rospy.Duration(calc(sign_size))):
					motor_command.linear.x  = 0.45
					motor_command.angular.z = 0.0
					pub.publish(motor_command)

				print("Turning")
				start = rospy.Time.now()
				while (rospy.Time.now() - start < rospy.Duration(3.52)):
					motor_command.linear.x  = 0.3
					motor_command.angular.z = -1.1 if detected == "r" else 1.1
					pub.publish(motor_command)
			else:
				motor_command.linear.x  = 0.3
				motor_command.angular.z = 0.0
				pub.publish(motor_command)
		else:
			if DETECTED_SIGN == "s":
				motor_command.linear.x  = 0.0
				motor_command.angular.z = 0.0
				pub.publish(motor_command)
			else:
				if not(math.isinf(MOST_RIGHT) or math.isinf(MOST_LEFT) or MOST_LEFT > 1.4  or MOST_RIGHT > 1.4):
					err = MOST_RIGHT - MOST_LEFT
					motor_z = k_err * err + k_der_err * (err - err_prev)
					err_prev = err
					motor_command.linear.x  = 0.3
					motor_command.angular.z = -err
					pub.publish(motor_command)
				else:
					motor_command.linear.x  = 0.3
					motor_command.angular.z = 0.0
					pub.publish(motor_command)
		delay.sleep()
