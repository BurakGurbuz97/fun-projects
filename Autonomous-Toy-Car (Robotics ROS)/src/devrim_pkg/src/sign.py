#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


bridge = CvBridge()
publisher = None

#n -> no sign, l -> left, r -> right, s -> stop  
STATE_COUNTER = 0
STATE = "l"

STOP_SIGN_THRESHOLD = 99999999999
LOWER_RANGE_RED = np.array([0,50,50])
UPPER_RANGE_RED = np.array([10,255,255])
LOWER_RANGE_BLUE = np.array([120,50,50])
UPPER_RANGE_BLUE = np.array([130,255,255])

def is_valid(croped):
    number_of_pixels = croped.shape[0] * croped.shape[1]
    if number_of_pixels < 50:
        return False
    return True

#TRUE -> right , False -> left
def find_direction(croped):
    col = croped.shape[1]
    left, right = np.sum(croped[:,0:int(col/2)]), np.sum(croped[:,int(col/2):])
    if left > right:
        return True
    else:
        return False
   
def mask_red(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, LOWER_RANGE_RED, UPPER_RANGE_RED)
    res = cv2.bitwise_and(img,img, mask= mask)
    res = cv2.GaussianBlur(res, (7,7),cv2.BORDER_DEFAULT)
    imgray = np.sum(res, axis = 2).astype(np.uint8)
    return imgray

def mask_blue(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, LOWER_RANGE_BLUE, UPPER_RANGE_BLUE)
    res = cv2.bitwise_and(img,img, mask= mask)
    res = cv2.GaussianBlur(res, (7,7),cv2.BORDER_DEFAULT)
    imgray = np.sum(res, axis = 2).astype(np.uint8)
    return imgray

def crop_sign(img):
    ret, thresh = cv2.threshold(img, 20, 255, 0)
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) < 2:
        raise Exception(len(contours))
    mask = np.zeros_like(img)
    mask = cv2.drawContours(mask, contours , 1,  255, -1)
    y, x = np.where(mask == 255)
    topy, topx = (np.min(y), np.min(x))
    bottomy, bottomx = (np.max(y), np.max(x))
    croped1 = img[topy:bottomy+1, topx:bottomx+1]
    return croped1



def callback(img):
	global bridge
	global STATE_COUNTER
	global STATE
	found_state = None

	img = bridge.imgmsg_to_cv2(img, "rgb8")
	cv2.resize(img, (252, 336))
	blues = mask_blue(img)
	reds = mask_red(img)
	size = 0
	try:
	    if (np.sum(reds) > STOP_SIGN_THRESHOLD):
	        found_state = "s"
	        size = np.sum(reds)
	    else:    
	        croped = crop_sign(blues)
	        if not is_valid(croped):
	            raise Exception
	        if (find_direction(croped)):
	            found_state = "r"
	        else:
	            found_state = "l"
	        size = croped.shape[0] * croped.shape[1]
	except :
	    found_state = "n"

	if found_state == STATE:
		if (STATE_COUNTER > 10):
			STATE = found_state
		else:
			STATE_COUNTER += 1
	else:
		STATE = found_state
		STATE_COUNTER = 0

	publisher.publish(STATE + "," + str(size))



if __name__ == '__main__':
	rospy.init_node("sing_detect")
	rospy.Subscriber("/camera/rgb/image_raw", Image, callback)
	publisher = rospy.Publisher('/sign/detector', String, queue_size=10)
	rospy.spin()

