#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 2.2 of Krishi Bot (KB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			percepStack.py
# Functions:		
# 					[ Comma separated list of functions in this file ]


####################### IMPORT MODULES #######################
import cv2 
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
# You can add more if required
##############################################################


# Initialize Global variables

bridge = CvBridge()

global pose2
pose2= []
################# ADD UTILITY FUNCTIONS HERE #################
def shape_color_centroid(th):
    
    '''
    A function that determines the shape and color of given object.
    '''
    shapes=[]
    countours, _ = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in countours:
        approx = cv2.approxPolyDP(contour, 0.009 * cv2.arcLength(contour, True), True)

        n = approx.ravel()

        if n[0] != 0:
            cnts = []

            area = cv2.contourArea(contour)
            #print(area)
            M = cv2.moments(contour)

            # calculate x,y coordinate of center
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            cnts.append(cX)
            cnts.append(cY)
            if area > 1000:
                shapes.append(cnts)
    return shapes


def flatten(l):
    return [item for sublist in l for item in sublist]
##############################################################


def img_clbck(img_msg):
    '''
    Callback Function for RGB image topic

    Purpose:
    -----
    Convert the image in a cv2 format and then pass it 
    to image_processing function by saving to the 
    'image' variable.

    Input Args:
    -----
    img_msg: Callback message.
    '''
    global pub_rgb ,bridge#, add global variable if any

    ############################### Add your code here #######################################
    try:
        image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    print(image.shape)
    ##########################################################################################
    pose = image_processing(image)
    pub_rgb.publish(str(pose))

def depth_clbck(depth_msg):
    '''
    Callback Function for Depth image topic

    Purpose:
	--- 
    1. Find the depth value of the centroid pixel returned by the
    image_processing() function.
    2. Publish the depth value to the topic '/center_depth'


    NOTE: the shape of depth and rgb image is different. 
    
    Input Args:
    -----
    depth_msg: Callback message.
    '''
    global pub_depth
    depth_image = bridge.imgmsg_to_cv2(depth_msg, "32FC1")
    #print(depth_image.shape)
    x=0
    y=0
    depth_val = []
    if len(pose2)>0:
        y , x = pose2[0][0] , pose2[0][1]
    y = int(y * 0.6667)
    x = int(x* 0.6625)
    #print(pose2)
   
    
    ############################### Add your code here #######################################
    print(depth_image[y][x])
    ##########################################################################################
    pub_depth.publish(str(depth_val))


def image_processing(image):
    '''
    NOTE: Do not modify the function name and return value.
          Only do the changes in the specified portion for this
          function.
          Use cv2.imshow() for debugging but make sure to REMOVE it before submitting.
    
    1. Find the centroid of the bell pepper(s).
    2. Add the x and y values of the centroid to a list.  
    3. Then append this list to the pose variable.
    3. If multiple fruits are found then append to pose variable multiple times.

    Input Args:
    ------
    image: Converted image in cv2 format.

    Example:
    ----
    pose = [[x1, y1] , [x2, y2] ...... ]
    '''
    global pose2
    pose = []
    ############### Write Your code to find centroid of the bell peppers #####################
    boundaries = [
        ([9, 147, 103], [22, 255, 255]),
        ([171, 122, 69], [179, 255, 255]),

    ]
    # Radius of circle
    radius = 2

    # Blue color in BGR
    color = (255, 0, 0)

    # Line thickness of 2 px
    thickness = 2
    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    for idx , (lower, upper) in enumerate(boundaries):
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            # find the colors within the specified boundaries and apply the mask
            mask = cv2.inRange(imgHSV, lower, upper)
            #print(shape_color_centroid(mask))
            pose.append(shape_color_centroid(mask))
            center_coordinates = tuple(shape_color_centroid(mask)[0])
            image = cv2.circle(image, center_coordinates, radius, color, thickness)
            #cv2.imshow(str(idx),image)
    #cv2.waitKey(0)
    pose2 = flatten(pose)
    ##########################################################################################
    return pose



def main():
    '''
    MAIN FUNCTION

    Purpose:
    -----
    Initialize ROS node and do the publish and subscription of data.

    NOTE: We have done the subscription only for one image, you have to iterate over 
    three images in the same script and publish the centroid and depth in the 
    same script for three images, calling the same callback function.

    '''

    #### EDIT YOUR CODE HERE FOR SUBSCRIBING TO OTHER TOPICS AND TO APPLY YOUR ALGORITHM TO PUBLISH #####
    global pub_rgb, pub_depth
    rospy.init_node("percepStack", anonymous=True)
    sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_1", Image, img_clbck)
    sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_1", Image, depth_clbck)


    pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
    pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)

    ####################################################################################################
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")