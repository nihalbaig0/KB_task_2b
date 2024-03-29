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

# Team ID:			[ KB_1842 ]
# Author List:		[ Nihal Baig , Ad-Deen Mahbub ]
# Filename:			percepStack.py
# Functions:		shape_color_centroid , flatten , img_clbck , depth_clbck , image_processing
# 					[ Comma separated list of functions in this file ]


####################### IMPORT MODULES #######################
import cv2 
from matplotlib import pyplot as plt
import rospy
from std_msgs.msg import Int16MultiArray , Float32MultiArray 
from cv_bridge import CvBridge, CvBridgeError 
from sensor_msgs.msg import Image
# from std_msgs.msg import String
import numpy as np
# import matplotlib.image as mpimg
# You can add more if required
##############################################################
import math

# Initialize Global variables

bridge = CvBridge()

global pose2
pose2= []
global zer 
global float_msg,int_msg
float_msg = Float32MultiArray()
int_msg = Int16MultiArray()
# global a
# a = None
################# ADD UTILITY FUNCTIONS HERE #################
def shape_color_centroid(th):
    
    '''
    A function that determines the shape and color of given object.
    '''
    shapes=[]
    countours, _ = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # print(type(countours))
    # cv2.imshow('contours', countours)                        #cv2 img show
    # cv2.waitKey(0)
    for contour in countours:
        approx = cv2.approxPolyDP(contour, 0.009 * cv2.arcLength(contour, True), True)
        # cv2.imshow('shapes', shapes)                        #cv2 img show
        # cv2.waitKey(0)
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
    # cv2.imshow('shapes', shapes)                        #cv2 img show
    # cv2.waitKey(0)
    # print(shapes)
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
    # global depth_counter , img_counter
    # depth_counter = 1
    # img_counter = 1
    global pub_rgb ,bridge, zer#, add global variable if any

    ############################### Add your code here #######################################
    try:
        image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    # print(image.shape)
    # img = cv2.imread(image, -1)
    # plt.imshow(image, interpolation='nearest')        #matplotlib img show
    # plt.show()
    # Displaying the image
    # cv2.imshow('image', image)                        #cv2 img show
    # cv2.waitKey(0)
    #print(image.shape)
    # zer = np.zeros_like(image)
    # print(zer)
    ##########################################################################################
    pose = image_processing(image)
    # for i in len(pose):
    #     for j in len(pose[i]):
    #         pose[i][j] = int(pose[i][j])
    int_msg.data = pose
    # int_msg.serialize('5')
    pub_rgb.publish(int_msg)
    print(pose)
    rospy.sleep(1)

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
    global pub_depth , zer 
    depth_image = bridge.imgmsg_to_cv2(depth_msg, "32FC1")
    #print(depth_image.shape)
    x=0
    y=0
    depth_val = []
    if len(pose2)>0:
        #y , x = pose2[0][0] , pose2[0][1]
    # y = int(y * 0.6667)
    # x = int(x* 0.6625)
    # print(depth_image[y][x])
        for y, x in pose2:
            K = np.array([476.7030836014194, 0.0, 400.5, 0.0, 476.7030836014194, 400.5, 0.0, 0.0, 1.0])
            m_fx = K[0]
            m_fy = K[4]
            m_cx = K[2]
            m_cy = K[5]
            inv_fx = 1. / m_fx
            inv_fy = 1. / m_fy
            depth_array = np.array(depth_image, dtype=np.float32)
            cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
            depth_8 = (depth_array * 255).round().astype(np.uint8)
            cv_depth = np.zeros((480,848,3),dtype=np.uint8)
            #print(cv_depth)
            cv_depth[:,:,0] = depth_8
            cv_depth[:,:,1] = depth_8
            cv_depth[:,:,2] = depth_8
            
                
            n = 0
            sum = 0
            for i in range(0,depth_image.shape[0]):
                for j in range(0,depth_image.shape[1]):
                    value = depth_image.item(i, j)
                    if value > 0.:
                        n = n + 1
                        sum = sum + value
                
            mean_z = sum / n
                
            point_z = mean_z * 0.001; # distance in meters
            point_x = (x - m_cx) * point_z * inv_fx;
            point_y = (y - m_cy) * point_z * inv_fy;
                
            #print("here")
                        
        
                        
            dist = math.sqrt(point_x * point_x + point_y * point_y + point_z * point_z)
            #print(dist)
            g = float("{:.1f}".format(dist))
            depth_val.append(g)
        
        print(depth_val)

        float_msg.data = depth_val
        pub_depth.publish(float_msg)
        # pose2 = 0
        rospy.sleep(1)
        # rospy.sleep(1)
        
    else: 
        pass
    ############################### Add your code here #######################################
    #print("distance" + str(dist))
    ##########################################################################################
    # print(depth_val)
    # rospy.sleep(1)
    # pub_depth.publish(str(depth_val))


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
    width = 848
    height = 480
    dim = (width, height)
    resized_image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    # plt.imshow(resized_image, interpolation='nearest')        #matplotlib img show
    # plt.show()
    #print(resized_image.shape)
    # Line thickness of 2 px
    thickness = 2
    imgHSV = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)
    # cv2.imshow("resized",resized_image)
    # cv2.waitKey(0)
   
    for idx , (lower, upper) in enumerate(boundaries):
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            # find the colors within the specified boundaries and apply the mask
            mask = cv2.inRange(imgHSV, lower, upper)
            # cv2.imshow("mask",mask)
            # cv2.waitKey(0)
            # img = mpimg.imread(mask)
            # plt.imshow(img)
            #print(shape_color_centroid(mask))
            #pose.append(shape_color_centroid(mask))
            if len(shape_color_centroid(mask)) > 0:
                pose.append(shape_color_centroid(mask))
                center_coordinates = tuple(shape_color_centroid(mask)[0])
            
                #image = cv2.circle(resized_image, center_coordinates, radius, color, thickness)
                #cv2.imshow(str(idx),resized_image)
    #cv2.waitKey(0)
    pose2 = flatten(pose)
    # print(pose2)
    pose = flatten(pose)
    # print(pose)
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
    pub_rgb = rospy.Publisher('/center_rgb', Int16MultiArray, queue_size = 1)
    pub_depth = rospy.Publisher('/center_depth', Float32MultiArray, queue_size = 1)
    while not rospy.is_shutdown():    
        # print(f'Image no. {i}')
        print("image 1")
        sub_image = rospy.Subscriber(f"/device_0/sensor_1/Color_0/image/data_1",Image, img_clbck)
        rospy.sleep(1.16)
        sub_image.unregister()
        sub_depth = rospy.Subscriber(f"/device_0/sensor_0/Depth_0/image/data_1",Image, depth_clbck)
        rospy.sleep(1)
        sub_depth.unregister()
        print("image 2")
        sub_image = rospy.Subscriber(f"/device_0/sensor_1/Color_0/image/data_2",Image, img_clbck)
        rospy.sleep(1)
        sub_image.unregister()
        sub_depth = rospy.Subscriber(f"/device_0/sensor_0/Depth_0/image/data_2",Image, depth_clbck)
        rospy.sleep(1)
        sub_depth.unregister()
        print("image 3")
        sub_image = rospy.Subscriber(f"/device_0/sensor_1/Color_0/image/data_3",Image, img_clbck)
        rospy.sleep(1)
        sub_image.unregister()
        sub_depth = rospy.Subscriber(f"/device_0/sensor_0/Depth_0/image/data_3",Image, depth_clbck)
        rospy.sleep(1)
        sub_depth.unregister()
        

        
        # sub1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_1", Image, img_clbck)
        # rospy.sleep(6)
        # pub1 =rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_1", Image, depth_clbck)
        # pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
        # pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)
        # rospy.sleep(6)
        # sub1.unregister()
        # rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_2", Image, img_clbck)
        # rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_2", Image, depth_clbck)
        # pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
        # pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)
        # # rospy.sleep(1)
        # rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_3", Image, img_clbck)
        # rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_3", Image, depth_clbck)
        # pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
        # pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)

        ####################################################################################################
    # rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")
