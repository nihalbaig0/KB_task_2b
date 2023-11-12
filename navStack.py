#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
global turn
global z
global x
global kp
global kd
global preverror
kp=3.50
kd=0.2
x=1.25
z=0.0
turn =1.35
preverror = 0
dt = 0.02


def pid(curr_error,kp,kd):
    # PD controller

    global preverror
    error_deriv = (curr_error - preverror) / dt
    preverror = curr_error
    control = kp * curr_error + kd* error_deriv
    return control

def laser_callback(msg:LaserScan):
    velocity_msg = Twist()
    global regions

    #Divided the scan area into 6 regions and working with left side only
    regions = {
        # 'Right3': avgmaker(list(msg.ranges[55:65])) ,
        # 'Right2': avgmaker(list(msg.ranges[175:185])) ,
        'Front': min(min(msg.ranges[350:370]),10),
        'Left1': min(min(msg.ranges[415:425]),10) ,
        'Left2': min(min(msg.ranges[535:545]),10) ,
        'Left3': min(min(msg.ranges[655:665]),10)}

    # print(f"R3={regions['Right3']}")
    # print(f"R2={regions['Right2']}")
    # print(f"front={regions['Front']}")
    # print(f"L1={regions['Left1']}")
    # print(f"L2={regions['Left2']}")
    # print(f"L3={regions['Left3']}")
    # print(f"Current time = {rospy.get_time()}")

    #The logics to navigate are given here
    if regions['Front'] > 1.0 and rospy.get_time() < 28.75:
        if regions['Left1'] <3.0 and regions['Left2'] < 1.9:
            if regions['Left3'] > 1.0 and regions['Left1'] <3.0 and regions['Left2'] < 1.9  :
                velocity_msg.linear.x = 1.0
                velocity_msg.angular.z = -0.5
                pub.publish(velocity_msg)
            elif regions['Left3'] > 0.60 :
                err= abs(regions['Left3']-0.60)
                velocity_msg.linear.x = x
                velocity_msg.angular.z = z + pid(err,kp,kd)
                pub.publish(velocity_msg)
                
            elif regions['Left3'] < 0.57:
                err= abs(regions['Left3']-0.57)
                velocity_msg.linear.x = x
                velocity_msg.angular.z = z - pid(err,kp,kd)
                pub.publish(velocity_msg)
                
            else:
                velocity_msg.linear.x = x
                velocity_msg.angular.z = 0
                pub.publish(velocity_msg)
            

        elif regions['Left1'] >2.5 or regions['Left2'] > 1.9  :

            if regions['Left3'] > 1.0:

                velocity_msg.linear.x = 0.5
                velocity_msg.angular.z = turn
                pub.publish(velocity_msg)

            elif regions['Left3'] > 0.60:
                err= abs(regions['Left3']-0.60)

                velocity_msg.linear.x = x
                velocity_msg.angular.z = z + pid(err,kp,kd)
                pub.publish(velocity_msg)
                
            elif regions['Left3'] < 0.57:
                err= abs(regions['Left3']-0.57)
                velocity_msg.linear.x = x
                velocity_msg.angular.z = z - pid(err,kp,kd)
                pub.publish(velocity_msg)
              
            else:
                velocity_msg.linear.x = x
                velocity_msg.angular.z = 0.0
                pub.publish(velocity_msg)
        else:
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.0
            pub.publish(velocity_msg)
    elif regions["Left3"] > 4.0:
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0
        pub.publish(velocity_msg)
    
    #print(velocity_msg)

    

def control_loop():
    rospy.init_node('ebot_controller')
    
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)

    
    rospy.spin()

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
