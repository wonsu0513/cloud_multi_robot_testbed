#! /usr/bin/python

# rospy for the subscriber
import roslib; #roslib.load_manifest('robot_control')
import rospy

# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from geometry_msgs.msg import Twist

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# Import User Libs
import numpy as np
import cv2
import imutils
import time
import math
import sys



#############################################################
# Node setting
#############################################################
rospy.init_node('robot_controller')
    
port_name = rospy.get_param('~robot_name','robot/hamster0')


# TODO: do we really want command line params in addition to parameter server params?
sys.argv = rospy.myargv(argv=sys.argv)
if len(sys.argv) >= 2 :
    port_name  = sys.argv[1]

robot_list = ['robot/hamster0','robot/hamster1','robot/hamster2','robot/hamster3','robot/hamster4','robot/hamster5','robot/hamster6','robot/hamster7','robot/hamster8','robot/hamster9']
robot_number = robot_list.index(port_name)

taget_pos_temp = [[325,215,math.radians(45)],[291,241,math.radians(45)],[323,268,math.radians(45)],[363,220,math.radians(45)],[396,240,math.radians(45)],[363,264,math.radians(45)]]

desire_pos_temp = [[120,50,math.radians(45)],[120,150,math.radians(45)],[120,250,math.radians(45)],[554,66,math.radians(45)],[554,165,math.radians(45)],[554,253,math.radians(45)]]

desire_pos = desire_pos_temp[robot_number]
#print desire_pos


d = 20 # unit : mm  
#P controller gains

#previous
k_rho = 0.2               # default 0.05   #should be larger than 0, i.e, k_rho > 0
k_alpha = 1               # k_alpha - k_rho > 0
k_beta = -0.01            # should be smaller than 0, i.e, k_beta < 0

#k_rho = 0.3               # default 0.05   #should be larger than 0, i.e, k_rho > 0
#k_alpha = 1.4               # k_alpha - k_rho > 0
#k_beta = -0.5            # should be smaller than 0, i.e, k_beta < 0

stop_range = 10



# Instantiate CvBridge
bridge = CvBridge()

robot_pos_information = np.zeros((5,5))


object_center_position = np.zeros((1,2))
path_information = []
pos_sensor_information =np.zeros((1,6))
position_control_data = np.zeros((1,2))
servo_value_data = 0
robot_selector_mode = 0

def robot_inforation_callback(msg):   
    global robot_pos_information
    try:
        robot_pos_information = bridge.imgmsg_to_cv2(msg,"passthrough")
    except CvBridgeError, e:
        print(e)
    else:
        #print robot_pos_information
        
        pass
        #print robot_pos_information[0]
        #print robot_pos_information[0][1]

def pos_sensor_information_callback(msg):
    global pos_sensor_information
    pos_sensor_information = msg.data
    
    #pos_sensor_information[0] #robot_x
    #pos_sensor_information[1]  #robot_y
    #pos_sensor_information[2]  #robot_th
    #pos_sensor_information[3]  #robot_prox_left
    #pos_sensor_information[4]  #robot_prox_right
    #pos_sensor_information[5]  #robot_FSR
   
def goal_position_information_callback(msg):
    global desire_pos
    desire_pos = msg.data

def robot_sw_poscallback(msg):
    global robot_selector_mode 
    robot_selector_mode= msg.data

def main():
    global position_control_data
    global pos_sensor_information
    global desire_pos
    global desire_pos_temp
    global robot_selector_mode
    #############################################################
    # Subscriber Parts
    #############################################################
    # Define your image topic
    robot_information_topic = "/robot_pos_information"
    rospy.Subscriber(robot_information_topic, Image, robot_inforation_callback)
    
    # Define your image topic
    # Set up your subscriber and define its callback
    	
    motor_vel_information_topic = "/hamster"+str(robot_number)+"/pos_sensor_data"
    rospy.Subscriber(motor_vel_information_topic, Int32MultiArray, pos_sensor_information_callback)
    
    goal_position_information_topic = "/hamster"+str(robot_number)+"/goal_position_data"
    rospy.Subscriber(goal_position_information_topic, Int32MultiArray, goal_position_information_callback)

    rospy.Subscriber("mode/sw",Int32,robot_sw_poscallback)

    #############################################################
    # Publisher Parts
    #############################################################
    pub = rospy.Publisher("/hamster"+str(robot_number)+"/position_control", Int32MultiArray, queue_size=10)
    #object_centermass = object_centermass
    
    
    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        # robot controller        
        if len(pos_sensor_information) < 2:
            #print "wonsu"
            pass 
        else:   
            if robot_selector_mode == 0:
                desire_pos = desire_pos_temp[robot_number] 
            elif robot_selector_mode == 3:
                desire_pos = taget_pos_temp[robot_number] 

            else:
                pass
            current_pos = [pos_sensor_information[0],pos_sensor_information[1],pos_sensor_information[2]]
            robot_controller(current_pos, desire_pos)
            pub.publish(Int32MultiArray(data=position_control_data))
        r.sleep()
    #############################################################
    # Running ROS
    #############################################################
    rospy.spin()        # Spin until ctrl + c

def robot_controller(start_pos, target_pos):
    global d
    #P controller gains
    global k_rho
    global k_alpha
    global k_beta 
    global stop_range
    global servo_value_data

    #output to control each motor
    global position_control_data

    #print target_pos, start_pos

    delta_x = target_pos[0] - start_pos[0]
    delta_y = target_pos[1] - start_pos[1]
    
    #print start_pos

    rho = math.sqrt(math.pow(delta_x,2)+math.pow(delta_y,2)) 
    #print rho

    # distance between targets and current position
    if rho < stop_range:
        #position_control_data = [0,0]
        vL = 0
        vR = 0
        #vL = -80
        #vR = +80
        servo_value_data = 0
        
    else:    
        servo_value_data = 90  
        alpha = -math.radians(start_pos[2])+math.atan2(delta_y,delta_x)
    
        # analge with traget and current position 
        if math.degrees(alpha) > 180:
            temp_alpha = math.degrees(alpha) - 360
            alpha = math.radians(temp_alpha)
        elif math.degrees(alpha) < -180:
            temp_alpha = math.degrees(alpha) + 360
            alpha = math.radians(temp_alpha)

        beta = -math.radians(start_pos[2])-alpha        # difference between angles 

        #print start_pos[2], alpha, beta

        v = k_rho*rho                       # P controller
        w = k_alpha*alpha + k_beta*beta

        #print v, w
        vL = np.around(v + d/2*w, 0)      # P controller
        vR = np.around(v - d/2*w, 0)      # P controller
        
        #print v + d/2*w,v - d/2*w
        if vL > 100:    vL = 100
        elif vR > 100:  vR = 100
        if vL < -100:   vL = -100
        elif vR < -100: vR = -100
    position_control_data = [vL,vR]

        
if __name__ == '__main__':
    main()