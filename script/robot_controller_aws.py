#! /usr/bin/python

# rospy for the subscriber
import roslib; #roslib.load_manifest('robot_control')
import rospy

# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension

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
rospy.init_node('robot_controller_aws')
    
port_name = rospy.get_param('~port','/dev/ttyUSB0')

# Number of seconds of sync failure after which Arduino is auto-reset.
# 0 = no timeout, auto-reset disabled
auto_reset_timeout = int(rospy.get_param('~auto_reset_timeout','0'))

# for systems where pyserial yields errors in the fcntl.ioctl(self.fd, TIOCMBIS, \
# TIOCM_DTR_str) line, which causes an IOError, when using simulated port
fix_pyserial_for_test = rospy.get_param('~fix_pyserial_for_test', False)


# TODO: do we really want command line params in addition to parameter server params?
sys.argv = rospy.myargv(argv=sys.argv)
if len(sys.argv) >= 2 :
    port_name  = sys.argv[1]

robot_list = ['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyUSB4','/dev/ttyUSB5','/dev/ttyUSB6','/dev/ttyUSB7','/dev/ttyUSB8','/dev/ttyUSB9']
robot_number = robot_list.index(port_name)


desire_pos_temp = [[200,270,math.radians(45)],[200,420,math.radians(45)],[460,420,math.radians(45)],[460,270,math.radians(45)]]
desire_pos = desire_pos_temp[robot_number]

pos_sensor_information =np.zeros((1,6))
goal_position_information = np.zeros((1,3))


def pos_sensor_information_callback(msg):
    global pos_sensor_information
    pos_sensor_information = msg.data
    
    #pos_sensor_information[0]  #robot_x
    #pos_sensor_information[1]  #robot_y
    #pos_sensor_information[2]  #robot_th
    #pos_sensor_information[3]  #robot_prox_left
    #pos_sensor_information[4]  #robot_prox_right
    #pos_sensor_information[5]  #robot_FSR

def goal_position_callback(msg):
    global goal_position_information 
    goal_position_information = msg.data


def main():
    global pos_sensor_information
    global goal_position_information 

    global position_control_data
    global dt

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    #############################################################
    # Subscriber Parts
    #############################################################
        
    # Define your image topic <- from video streaming
    # Set up your subscriber and define its callback
    motor_vel_information_topic = "/hamster"+str(robot_number)+"/pos_sensor_data"
    rospy.Subscriber(motor_vel_information_topic, Int32MultiArray, pos_sensor_information_callback)
   
    goal_position_topic = "/hamster"+str(robot_number)+"/goal_position"
    rospy.Subscriber(goal_position_topic, Int32MultiArray, goal_position_callback)
       
    #############################################################
    # Publisher Parts
    #############################################################
    pub = rospy.Publisher("/hamster"+str(robot_number)+"/vel_control", Int32MultiArray, queue_size=10)

    r = rospy.Rate(10) # 10hz
    goal_position_information = [150, 150, 45]

    while not rospy.is_shutdown():
        if len(pos_sensor_information) < 2:
            pass 
        else:
            robot_current_position = [pos_sensor_information[0],pos_sensor_information[1],pos_sensor_information[2]]
            robot_goal_position = [goal_position_information[0], goal_position_information[1], goal_position_information[2]]
            robot_sensors = [pos_sensor_information[3],pos_sensor_information[4],pos_sensor_information[5],pos_sensor_information[6]]

            position_control_data =  robot_controller(robot_current_position, robot_goal_position, robot_sensors) 

            pub.publish(Int32MultiArray(data=position_control_data))

        last_time = current_time
        r.sleep()
    
    #############################################################
    # Running ROS
    #############################################################
    rospy.spin()        # Spin until ctrl + c


def robot_controller(start_pos, target_pos, robot_sensors):
    #start_pos = [Robot_current_position_X, Robot_current_position_Y, Robot_current_heading_angle]
    #target_pos = [Robot_goal_position_X, Robot_goal_position_Y, Robot_goal_heading_andgle]
    #robot_sensors = [Left_proximity, Right_proximity, Left_ground, Right_ground]

    global pos_sensor_information
    maximum_motor_speed = 100

    d = 20 # unit : mm  

    #P controller gains
    k_rho = 0.2               # default 0.05   #should be larger than 0, i.e, k_rho > 0
    k_alpha = 1               # k_alpha - k_rho > 0
    k_beta = -0.01            # should be smaller than 0, i.e, k_beta < 0

    stop_range = 10

    #print target_pos, start_pos
    delta_x = target_pos[0] - start_pos[0]
    delta_y = target_pos[1] - start_pos[1]
   
    rho = math.sqrt(math.pow(delta_x,2)+math.pow(delta_y,2)) 

    # distance between targets and current position    
    if rho < stop_range :
        #position_control_data = [0,0]
        vL = 0
        vR = 0
    else:    
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

        # avoid the obstacles using proximity sensors        
        if robot_sensors[0] > 30:
            vL += 0.5*maximum_motor_speed
            vR -= 0.5*maximum_motor_speed
        elif robot_sensors[1] > 30:
            vL -= 0.5*maximum_motor_speed
            vR += 0.5*maximum_motor_speed
        elif robot_sensors[1] > 30 and robot_sensors[1] > 30:
            vL -= 0.5*maximum_motor_speed
            vR -= 0.5*maximum_motor_speed

        #print v + d/2*w,v - d/2*w
        if vL > 100:    vL = 100
        elif vR > 100:  vR = 100
        if vL < -100:   vL = -100
        elif vR < -100: vR = -100    
            
    return [vL,vR]

        
if __name__ == '__main__':
    main()

