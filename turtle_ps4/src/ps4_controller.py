#!/usr/bin/env python
from pickle import FALSE, TRUE
from turtle import clear
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, SetPenRequest
from m2_ps4.msg import Ps4Data
from std_srvs.srv import *

# hint: some imports are missing

old_data = Ps4Data()
vel = Twist()

cur_linear_lvl=3
cur_angular_lvl=3
def callback(data):
    print("hello")


    global old_data
    global vel
    global cur_linear_lvl
    global cur_angular_lvl
    
    
    vel.linear.x = 0
    vel.linear.y = 0 
    vel.linear.z = 0
    
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0
    

    if():
        srv_clear(EmptyRequest())
    # you should publish the velocity here!
    # rospy.loginfo("linear:\n X:%f\n Y:%f\n Z:%f\nAngular:\n X:%f\n Y:%f\n Z:%f\n---"%(vel.linear.x, vel.linear.y, vel.linear.z, vel.angular.x, vel.angular.y, vel.angular.z))
    print(vel,flush =True)
    
    # hint: to detect a button being pressed, you can use the following pseudocode:
    # 
    # if ((data.button is pressed) and (old_data.button not pressed)),
    # then do something...

    # changing the velocity and the direction
    if (data.hat_ly != old_data.hat_ly):
        vel.linear.x = data.hat_ly * cur_linear_lvl
    if (data.hat_ly == old_data.hat_ly):
        vel.linear.x = data.hat_ly * cur_linear_lvl
    if (data.hat_lx != old_data.hat_lx):
        vel.angular.z = data.hat_lx * cur_angular_lvl
    if (data.hat_lx == old_data.hat_lx):
        vel.angular.z = data.hat_lx * cur_angular_lvl

    # changing the velocity scale(5 lvl)
    if (data.dpad_y == 1) and (old_data.dpad_y == 0):
        if (cur_linear_lvl >= 3 and cur_linear_lvl < 7):
            cur_linear_lvl += 1
    if (data.dpad_y == -1) and (old_data.dpad_y == 0):
        if (cur_linear_lvl >3 and cur_linear_lvl <= 7):
            cur_linear_lvl -= 1
    if (data.dpad_x == 1) and (old_data.dpad_x == 0):
        if (cur_angular_lvl >= 3 and cur_angular_lvl < 7):
            cur_angular_lvl += 1
    if (data.dpad_y == -1) and (old_data.dpad_y == 0):
        if (cur_angular_lvl > 3 and cur_angular_lvl <= 7):
            cur_angular_lvl -= 1

    # clear background
    if (data.ps == True) and (old_data.ps == False):
        srv_clear(EmptyRequest())

    


    # Set pen
    if(data.triangle == True) and (old_data.triangle == False):
        set_pen_req = SetPenRequest()
        set_pen_req.r = 0
        set_pen_req.g = 255
        set_pen_req.b = 0
        srv_col(set_pen_req)
    if(data.circle == True) and (old_data.circle == False):
        set_pen_req = SetPenRequest()
        set_pen_req.r = 255
        set_pen_req.g = 0
        set_pen_req.b = 0
        srv_col(set_pen_req)
    if(data.cross == True) and (old_data.cross== False):
        set_pen_req = SetPenRequest()
        set_pen_req.r = 0
        set_pen_req.g = 0
        set_pen_req.b = 255
        srv_col(set_pen_req)
    if(data.square == True) and (old_data.square == False):
        set_pen_req = SetPenRequest()
        set_pen_req.r = 128
        set_pen_req.g = 0
        set_pen_req.b = 128
        srv_col(set_pen_req)
    old_data = data
    pub.publish(vel)

if __name__ == '__main__':
    
    rospy.init_node('ps4_controller')

    # publisher object goes here... hint: the topic type is Twist
    pub = rospy.Publisher('turtle1/cmd_vel',Twist,queue_size = 1)
    # subscriber object goes here
    sub = rospy.Subscriber('input/ps4_data',Ps4Data,callback)

    
    # one service object is needed for each service called!
    # DO LATER!
    srv_col = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
    srv_clear = rospy.ServiceProxy('/clear', Empty)
    # fill in the other service client object...
    
    rospy.spin()