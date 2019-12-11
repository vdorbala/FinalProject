#!/usr/bin/env python


#HOW TO USE THE CONTROLLER
# Publish Quarternion messages to /moveto_cmd_body
# where x,y,z is the position you want to move to relative to the body frame (in meters)

# w is how you indicate latching, if w=1 the command will be executed until completion and all else will be ignored
# if w is anything else, the command is free to be updated mid execution

import rospy
import time
import numpy as np
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

pub_commands= rospy.Publisher('/moveto_cmd_body',Quaternion,queue_size=1)
pub_takeoff= rospy.Publisher('/bebop/takeoff',Empty,queue_size=1)
pub_land= rospy.Publisher('bebop/land',Empty,queue_size=1)

def hardcoded_Wall():
    global global_pos
    print('starting node')

    rospy.Subscriber('/bebop/odom', Odometry, writeOdom)
    rospy.init_node('moveto_tester', anonymous=True, log_level=rospy.WARN)
    command=Quaternion()
    time.sleep(1)

    print('Taking off') #i think the node was too speedy or something, ignore this takeoff mess its just a hack for testing
    pub_takeoff.publish()
    # print('huh?')
    # time.sleep(5.)
    # print('Taking off try 2')
    # pub_takeoff.publish()
    # print('huh?')
    # time.sleep(5.)
    time.sleep(1.5)

    #### Move 1: Line up with Wall 
    zcmd = 1.7 - global_pos.position.z # meters, global to body
    command.x = 0
    command.y = -0.2 # 0.2 meters right
    command.z = zcmd
    command.w = 0 # Latching disabled
    # SEND IT
    print('sending command 1: ',command)
    pub_commands.publish(command)

    time.sleep(5)
    #### THIS IS FOR INITIAL TESTING
    # pub_land.publish()


    


    #### Move 2, Cross the Wall, go fwd
    command.x = 2
    command.y = 0
    command.z = 0
    command.w = 1 # Latching enabled
    # SEND IT
    print('sending command 2 CROSS THE WALL: ',command)
    pub_commands.publish(command)
    # wait for it
    time.sleep(8)

    command.x = 0
    command.y = 0 # 0.2 meters right
    command.z = 0
    command.w = 0 # Latching disabled
    # SEND IT
    print('sending command 000: ',command)
    pub_commands.publish(command)


    #### Move 3, Yaw right
    command.x = 0
    command.y = 0
    command.z = 0
    command.w = -30 # Yaw Command, positive left
    # SEND IT
    print('sending command 3, Yaw towards the window: ',command)
    pub_commands.publish(command)
    # wait for it
    time.sleep(4)


    #### Move 4, Move initially to center on the window (no latching, let the window controller take over)
    command.x = 0
    command.y = 0
    command.z = 0
    command.w = 0 # no latching
    # SEND IT
    print('sending command 4, Initial centering: ',command)
    pub_commands.publish(command)
    # wait for it
    time.sleep(5)
    pub_land.publish()   

    # Now handoff to the window controller
    #rospy.signal_shutdown('Node is finished, shut down')

    #HOW TO USE THE CONTROLLER
    # Publish Quarternion messages to /moveto_cmd_body
    # where x,y,z is the position you want to move to relative to the body frame (in meters)

    # w is how you indicate latching, if w=1 the command will be executed until completion and all else will be ignored
    # if w is anything else, the command is free to be updated mid execution


def writeOdom(data):
    global global_pos
    global global_vel
    global_pos=data.pose.pose
    global_vel=data.twist.twist
    

if __name__ == '__main__':

    hardcoded_Wall()
    rospy.spin()