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
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

pub_commands= rospy.Publisher('/moveto_cmd_body',Quaternion,queue_size=1)
pub_yaw= rospy.Publisher('/yawto_cmd_body',Point,queue_size=1)
pub_takeoff= rospy.Publisher('/bebop/takeoff',Empty,queue_size=1)


def tester():
    
    print('starting node')

    rospy.init_node('moveto_tester', anonymous=True, log_level=rospy.WARN)
    command=Quaternion()

    # yawcmd=Point()
    
    print('Taking off') #i think the node was too speedy or something, ignore this takeoff mess its just a hack for testing
    pub_takeoff.publish()
    print('huh?')
    time.sleep(5.)
    print('Taking off try 2')
    pub_takeoff.publish()
    print('huh?')
    time.sleep(5.)

    once=0

    #HOW TO USE THE CONTROLLER
    # Publish Quarternion messages to /moveto_cmd_body
    # where x,y,z is the position you want to move to relative to the body frame (in meters)

    # w is how you indicate latching, if w=1 the command will be executed until completion and all else will be ignored
    # if w is anything else, the command is free to be updated mid execution


    while not rospy.is_shutdown():
        print 'publishing 1'
        command.x=1
        command.w=1
        pub_commands.publish(command)
        time.sleep(8.5)
        print 'publishing 2'
        command.x=-1
        command.w=90
        pub_commands.publish(command)
        time.sleep(3.5)
        print 'publishing 3'
        command.x=0
        command.y=1
        command.z=0
        command.w=0
        pub_commands.publish(command)
        time.sleep(1.0)
        print 'publishing 4'
        command.x=0
        command.y=-.5
        command.z=0
        command.w=0
        pub_commands.publish(command)
        time.sleep(1.0)

        print 'publishing 4'
        command.x=0
        command.y=-.5
        command.z=0
        command.w=90
        pub_commands.publish(command)
        time.sleep(3.0)



        
        # if once==0:
        #     print 'publishing latch'
        #     once=1

        #     command.x=0
        #     command.y=-1
        #     command.w=1
        #     pub_commands.publish(command)
        #     time.sleep(1.5)
        #     command.w=0
        #     command.x=0
        #     command.y=0


        	


    

if __name__ == '__main__':

    tester()
    # rospy.spin()
