#!/usr/bin/env python

#HOW TO USE THE CONTROLLER
# Publish Point messages to /yawto_cmd_body
# where x=0,y=0,z=yaw angle in degrees is the position you want to move to relative to the body frame (in degrees)
#positive is to the left (right hand rule) z axis is up

#!!!!!!!!!!!!!!!! Takeoff command is not handled here


import rospy
import time
import numpy as np
from nav_msgs.msg import Odometry # We need this message type to read position and attitude from Bebop nav_msgs/Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

error_integral=0.
error=1000.

global_command= Twist()
global_setpoint=0.
global_current_yaw=0.
global_expected_yaw=0.

initial_cmd=Point()


islanded=False


pub_commands= rospy.Publisher('bebop/cmd_vel',Twist,queue_size=1)
pub_land= rospy.Publisher('bebop/land',Empty,queue_size=1) #trigger with pub_land.publish()




def quat_mult(a,b):
    
    c = np.array([0.,0.,0.,0.])
    c[0] = (a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3] )
    c[1] = (a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2] )
    c[2] = (a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1] )
    c[3] = (a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0] )
    return c

def odomlistener():

    global global_pos
    global islanded

    # set initial command to 0,0,0 body frame
    update_setpoint(initial_cmd)

    rospy.init_node('moveto_controller', anonymous=True, log_level=rospy.WARN)
    # Update the odometry value as it comes in from the Bebop
    rospy.Subscriber('/bebop/odom', Odometry, updateOdom)
    # Update land boolean to disable the controller if land is commanded
    rospy.Subscriber('/bebop/land',Empty, setlanded)
    # when a body frame command is recieved, update the setpoint
    rospy.Subscriber('/yawto_cmd_body',Point, update_setpoint)

    print('node started')
    time.sleep(1.)
    islanded=False

    # set telemetry rate to 5 hz, same as the odometry update 
    telemrate = 5
    rate = rospy.Rate(telemrate)
    # Run the controller to track to setpoint
    while not rospy.is_shutdown():
        yawto_body() # Runs the controller at set frequency hz
        rate.sleep()    

def updateOdom(msg):
    global global_current_yaw
    rospy.loginfo(msg.pose.pose)
    rospy.loginfo(msg.twist.twist)
    q=msg.pose.pose.orientation
    global_current_yaw = np.arctan2( 2*(q.w*q.z + q.x*q.y), 1-2*( q.y*q.y + q.z*q.z) )*(180/3.14156) #in deg

def setlanded(msg):
    global islanded
    islanded=True
    pub_land.publish()

def update_setpoint(data):
    global global_setpoint
    global global_expected_yaw
    global error
    global islanded


    if islanded==False:
        global_setpoint=data.z
        global_expected_yaw=global_current_yaw+global_setpoint

        if global_expected_yaw<-180:
            global_expected_yaw=global_expected_yaw+360
        if global_expected_yaw>180:
            global_expected_yaw=global_expected_yaw-360


        error=global_setpoint
    else:
        x=0
        y=0
        z=0
        print('IGNORING DUE TO LANDED')

def yawto_body():
    global global_current_yaw
    global global_expected_yaw
    global global_command
    global islanded
    global global_setpoint
    global error_integral
    global error


    acceptable_error=1. #degrees
    # adaptive_threshold=.35

    if islanded==False:
        
        print('Running right now with error: ',error)

        if abs(error)>acceptable_error and islanded==False:
            print '---------------------'


            error= global_expected_yaw-global_current_yaw

            if error>180:
                error= -(360-error)
            if error<-180:
                error= (360+error)

            error_integral=error_integral+error

            command = .008*error + .00003*error_integral

            print('error: ',error,'  error_integral: ',error_integral,'  cmd: ',command)
            print('current: ',global_current_yaw,' expected: ', global_expected_yaw)

            
            global_command.linear.x=0
            global_command.linear.y=0
            global_command.linear.z=0
            global_command.angular.x=0
            global_command.angular.y=0
            global_command.angular.z=command



            pub_commands.publish(global_command)

            error= global_expected_yaw-global_current_yaw

            if error>180:
                error= -(360-error)
            if error<-180:
                error= (360+error)

    else:
        print('Landed')
        pub_land.publish()
        islanded=True


if __name__ == '__main__':

    odomlistener()
    # rospy.spin()