#!/usr/bin/env python

#HOW TO USE THE CONTROLLER
# Publish Quarternion messages to /moveto_cmd_body
# where x,y,z is the position you want to move to relative to the body frame (in meters)

# w is how you indicate latching, if w=1 the command will be executed until completion and all else will be ignored
# if w is anything else, the command is free to be updated mid execution

#IF W IS NOT 0 or 1 IT WILL YAW THAT AMOUNT, and reject all other commands until its done

#!!!!!!!!!!!!!!!! Takeoff command is not handled here


import rospy
import time
import numpy as np
from nav_msgs.msg import Odometry # We need this message type to read position and attitude from Bebop nav_msgs/Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Empty


adaptive_threshold=.35
acceptable_error=.09

error_integral=np.array([0.,0.,0.])
error=1000

global_pos= Pose()#np.array([0.,0.,0.])

global_command= Twist()
global_vel=Twist()
global_waypoint=Point()

global_marker_center=np.array([0.,0.])
global_marker_center_avg=np.array([0.,0.])

expected_pos_inertial= np.array([0.,0.,0.])
x=0
y=0
z=0

yaw_error=0
yaw_error_integral=0
global_expected_yaw=0
global_current_yaw=0

#make smoother transition into adaptive gain reigon
started_far=False

islanded=True


pub_commands= rospy.Publisher('bebop/cmd_vel',Twist,queue_size=1)
pub_takeoff= rospy.Publisher('bebop/takeoff',Empty,queue_size=1)
pub_land= rospy.Publisher('bebop/land',Empty,queue_size=1) #trigger with pub_land.publish()

followthrough=False

initial_cmd = Quaternion() # All 0



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
    rospy.Subscriber('/bebop/takeoff',Empty, settakeoff)
    # when a body frame command is recieved, update the setpoint
    rospy.Subscriber('/moveto_cmd_body',Quaternion, update_setpoint)

    print('node started')
    time.sleep(1.)
    islanded=False

    # set telemetry rate to 5 hz, same as the odometry update 
    telemrate = 5
    rate = rospy.Rate(telemrate)
    # Run the controller to track to setpoint
    while not rospy.is_shutdown():
        moveto_body() # Runs the controller at set frequency hz
        rate.sleep()    

def updateOdom(msg):
    global global_pos
    global global_vel
    global global_current_yaw

    rospy.loginfo(msg.pose.pose)
    rospy.loginfo(msg.twist.twist)
    global_pos=msg.pose.pose
    global_vel=msg.twist.twist

    q=msg.pose.pose.orientation
    global_current_yaw = np.arctan2( 2*(q.w*q.z + q.x*q.y), 1-2*( q.y*q.y + q.z*q.z) )*(180/3.14156) #in deg

def settakeoff(msg):
    global islanded
    islanded=False
    

def setlanded(msg):
    global islanded
    islanded=True
    # pub_land.publish()

def update_setpoint(data):
    global global_pos
    global global_waypoint
    global expected_pos_inertial
    global x
    global y
    global z
    global error
    global islanded
    global followthrough
    global error_integral

    global global_expected_yaw
    global yaw_error
    global yaw_error_integral

    global started_far
    global adaptive_threshold
    global acceptable_error


    if islanded==False:
        
        if np.linalg.norm([data.x,data.y,data.z]) < 10:
            if followthrough==False:


                if data.w==0 or data.w==1:
                    #okay we do a waypoint move

                    if data.w==1:
                        followthrough=True



                    x = data.x # Positive fwd, body frame
                    y = data.y # Positive left, body frame
                    z = data.z # positive up, body frame
                    # IF Z WAS IN INERTIAL FRAME, RUN THE NEXT LINE
                    #    # transpose intertial coord z to body frame
                    #z = z - global_pos.position.z

                    #okay so have a vector in the actual body frame, want to convert to inertial/odom frame
                    quat_B_to_I= np.array([global_pos.orientation.w, -global_pos.orientation.x, -global_pos.orientation.y, -global_pos.orientation.z])
                    quat_B_to_I_inv= np.array([global_pos.orientation.w, global_pos.orientation.x, global_pos.orientation.y, global_pos.orientation.z])
                    command_quat_body=np.array([0, x,y,z])
                    temp= quat_mult(quat_B_to_I_inv,command_quat_body)
                    command_quat_inertial= quat_mult(temp,quat_B_to_I)
                    command_vect_inertial= command_quat_inertial[1:]

                    #okay now have command in the odom frame, relative to body, so make it relative to origin
                    # Update the setpoint in global coordinates below
                    expected_pos_inertial= np.array([global_pos.position.x, global_pos.position.y, global_pos.position.z]) + command_vect_inertial
                    error= np.linalg.norm(command_vect_inertial)


                    if error>adaptive_threshold:
                        started_far=True

                    if error<acceptable_error: #if youre asked to just stop
                        error=10

                    error_integral=np.array([0.,0.,0.])

                    #publish it for RVIZ here if wanted
                    # global_waypoint.x = expected_pos_inertial[0]
                    # global_waypoint.y = expected_pos_inertial[1]
                    # global_waypoint.z = expected_pos_inertial[2]
                    # pub_waypoints.publish(global_waypoint)

                    # Debugging, display body frame command
                    print('\n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n') #clears the screen basically
                    print('Setpoint Updated (body frame): ',np.array([x,y,z,data.w]))
                else:
                    #okay we yaw only
                    followthrough=True

                    print('\n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n') #clears the screen basically
                    print('Setpoint Updated (body frame): ',np.array([x,y,z,data.w]))

                    global_expected_yaw=global_current_yaw+data.w

                    if global_expected_yaw<-180:
                        global_expected_yaw=global_expected_yaw+360
                    if global_expected_yaw>180:
                        global_expected_yaw=global_expected_yaw-360

                    yaw_error= data.w
                    yaw_error_integral=0

            else:
                print('IGNORING due to FOLLOWTHROUGH last command')
        else:
            print('FUCK YOU THATS TOO FAR')
    else:


        x=0
        y=0
        z=0
        print('IGNORING DUE TO LANDED')


def moveto_body():
    global global_pos
    global global_vel
    global global_command
    global expected_pos_inertial
    global islanded
    global global_waypoint
    global error_integral
    global error
    global followthrough

    global global_current_yaw
    global global_expected_yaw
    global yaw_error_integral
    global yaw_error

    global adaptive_threshold
    global started_far

    global acceptable_error


    


    acceptable_yaw_error=2

    move_array=np.array([0.,0.,0.])

    if islanded==False:
        
        

        if yaw_error==0:
            print('Running right now with error: ',error)

            error= np.linalg.norm(np.array([global_pos.position.x, global_pos.position.y, global_pos.position.z])-expected_pos_inertial)

            if error<acceptable_error and followthrough==True:
                print 'Finished latched command'
                followthrough=False

            if error>acceptable_error and islanded==False and expected_pos_inertial[2]>.2:

                
                current_pos_inertial=np.array([global_pos.position.x, global_pos.position.y, global_pos.position.z])
                move_vect_inertial= expected_pos_inertial-current_pos_inertial

                #okay need to convert to vector in body frame to figure out where to move
                #using current orientation global_pos in case something is flukey
                quat_I_to_B= np.array([global_pos.orientation.w, global_pos.orientation.x, global_pos.orientation.y, global_pos.orientation.z])
                quat_I_to_B_inv= np.array([quat_I_to_B[0], -quat_I_to_B[1], -quat_I_to_B[2],-quat_I_to_B[3]])
                
                move_quat_inertial= np.array([0, move_vect_inertial[0],  move_vect_inertial[1], move_vect_inertial[2]])
                temp= quat_mult(quat_I_to_B_inv,move_quat_inertial)
                move_quat_body= quat_mult(temp,quat_I_to_B)
                move_vect_body= move_quat_body[1:]# this is basically your error vector

                velocity_vect_body= np.array([global_vel.linear.x, global_vel.linear.y, global_vel.linear.z])
                error_integral=error_integral+move_vect_body


                move_vect_body=np.array(move_vect_body)
                velocity_vect_body=np.array(velocity_vect_body)
                error_integral=np.array(error_integral)

                #move_vect_body[2]=1.29*move_vect_body[2]


                #ADAPTIVE GAINS FOR WHEN NEARBY
                print('error-------------------------------------------------------------------------')
                print(error)
                print(' ')

                if error<adaptive_threshold:
                    print '----------adaptive gains active------------'
                    
                    if started_far:
                        print 'RESETING INTEGRAL'
                        error_integral= np.array([0.,0.,0.]) #not completely 0 
                        started_far=False

                    print('move_vect_body: ',move_vect_body)
                    print('velocity: ',velocity_vect_body)
                    print('error_integral: ',error_integral)
                    print(' ')

                    move_array[0]=.14*move_vect_body[0] - .22*velocity_vect_body[0] + .0011*error_integral[0] #TUNE THIS
                    move_array[1]=.14*move_vect_body[1] - .22*velocity_vect_body[1] + .0011*error_integral[1]
                    move_array[2]=.63*move_vect_body[2] - .10*velocity_vect_body[2] + .0011*error_integral[2]
                else:

                    print('move_vect_body: ',move_vect_body)
                    print('velocity: ',velocity_vect_body)
                    print('error_integral: ',error_integral)
                    print(' ')



                    move_array[0]=.08*move_vect_body[0] - .19*velocity_vect_body[0] + .0006*error_integral[0] #TUNE THIS
                    move_array[1]=.08*move_vect_body[1] - .19*velocity_vect_body[1] + .0006*error_integral[1]
                    move_array[2]=.53*move_vect_body[2] - .10*velocity_vect_body[2] + .001*error_integral[2]

                
                
                #timedelay= .1#TUNE THIS
                print('move vect: ', move_array)
                print(' ')
                print('command is')
                print(np.array([x,y,z]))
                print('expected_pos_inertial')
                print(expected_pos_inertial)
                print('current_pos_inertial')
                print(current_pos_inertial)



                global_command.linear.x=move_array[0]
                global_command.linear.y=move_array[1]
                global_command.linear.z=move_array[2]
                global_command.angular.x=0
                global_command.angular.y=0
                global_command.angular.z=0



                pub_commands.publish(global_command)
                # error= np.linalg.norm(np.array([global_pos.position.x, global_pos.position.y, global_pos.position.z])-expected_pos_inertial)



        else:
            print('YAW Running right now with error: ',yaw_error)

            if abs(yaw_error)>acceptable_yaw_error and islanded==False:
                print '---------------------'


                yaw_error= global_expected_yaw-global_current_yaw

                if yaw_error>180:
                    yaw_error= -(360-yaw_error)
                if yaw_error<-180:
                    yaw_error= (360+yaw_error)

                yaw_error_integral=yaw_error_integral+yaw_error

                command = .005*yaw_error + .00003*yaw_error_integral

                print('yaw error: ',yaw_error,' yaw  error_integral: ',yaw_error_integral,'  cmd: ',command)
                print('current: ',global_current_yaw,' expected: ', global_expected_yaw)

                
                global_command.linear.x=0
                global_command.linear.y=0
                global_command.linear.z=0
                global_command.angular.x=0
                global_command.angular.y=0
                global_command.angular.z=command



                pub_commands.publish(global_command)

                yaw_error= global_expected_yaw-global_current_yaw

                if yaw_error>180:
                    yaw_error= -(360-yaw_error)
                if yaw_error<-180:
                    yaw_error= (360+yaw_error)

            if abs(yaw_error)<acceptable_yaw_error:
                print 'finished yaw'
                followthrough=False
                yaw_error=0
                global_command.linear.x=0
                global_command.linear.y=0
                global_command.linear.z=0
                global_command.angular.x=0
                global_command.angular.y=0
                global_command.angular.z=0
                pub_commands.publish(global_command)


    else:
        print('Landed')
        # pub_land.publish()
        # islanded=True




if __name__ == '__main__':

    odomlistener()
    # rospy.spin()