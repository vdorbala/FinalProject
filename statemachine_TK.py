#!/usr/bin/env python
import rospy
import roslaunch
import time
import os
import subprocess
import numpy as np
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Empty
# from bondpy import bondpy
 

############################################## STATE MACHINE ######################################################


# def init():
# ## Initialize all the inputs required. (Cameras and Odom)
    
#   uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#   roslaunch.configure_logging(uuid)

#   # Bebop and both cameras
#   cli_args1 = ['bebop_driver', 'bebop_node.launch']
#   cli_args2 = ['dqmc', 'justcamera.launch']
#   battery = ['dqmc', 'batterycheck.launch']

#   # roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
#   # roslaunch_args1 = cli_args1[2:]

#   roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
#   roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)
#   roslaunch_file3 = roslaunch.rlutil.resolve_launch_arguments(battery)

#   launch_files = [roslaunch_file1, roslaunch_file2, roslaunch_file3]

#   parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

#   parent.start()

#   return 0

pub_takeoff= rospy.Publisher('/bebop/takeoff',Empty,queue_size=1)

def runscript(input_file, flag=1):

    process = subprocess.Popen(input_file, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

    if flag==1:
        while True:
            output = process.stdout.readline()
            print(output)
            if process.poll() is not None:
                break

def main():

    runscript("/home/qdmc/bebop_ws/src/dqmc/scripts/MoveToController.py",0)

    runscript("/home/qdmc/bebop_ws/src/dqmc/scripts/Window_and_wall.py")

    print(" Passed Window SUCCESSFULLY! \n Finished Stage 1 and stage 2!")

    # print('Taking off') #i think the node was too speedy or something, ignore this takeoff mess its just a hack for testing
    # pub_takeoff.publish()
    # print('huh?')
    # time.sleep(5.)
    # print('Taking off try 2')
    # pub_takeoff.publish()
    # print('huh?')
    # time.sleep(5.)
    time.sleep(8)

    # #### Move 3, Yaw right (-ve)
    command.x = 0
    command.y = 0
    command.z = 0
    command.w = -45 # Yaw Command, positive left
    # # SEND IT
    print('sending yaw command, Yaw towards the bridge: ',command)
    command_pub.publish(command)
    # wait for it
    time.sleep(4)


    

    print("STARTING BRIDGE SCRIPT")

    runscript("/home/qdmc/bebop_ws/src/dqmc/scripts/Bridge.py")

    print("CROSSED BRIDGE SUCCESSFULLY! \n STAGE 3 COMPLETE!")



    time.sleep(3)

    # #### Move 3, Yaw right (-ve)
    command.x = 0
    command.y = -1
    command.z = -.5
    command.w = 0 # Yaw Command, positive left
    # # SEND IT
    print('sending transition command, move towards the target: ',command)
    command_pub.publish(command)
    # wait for it
    time.sleep(5)



   
    # print("FINISHED YAWING NOW!")

    # time.sleep(4.)
    # pub_takeoff.publish()
    # time.sleep(4.)

    runscript("/home/qdmc/bebop_ws/src/dqmc/scripts/circles_detect_ros.py")

    print("STAGE 4 COMPLETE")

    # time.sleep(5.)

    # command.w = 0
    # command.x = 0
    # command.y = 0
    # command.z = 0
    # command_pub.publish(command)

    time.sleep(3.)

    pub_takeoff.publish()
    time.sleep(3.)
    
    # ### KILLL COMMAND! rospy.signal_shutdown('WOOOOOOOOOOOOOOOHOOOOO')
    runscript("/home/qdmc/bebop_ws/src/dqmc/scripts/FindFeet_Controller_V2.py")

    print("STAGE 5 COMPLETE")
        
    runscript("/home/qdmc/bebop_ws/src/dqmc/scripts/circles_detect_ros_no_yaw.py")
    
    print("STAGE 6 COMPLETE")

    # runscript("stage12.py")
    # runscript("stage3.py")
    # runscript("stage4.py")
    # runscript("stage5.py")
    # runscript("stage6.py")

    return 0

if __name__ == '__main__':
    
    rospy.init_node('ALLNODE')
    
    pub_takeoff= rospy.Publisher('/bebop/takeoff',Empty,queue_size=1)
    command_pub = rospy.Publisher('/moveto_cmd_body', Quaternion, queue_size=1)

    command=Quaternion()

    path = os.getcwd()
    # init()
    main()
