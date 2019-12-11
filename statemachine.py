import rospy
import roslaunch
import time
import os
import subprocess
import numpy as np


############################################## STATE MACHINE ######################################################


def init():
## Initialize all the inputs required. (Cameras and Odom)
	
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)

	# Bebop and both cameras
	cli_args1 = ['bebop_driver', 'bebop_node.launch']
	cli_args2 = ['dqmc', 'justcamera.launch']
	battery = ['dqmc', 'batterycheck.launch']

	# roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
	# roslaunch_args1 = cli_args1[2:]

	roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
	roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)
	roslaunch_file3 = roslaunch.rlutil.resolve_launch_arguments(battery)

	launch_files = [roslaunch_file1, roslaunch_file2, roslaunch_file3]

	parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

	parent.start()

	return 0

def runscript(input_file, flag=1):

    process = subprocess.Popen(input_file, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

    if flag==1:
	    while True:
        # output = process.stdout.readline()
	        if process.poll() is not None:
	            break


def main():

	runscript("./test3.py",0)
	runscript("./test1.py")
	runscript("./test2.py")
	# runscript("stage12.py")
	# runscript("stage3.py")
	# runscript("stage4.py")
	# runscript("stage5.py")
	# runscript("stage6.py")

	return 0

if __name__ == '__main__':

	# init()
	main()