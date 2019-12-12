#!/usr/bin/env python

#testttt2

import numpy as np
import rospy
import cv2
import os
import Tkinter, tkFileDialog
# from time import time #laptop version
import time #upboard version
import imutils
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
img_pub = rospy.Publisher("/window_mask",Image,queue_size=1)
command_pub = rospy.Publisher("/moveto_cmd_body",Quaternion,queue_size=1)
yaw_pub= rospy.Publisher('/yawto_cmd_body',Point,queue_size=1)
pub_takeoff= rospy.Publisher('/bebop/takeoff',Empty,queue_size=1)
pub_land= rospy.Publisher('bebop/land',Empty,queue_size=1)

points_3d = np.float32([[0.0, 0.0, 0.0], [-390, -215, 0.0], [390, -215, 0.0], [420, 215, 0.0], [-420, 215, 0.0]]) 
intrinsics = np.array([345.1095082193839, 344.513136922481, 315.6223488316934, 238.99403696680216]) #mm
dist_coeff = np.array([-0.3232637683425793, 0.045757813401817116, 0.0024085161807053074, 0.003826902574202108])
K = np.float64([ [intrinsics[0], 0.0, intrinsics[2]], [0.0, intrinsics[1], intrinsics[3]], [0.0, 0.0, 1.0]])
GMMin = './YellowGMM3_100.npz'
thresh = .3#1e-4#1e-5

npzfile = np.load(GMMin)
k = npzfile['arr_0']
mean = npzfile['arr_1']
cov = npzfile['arr_2']
pi = npzfile['arr_3']

#matrix from body frame to camera frame:
cRb = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
bRc = np.transpose(cRb)

#TUNABLE PARAM:############################################################################
#want to be 1 meter in front of window before shooting the shot so
# vdes_ini = np.array([[0],[0],[1000]])

#little hax for better line up
#ask for slightly left since approaching from right
#ask for slightly low since camera on bottom of quad
vdes_ini = np.array([[-40],[-40],[1000]])

pause_active=False
pauselength=2.4 #seconds

pause_start_time=0

######################################################################################

#TODO:
#add actual commands sent to controller. currently just calcs commands
global_command=Quaternion()

global_wallisdone=False

global_first_window_move=True

global_lastmags = np.ones((1,5))*20000

#add confidence metric that increases when you find the same window over and over and lowers when you dont find shit
#theres some false window positives in the bag, that *shouldnt* be a problem but this will help with that
#make it so you only execute commands if your confidence value is high




def hardcoded_Wall():
    global global_wallisdone
    print('starting wall')

    command=Quaternion()

    

    #### Move 1: Line up with Wall 
    # zcmd = 1.7 - global_pos.position.z # meters, global to body
    zcmd = .8
    command.x = 0
    command.y = -0.2 # 0.2 meters right
    command.z = zcmd
    command.w = 0 # Latching disabled
    # SEND IT
    print('sending command 1: ',command)
    command_pub.publish(command)

    time.sleep(5)
    #### THIS IS FOR INITIAL TESTING
    # pub_land.publish()


    


    #### Move 2, Cross the Wall, go fwd
    command.x = 1.8
    command.y = 0
    command.z = 0
    command.w = 1 # Latching enabled
    # SEND IT
    print('sending command 2 CROSS THE WALL: ',command)
    command_pub.publish(command)
    # wait for it
    time.sleep(8)

    command.x = 0
    command.y = 0 # 0.2 meters right
    command.z = 0
    command.w = 0 # Latching disabled
    # SEND IT
    print('sending command 000: ',command)
    command_pub.publish(command)


    #### Move 3, Yaw right
    command.x = 0
    command.y = 0
    command.z = 0
    command.w = -30 # Yaw Command, positive left
    # SEND IT
    print('sending command 3, Yaw towards the window: ',command)
    command_pub.publish(command)
    # wait for it
    time.sleep(4)


    #### Move 4, Move initially to center on the window (no latching, let the window controller take over)
    command.x = 0
    command.y = 0
    command.z = 0
    command.w = 0 # no latching
    # SEND IT
    print('sending command 4, Initial centering: ',command)
    command_pub.publish(command)
    # wait for it
    # time.sleep(1)
    # pub_land.publish()   

    global_wallisdone=True


    # Now handoff to the window controller
    #rospy.signal_shutdown('Node is finished, shut down')

    #HOW TO USE THE CONTROLLER
    # Publish Quarternion messages to /moveto_cmd_body
    # where x,y,z is the position you want to move to relative to the body frame (in meters)

    # w is how you indicate latching, if w=1 the command will be executed until completion and all else will be ignored
    # if w is anything else, the command is free to be updated mid execution



def applyCorners2Mask(mask,img):

	center, inner_corners, outer_corners = np.zeros((1,2)),np.zeros((4,2)),np.zeros((4,2))


	gray=mask.copy()
	gray = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)[1]
	gray = cv2.dilate(gray,np.ones((3,3), np.uint8),iterations=4)
	gray = cv2.erode(gray,np.ones((3,3), np.uint8),iterations=3)
	#gray = cv2.dilate(gray,np.ones((5,5), np.uint8),iterations=3)
	# cv2.imshow('dialate2',gray)

	temp_gray=gray.copy()
	thresh = cv2.threshold(temp_gray, 60, 255, cv2.THRESH_BINARY)[1]

	cnts = cv2.findContours(thresh.copy(), cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	edges=0*gray.copy()
	countours_exist=None
	maxA=0.

	max_contour=None
	for c in cnts:
		# compute the center of the contour
		A = cv2.contourArea(c)

		countours_exist=1
		if A>maxA:
			maxA=A
			max_contour=c


	if countours_exist==1:
		p_min= .15* cv2.arcLength(max_contour,True)
		equi_radius = .5*np.sqrt(4*maxA/np.pi)
		M = cv2.moments(max_contour)
		cx0 = int(M['m10']/M['m00'])
		cy0 = int(M['m01']/M['m00'])

		# print('p_min is: ')
		# print(p_min)
		for c in cnts:
			# compute the center of the contour
			perimeter = cv2.arcLength(c,True)

			if perimeter>p_min:
				
				M = cv2.moments(c)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])

				if np.linalg.norm(np.array([cx-cx0,cy-cy0]))< equi_radius:

					cv2.drawContours(edges, [c], -1, (255), 1)
		

	# cv2.imshow('edges',edges)
	

	if countours_exist is None:
		return center, inner_corners, outer_corners
	else:
		# draw the contour and center of the shape on the image
		
		center=np.array([cx0,cy0])
		min_line_scale=.3
		if p_min<60:
			min_line_scale=.5
		#lines = cv2.HoughLines(edges,1,np.pi/180, int(.3*p_min)) 

		# start_t=time()
		lines = cv2.HoughLinesP (edges, 1, np.pi/180, int(.3*p_min), minLineLength = 10, maxLineGap = 3)
		# print("--- %s s HoughLinesP ---" % (time() - start_t))
		


		if lines is None:
			return center, inner_corners, outer_corners
		if lines.shape[0]<2:
			return center, inner_corners, outer_corners
		else:
			# cv2.imshow('lines drawn',draw_lines_P(lines,img.copy()))
			# start_t=time()
			center, inner_corners, outer_corners = findCornersP(lines)
			# print("--- %s s find corners P ---" % (time() - start_t))

			if center is None or np.isnan(center).any() or np.isnan(inner_corners).any() or np.isnan(outer_corners).any():
				return np.array([cx0,cy0]),np.zeros((4,2)),np.zeros((4,2))
			else:
				H= np.abs(inner_corners[3,1]-inner_corners[0,1]) #height
				W= np.abs(inner_corners[2,0]-inner_corners[1,0]) #width

				if H/W < 4.:
					return center, inner_corners, outer_corners
				else:
					return np.array([cx0,cy0]),np.zeros((4,2)),np.zeros((4,2))
				
def drawCorners( center, inner_corners, outer_corners,imgOG):


	cv2.circle(imgOG,(int(center[0,0]),int(center[0,1])),3,(0,0,255),-1)
	for i in range(inner_corners.shape[0]):
		cv2.circle(imgOG,(int(inner_corners[i,0]),int(inner_corners[i,1])),3,(255,0,0),-1)
		cv2.circle(imgOG,(int(outer_corners[i,0]),int(outer_corners[i,1])),3,(0,255,0),-1)		 	

	return imgOG

def findCornersP(lines):
	#make into a Nx2 rather than Nx1x2

	lines=np.squeeze(lines)

	#[x1 y1 x2 y2]
	#yoink out the lines that are horizontal or vertical and place them in seperate arrays

	#+.001 is a hack to prevent div by 0
	vert_lines= lines[np.where( ( abs(lines[:,2]-lines[:,0])/(abs(lines[:,3]-lines[:,1])+.001) < .55 ) )] #vertical
	hor_lines= lines[np.where( (abs(lines[:,3]-lines[:,1])/(abs(lines[:,2]-lines[:,0])+.001) < .55) )]

	intersections=None
	#find intersections of vertical and horizontal lines only, not all lines            
	for i in range(hor_lines.shape[0]):
		for j in range(vert_lines.shape[0]):

			#point on hor line
			x1=hor_lines[i,0]
			y1=hor_lines[i,1]
			#point on vert line
			x2=vert_lines[j,0]
			y2=vert_lines[j,1]

			m1= float(hor_lines[i,3]-y1)/(hor_lines[i,2]-x1)#slope of hor line

			if (vert_lines[j,2]-x2)==0:
				#perfectly vertical line, so avoid div by 0 error
				xi=x2
				yi= m1*(xi-x1)+y1
			else:
				m2= float(vert_lines[j,3]-y2)/(vert_lines[j,2]-x2)
				
				#intersection of 2 lines in point-slope form
				xi= (m2*x2 - m1*x1 + y1 - y2)/(m2-m1)
				yi= m1*(xi-x1)+y1


			if i==0 and j==0:
				intersections=np.array([[xi,yi]])
			else:
				intersections=np.append(intersections,np.array([[xi,yi]]),axis=0)
				

	if intersections is None:
		return np.zeros((1,2)),np.zeros((4,2)),np.zeros((4,2))
	else:
		extremea=np.array([np.amax(intersections,axis=0) , np.amin(intersections,axis=0)])
		mid=((extremea[0,:]-extremea[1,:])/2)+extremea[1,:]

		#extremea: max_x, max_y; min_x,min_y
		# print('inter')
		# print(intersections)
		# print('extremea')
		# print(extremea)
		# print('mid')
		# print(mid)

		#throw out points in the middle of maxima
		#horizontally
		intersections= np.delete(intersections, np.where( (.3<= (intersections[:,0]-extremea[1,0])/(extremea[0,0]-extremea[1,0])) & ( (intersections[:,0]-extremea[1,0])/(extremea[0,0]-extremea[1,0]) <= .7 )), axis=0)
		#vertically
		#at an angle the close vertical side will be very large in pixels, while the far can be quite small, so range is bigger for these intersections
		intersections= np.delete(intersections, np.where( (.4<= (intersections[:,1]-extremea[1,1])/(extremea[0,1]-extremea[1,1])) & ( (intersections[:,1]-extremea[1,1])/(extremea[0,1]-extremea[1,1]) <= .6 )), axis=0)

		# print('inter after delete')
		# print(intersections)

		label = 1*((intersections[:,0] > mid[0]) & (intersections[:,1] > mid[1]))+ 2*((intersections[:,0] < mid[0]) & (intersections[:,1] > mid[1])) + 3*((intersections[:,0] > mid[0]) & (intersections[:,1] < mid[1])) + 4* ((intersections[:,0] < mid[0]) & (intersections[:,1] < mid[1]))
		label=label-1

		# print label


		if np.isin(np.array([0,1,2,3]),label).all(): #if all 4 corners are found
			# print 'ineter where label=0'
			corner0 = intersections[np.squeeze(np.where(label==0)),:]
			corner1 = intersections[np.squeeze(np.where(label==1)),:]
			corner2 = intersections[np.squeeze(np.where(label==2)),:]
			corner3 = intersections[np.squeeze(np.where(label==3)),:]

			corner0=np.reshape(corner0,(corner0.size/2,2))
			corner1=np.reshape(corner1,(corner1.size/2,2))
			corner2=np.reshape(corner2,(corner2.size/2,2))
			corner3=np.reshape(corner3,(corner3.size/2,2))



			centers= np.array( [np.mean(corner0,axis=0),np.mean(corner1,axis=0),np.mean(corner2,axis=0),np.mean(corner3,axis=0)])
			center_temp= np.mean(centers,axis=0)

			center= np.array([[center_temp[0],center_temp[1]]])

			#stores the max/min (col 0, col1 ) distances in each cluster (row) 
			dist_store=np.array([[0.,999999.],[0.,999999.],[0.,999999.],[0.,999999.]]) 
			inner_corners=np.zeros((4,2))
			outer_corners=np.zeros((4,2))
			#find the minimum and maximum dist in each corner, gives inner and outer corners
			for i in range(intersections.shape[0]):
				dist= np.linalg.norm((intersections[i,:]-center))

				if dist> dist_store[label[i],0]: #if dist is greater that the max (col 0) in its group (label[i])
					dist_store[label[i],0]=dist
					outer_corners[label[i],:]=intersections[i,:]
				if dist< dist_store[label[i],1]: #if dist is less that the min (col 1) in its group (label[i])
					dist_store[label[i],1]=dist
					inner_corners[label[i],:]=intersections[i,:]
			return center, inner_corners, outer_corners
		else:
			return np.zeros((1,2)),np.zeros((4,2)),np.zeros((4,2))

def draw_lines_P(lines,frame):
	lines=np.squeeze(lines)

	# print(lines)
	for i in range(lines.shape[0]):
		
		cv2.line(frame,(lines[i,0],lines[i,1]),(lines[i,2],lines[i,3]),(0,255,0),2)
	
	return frame

def GMM(img,thresh,k,mean,cov,pi):

	print('Running GMM')
	icov = np.linalg.inv(cov)
	pmax = np.sum(pi[:])
	p = np.zeros((k,img.shape[0],img.shape[1]))
	img_flat= np.reshape(img, (img.shape[0]*img.shape[1] , 3))
	indx= np.where(img_flat[:,0] > 1)
	for i in range(1,k):
		diff= np.matrix(img_flat-mean[i])
		p_flat=np.zeros((diff.shape[0],1))
		p_flat[indx]= pi[i]*np.exp( -.5* np.sum( np.multiply( diff[indx] * icov[i], diff[indx]),axis=1))
		p[i,:,:]= np.reshape(p_flat, (img.shape[0],img.shape[1]))
	p_sum = np.sum(p,axis = 0)/pmax
	mask = cv2.inRange(p_sum,thresh,1)
	# kernel = np.ones((3,3),np.uint8)

	return mask

def img_callback(data):
	global global_command
	global global_wallisdone
	global global_lastmags
	global pause_active
	global pauselength
	global pause_start_time



	img = bridge.imgmsg_to_cv2(data, "bgr8")


	# start_t=time()
	start_t=time.time()
	median = cv2.medianBlur(img,5)
	# median = cv2.medianBlur(median,5)
	mask = GMM(median,thresh,k,mean,cov,pi)
	res = cv2.bitwise_and(img,img, mask= mask)
	center, inner_corners, outer_corners= applyCorners2Mask(mask,img)

	if not (inner_corners==0).any():
		points_2d = np.float32([[center[0,0], center[0,1]], [inner_corners[1,0], inner_corners[1,1]],
			[inner_corners[0,0], inner_corners[0,1]], [inner_corners[2,0], inner_corners[2,1]],
			[inner_corners[3,0], inner_corners[3,1]]]) 

		# rvec and tvec will give you the position of the world frame(defined at the center of the window) relative to the camera frame 
		_res, rvec, tvec = cv2.solvePnP(points_3d, points_2d, K, dist_coeff, None, None, False, cv2.SOLVEPNP_ITERATIVE)
		rmat,idk = cv2.Rodrigues(rvec)

		#vector to window in camera frame
		rw_inc=tvec
		#this is the vector to the window center in the body frame
		rw_inb = np.matmul(bRc,rw_inc)
		#matrix from Inertial(window) frame to camera frame
		cRi = rmat
		bRi = np.matmul(bRc,cRi)
		#desired vector in body frame:
		vdes_inb = np.matmul(bRi,vdes_ini)
		#desired position in body frame
		rdes_inb = rw_inb + vdes_inb
		#lets just yaw toward the window each time, by the time we get lined up should be good
		#this also prevents yawing too much and losing sight
		yaw_des= np.arctan(rw_inb[1]/rw_inb[0])  * (180/3.14) 

		print("--- %s full operation ---" % (time.time() - start_t))

		# mask= cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
		mask=drawCorners(center, inner_corners, outer_corners,res)

		print 'command to get to infront of window (m)'
		print rdes_inb/1000.
		print 'command to get to window (m)'
		print rw_inb/1000.
		print 'yaw this much to look at window (deg) +ve left'
		print yaw_des

		if global_wallisdone:

			if global_first_window_move:
				#if its the first window move you wanna fly and then do a good yaw,

				print('FIRST GO')
				#fly:
				global_command.x=.9*(rdes_inb[0]/1000.)
				global_command.y=.9*(rdes_inb[1]/1000.)
				global_command.z=.9*(rdes_inb[2]/1000.)
				global_command.w=1.
				command_pub.publish(global_command)

				#good yaw:
				x= np.linalg.norm(rdes_inb/1000.)
				y= np.linalg.norm(rw_inb/1000.)

				A= np.arccos( (x*x + y*y - 1)/(2*x*y))
				Bp= np.pi - np.arccos( (x*x + 1 - y*y)/(2*x))

				#you want to yaw then:
				firstyaw= yaw_des + ((Bp - A)*180/np.pi)

				global_first_window_move=False


			else:

				if pause_active:
					print('pausing like a good boi')
					if time.time() - pause_start_time > pauselength:
						pause_active=False

				else:

					#coarse yaw, doesnt need to be perfect when youre far away
					if np.abs(yaw_des)>25:
						#yaw only
						global_command.x=0
						global_command.y=0
						global_command.z=0
						global_command.w=yaw_des
						command_pub.publish(global_command)
						pause_active=True
						pause_start_time=time.time()
						
					else:
						mag= np.linalg.norm(rdes_inb/1000.)

						global_lastmags[0,0:4]=global_lastmags[0,1:5]
						global_lastmags[0,4]=mag

						print('running avg:',np.linalg.norm(global_lastmags))


						
						if np.linalg.norm(global_lastmags)<.12:
							pub_land.publish()
							print('\n \n \n \n \n')
							print('SHOOT BITCH!')
							pub_land.publish()
							print('\n \n \n \n \n')
							print('SHOOT BITCH!')
							pub_land.publish()
							print('\n \n \n \n \n')
							print('SHOOT BITCH!')
							pub_land.publish()
							print('\n \n \n \n \n')
							print('SHOOT BITCH!')
							pub_land.publish()

						else:
							if mag<.85:


								global_command.x=.8*(rdes_inb[0]/1000.)
								global_command.y=.8*(rdes_inb[1]/1000.)
								global_command.z=.8*(rdes_inb[2]/1000.)
								global_command.w=0
								command_pub.publish(global_command)

								pause_active=True
								pause_start_time=time.time()
							else:
								global_command.x=.6*(rdes_inb[0]/1000.)
								global_command.y=.6*(rdes_inb[1]/1000.)
								global_command.z=.6*(rdes_inb[2]/1000.)
								global_command.w=1.
								command_pub.publish(global_command)

	else:
		print 'Not enough corners found'

	# mask= cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
	img_pub.publish(bridge.cv2_to_imgmsg(res, "bgr8"))





def WindowNode():
	rospy.init_node('Window_master', anonymous=False, log_level=rospy.WARN)
	# img_sub = rospy.Subscriber("/image_raw_throttle", Image, img_callback)
	# print('Taking off') #i think the node was too speedy or something, ignore this takeoff mess its just a hack for testing
	# time.sleep(5.)
	# pub_takeoff.publish()
	# print('huh?')

	
	print('Taking off') #i think the node was too speedy or something, ignore this takeoff mess its just a hack for testing
	# pub_takeoff.publish()
	print('huh?')
	time.sleep(8.)
	print('Taking off try 2')
	pub_takeoff.publish()
	print('huh?')
	time.sleep(6.)
	# time.sleep(4.5)
	hardcoded_Wall()

	img_sub = rospy.Subscriber("/image_raw", Image, img_callback, buff_size=2**24, queue_size=1)
	rospy.spin()

if __name__ == '__main__':
	WindowNode()
