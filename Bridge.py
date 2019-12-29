#!/usr/bin/env python

import cv2 
import numpy as np
import imutils
import time
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry # We need this message type to read position and attitude from Bebop nav_msgs/Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Empty
from cv_bridge import CvBridge, CvBridgeError

import rospy



##################################
# READ ME:

# HARDCODE the angle to the bridge when this node starts.
# IN degrees, from 0 is facing forward, positive is to the left or counterclockwise!

hardcoded_angle= -70 #degrees from facing forward, positive is left

#this is angle to turn after bridge is corossed
hardcoded_yaw= 0#-60 #same definitions as above

# This node needs a bottom camera

############################################################################
pauselength=8



pause_active=False
pause_start_time=0




# from multiprocessing import Pool,Process, Queue
outt=Point()
global_pos= Pose()#np.array([0.,0.,0.])
global_vel=Twist()
global_bridge_description=np.array([0.,0.,0.])
global_last_good_bridge=np.array([0.,0.,0.])
global_last_iffy_bridge=np.array([0.,0.,0.])
global_last_good_pos= Twist()

first_got=False
crossed=False
new_bridge=False

global_command=Quaternion()

frames_checked=0

bridge = CvBridge()
img_pub = rospy.Publisher("/center_point_img",Image,queue_size=1)
command_pub = rospy.Publisher("/moveto_cmd_body",Quaternion,queue_size=1)

pub_land= rospy.Publisher('bebop/land',Empty,queue_size=1)

def poor_mans_differential(img, cannyval):
	edges = cv2.Canny(img,cannyval,cannyval*2.7,apertureSize = 3)
	# edges = cv2.dilate(edges,np.ones((3,3),np.uint8),iterations = 1)
	return edges

def double_threshold(img,r1,r2,threshinner,threshouter):

	inner_only= img*0

	outer_only=inner_only+1#np.ones(img.shape)

	inner_only[int(img.shape[0]/2-r1):int(img.shape[0]/2+r1) , int(img.shape[1]/2-r2):int(img.shape[1]/2+r2)]=1
	outer_only[int(img.shape[0]/2-r1):int(img.shape[0]/2+r1) , int(img.shape[1]/2-r2):int(img.shape[1]/2+r2)]=0

	inner_pic= cv2.bitwise_and(img,img, mask=inner_only)
	outer_pic= cv2.bitwise_and(img,img, mask=outer_only)

	outer_mask= cv2.inRange(outer_pic, threshouter[0],threshouter[1])#100, 200)
	inner_mask= cv2.inRange(inner_pic, threshinner[0],threshinner[1])#130, 230)

	mask=cv2.bitwise_or(outer_mask,inner_mask)

	return mask
	# mask = cv2.inRange(img_filt, 0, 13)

def search_for_rects(max_contour,cnts):

	got_bridge=False
	cx0=0
	cy0=0
	bridge_angle=0
	first_hole_angle=None


	checked=np.array([[-1,-1]])

	p_min= .15* cv2.arcLength(max_contour,True)
	equi_radius = .125/.15*p_min #.5*np.sqrt(4*maxA/np.pi)
	M = cv2.moments(max_contour)
	cx0 = int(M['m10']/(M['m00']+.0000000001))
	cy0 = int(M['m01']/(M['m00']+.0000000001))

	# cv2.drawContours(image_raw, [max_contour], -1, 255, 4)
	for c in cnts:
		perimeter = cv2.arcLength(c,True)
		if perimeter>p_min: #if its not very small
			M = cv2.moments(c)
			cx = int(M['m10']/(M['m00']+.0000000001))
			cy = int(M['m01']/(M['m00']+.0000000001))


			if np.equal(checked,[cx,cy]).all(1).any()==False:
				checked = np.append(checked, [[cx,cy]], axis=0)

				if np.linalg.norm(np.array([cx-cx0,cy-cy0]))< 1.2*equi_radius: #if its within the "bridge" body

					
					hull_area = cv2.contourArea(cv2.convexHull(c))+.0000000001
					solidity = float(M['m00'])/hull_area
					# print 'solidity: ',solidity
					if solidity>.8:


						rect=cv2.minAreaRect(c)
						# print '--'
						# print rect

						angle=rect[2]
						L=rect[1][0]
						W=rect[1][1]

						# cv2.drawContours(image_raw, [c], -1, 0, 2)
						if L<W: #if length is shorter than width. then the angle is actually 90deg off
							W=rect[1][0]
							L=rect[1][1]
							if rect[2]<0:
								angle=rect[2]+90
							else:
								angle=rect[2]-90

						# print angle, L/W
						if np.abs((L/W)-3)<.66: #if its got the aspect ratio of the hole
							# print 'got a rectangle'
							# cv2.drawContours(image_raw, [c], -1, 125, 3)
							if first_hole_angle is None:
								first_hole_angle=angle
							else:
								if np.abs(angle-first_hole_angle) < 5:
									got_bridge=True
	return got_bridge,first_hole_angle,cx0,cy0

def find_bridge(image_raw):


	# print '--------------'

	cannyval=int(np.amin(image_raw.shape)*.12)#.35)#.31)
	# print cannyval
	# adaptive_filtersize= int(np.amin(image_raw.shape)*.01)
	# print adaptive_filtersize

	
	img_filt=poor_mans_differential(image_raw,cannyval)
	mask = cv2.inRange(img_filt, 0, 13)
	# mask2 = cv2.inRange(image_raw, 130, 230)
	# mask = cv2.erode(mask,np.ones((3,3),np.uint8),iterations = 1)
	# mask = cv2.dilate(mask,np.ones((3,3),np.uint8),iterations = 2)
	# 



	mask2=double_threshold(image_raw,60,80,[200,260],[170,260])
	mask= cv2.bitwise_and(mask,mask2)

	# plt.cla()
	# plt.imshow(mask)
	# plt.pause(.005)


	# mask = cv2.erode(mask,np.ones((3,3),np.uint8),iterations = 1)
	# mask = cv2.dilate(mask,np.ones((3,3),np.uint8),iterations = 1)
	# 

	

	# edges = cv2.Canny(mask,cannyval,cannyval*3,apertureSize = 3) 

	# cv2.imshow('canny',edges)

	cnts = cv2.findContours(mask, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)

	countours_exist=False
	if len(cnts)>3:
		countours_exist=True
		cnts = sorted(cnts, key=lambda x: cv2.arcLength(x,False))
	

	# for c in cnts:
		
	# 	# A = cv2.contourArea(c)
	# 	A = cv2.arcLength(c,False)
		
	# 	if A>maxA:
	# 		countours_exist=True
	# 		maxA=A
	# 		max_contour=c
	got_bridge=None
	first_hole_angle=None
	cx0=0.
	cy0=0.



	if countours_exist:

		#longest contour
		max_contour=cnts[-1]
		got_it,t1,x1,y1 = search_for_rects(max_contour,cnts)

		#if you dont get anything below then go with your biggest one
		first_hole_angle=t1
		cx0=x1
		cy0=y1

		if got_it==True: #if its the bridge then good, dont try shit
			got_bridge=True
		else: #try the second longest if not sure about first
			max_contour=cnts[-2]
			got_it,t2,x2,y2 = search_for_rects(max_contour,cnts)

			if got_it==True: #if its the bridge then good
				got_bridge=True
				first_hole_angle=t2
				cx0=x2
				cy0=y2
			else: #try the third longest
				max_contour=cnts[-3]
				got_it,t3,x3,y3 = search_for_rects(max_contour,cnts)

				if got_it==True: #if its the bridge then good
					got_bridge=True
					first_hole_angle=t3
					cx0=x3
					cy0=y3


	#for debigging
	angle = None

	if first_hole_angle is not None:

		angle=(first_hole_angle+90) * 3.14/180	

	if got_bridge or (first_hole_angle is not None):

		# print 'Got bridge'
		
		angle=(first_hole_angle+90) * 3.14/180

		x2 = int(round(cx0 + 1000* np.cos(angle)))
		y2 = int(round(cy0 + 1000 * np.sin(angle)))
		x1 = int(round(cx0 - 1000* np.cos(angle)))
		y1 = int(round(cy0 - 1000 * np.sin(angle) ))

		if got_bridge==True:
			cv2.line(mask,(x1,y1),(x2,y2),(255,255,0),5)
			cv2.circle(mask,(int(cx0),int(cy0)),3,(255,0,0),-1)
		else:
			# print 'NOT SUPER SURE'
			cv2.line(mask,(x1,y1),(x2,y2),(0,255,0),1)
			cv2.circle(mask,(int(cx0),int(cy0)),3,(255,0,0),-1)

		# cv2.imshow('image_raw',image_raw)
	
	
	# print("--- %s seconds ---" % (time.time() - start_time))
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()
	# plt.cla()
	# plt.imshow(image_raw)
	# plt.pause(.005)
	

	return got_bridge, mask, cx0, cy0, angle

def img_callback(data):
	global global_bridge_description
	global frames_checked
	global global_last_iffy_bridge
	global first_got
	global new_bridge

	# print 'recieving images'

	frames_checked=frames_checked+1

	img = bridge.imgmsg_to_cv2(data, "bgr8")
	imgbgr=img.copy()

	img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	#scale image down for speed, reduces accuracy
	imgScale=1

	gotit, img_center,x,y, angle = find_bridge(img)

	

	#going to try to validate its a good bridge here
	if gotit or (angle is not None):

		
			

		if gotit==True:

			if first_got==False:
				first_got=True
				global_last_iffy_bridge[0]= int(x*(1/imgScale))+.5 #horizontal like
				global_last_iffy_bridge[1]=int(y*(1/imgScale))+.5
				global_last_iffy_bridge[2]=angle

				print 'ENTERED FIRST GOT'
				frames_checked=0

			print 'Got a new bridge  ','damn sure'
			#then found at least 2 holes that are good
			# cv2.line(imgbgr,(x1,y1),(x2,y2),(255,255,0),5)
			# cv2.circle(imgbgr,(int(cx0),int(cy0)),3,(255,0,0),-1)
			global_bridge_description[0]=int(x*(1/imgScale)) #horizontal like
			global_bridge_description[1]=int(y*(1/imgScale))
			global_bridge_description[2]=angle #radians yo
			frames_checked=0

			global_last_iffy_bridge=global_bridge_description
			move_appropriately(global_bridge_description)


		else:
			print 'Got a new bridge  ','NOT SUPER SURE'
			#if not super sure then iffy .5 solution
			global_bridge_description[0]=int(x*(1/imgScale))+.5 #horizontal like
			global_bridge_description[1]=int(y*(1/imgScale))+.5
			global_bridge_description[2]=angle #radians yo
			# frames_checked=0

			# if np.linalg.norm(global_last_iffy_bridge[:2]-(global_bridge_description[:2]-.5))<4  and np.abs(global_last_iffy_bridge[2]-global_bridge_description[2])<.1:
			# 	print('DIS DAT SKETCHY WETCHY but ill take it')

			# 	global_last_iffy_bridge=global_bridge_description
			# 	#update global good bridge?
			# 	move_appropriately(global_bridge_description)

			# else:
			global_last_iffy_bridge=global_bridge_description

	else:
		print('--')



	img_center = cv2.cvtColor(img_center,cv2.COLOR_GRAY2BGR)
	img_pub.publish(bridge.cv2_to_imgmsg(img_center, "bgr8"))
	# bridge_pub.publish(outt)

def odom_callback(msg):
	global global_pos
	global global_vel
	# rospy.loginfo(msg.pose.pose)
	# rospy.loginfo(msg.twist.twist)

	#global_pos=np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])
	#print(global_pos)
	global_pos=msg.pose.pose
	global_vel=msg.twist.twist

def search_bridge():
	global global_command
	global global_bridge_description
	global first_got

	global global_last_iffy_bridge

	global global_command

	global pause_active
	global pauselength
	global pause_start_time

	global hardcoded_angle
	global hardcoded_yaw

	global frames_checked
	global crossed
	global new_bridge
	


	if global_pos.position.z<2.0:

		print('moving up')
		global_command.x = 0
		global_command.y = 0 
		global_command.z = .35
		global_command.w = 0 # Latching disabled, if we see something sooner might as well go then
		# SEND IT
		print('sending command up: ',global_command)
		command_pub.publish(global_command) #move up to see something
	else:				



		if crossed==False:
			if not first_got:

				print('im high up and dont see shit')
				
				hardcoded_angle_rad= hardcoded_angle*(3.14/180)

				global_command.x = .21*np.cos(hardcoded_angle_rad)
				global_command.y = .21*np.sin(hardcoded_angle_rad)
				global_command.z = 0
				global_command.w = 0 # Latching disabled, if we see something sooner might as well go then
				# SEND IT
				print('sending command search: ',global_command)
				command_pub.publish(global_command)

					# pub_land.publish() #edge case shit here

			else: #we something at some point
				if global_bridge_description.all()==0:
					#we dont see shit rn

					#check counter, if its been a while then cry and think about doing a sketchyboiii
					if frames_checked>1000:
						print('I WOULD DO SKETCHY SHIT NOW')
						# pub_land.publish()
						move_appropriately(global_last_iffy_bridge)
		else:
			print 'I already crossed'


			print('time: ',time.time() - pause_start_time)

			if time.time() - pause_start_time > pauselength: #this is non-blocking pause implementation
				pause_active=False

				bridge_angle= -global_last_good_bridge[2] #in rad %this makes sense in regular ass x, y coordinates (not image)
				bridge_angle= 180*bridge_angle/np.pi #convert to degrees

				if bridge_angle>0:

					yaw_right_2bridge= 90-bridge_angle
					yaw_right_2wall= yaw_right_2bridge + 60
				else:
					yaw_left_2bridge= 90+bridge_angle
					yaw_right_2wall= 60 - yaw_left_2bridge

				print('bridge_angle: ',bridge_angle)
				print('YAWING--------------------------------------------')
				global_command.x=0
				global_command.y=0
				global_command.z=0
				#global_command.w=-yaw_right_2wall
				global_command.w= hardcoded_yaw
				command_pub.publish(global_command)
				time.sleep(1)
				rospy.signal_shutdown('BOOTY')


def find_bridge_main():
	rospy.init_node('find_bridge', anonymous=True)
	# img_sub = rospy.Subscriber("/duo3d/left/image_rect_throttle", Image, img_callback,queue_size=1,buff_size=52428800)
	

	# #COMMENT OUT THIS SECTION HERE WHEN NOT TESTING!!!
	# pub_takeoff= rospy.Publisher('/bebop/takeoff',Empty,queue_size=1)
	# print('Taking off') #i think the node was too speedy or something, ignore this takeoff mess its just a hack for testing
	# # pub_takeoff.publish()
	# print('waiting?')
	# time.sleep(5.)
	# print('Taking off try 1')
	# pub_takeoff.publish()
	# print('huh?')
	# time.sleep(6.)


	img_sub = rospy.Subscriber("/duo3d/left/image_rect", Image, img_callback,queue_size=1,buff_size=52428800)
	rospy.Subscriber('/bebop/odom', Odometry, odom_callback)
	# rospy.spin()

	telemrate = 3
	rate = rospy.Rate(telemrate)
	# spin() simply keeps python from exiting until this node is stopped
	while not rospy.is_shutdown():
		search_bridge()
		rate.sleep()

def move_appropriately(bridge_description):
	global global_pos
	global global_command
	global frames_checked
	global crossed

	global pause_active
	global pause_start_time
	frames_checked=0

	# global_marker_center=bridge_description
	#keyboard w 17.5

	#fit vertically in frame at 8in z
	#fit horizontally 6
	FOVx=56 #deg
	FOVy=50
	
	if not crossed:
		C=np.array([160,120,0])
		A=np.array([bridge_description[0],bridge_description[1],0])
		B=np.array([A[0] + 30* np.cos(bridge_description[2]),A[1] + 30 * np.sin(bridge_description[2]),0])

		dist=np.linalg.norm(np.cross(C-A,B-A))/np.linalg.norm(B-A)

		dist_real= np.tan(((dist/140)*53)*np.pi/180)*global_pos.position.z
		
		print 'dist_real: ',dist_real
		print 'bridge is: ',bridge_description


		if bridge_description[0]%1 != 0 and bridge_description[1]%1 != 0:
			dist_real=.5


		if dist_real<.03:

			vector2bridge= np.array([A[0]-160,120-A[1]])#-A[:2]# if 0,0 in top left corner
			
			print 'vector2bridge: ',vector2bridge
			deg_offsets= (vector2bridge/(2*np.array([160,120]))) * np.array([FOVx,FOVy])
			marker_loc=np.tan(deg_offsets*np.pi/180)*global_pos.position.z #marker loc relative to you 
			#marker_loc is x right, y forward
			#shoot the bridge here!


			overshoot=.6

			#ahhhh, i am an idiot, overshoot is done like dis:
			mag= np.linalg.norm(marker_loc)

			if mag<.1:
				overshoot=.65

			shooties = (marker_loc/mag)*(mag+overshoot) #fucking duh


			# moveto_body(marker_loc[1]+overshoot,-marker_loc[0]-overshoot,0) #body x is forward, y is left
			#stop moving first (sometimes its still drifting and shit)
			global_command.x = 0
			global_command.y = 0
			global_command.z = 0
			global_command.w = 0 # 
			# SEND IT
			print('sending stop command: ',global_command)
			command_pub.publish(global_command)

			time.sleep(1)

			global_command.x = shooties[1]
			global_command.y = -1*shooties[0]
			global_command.z = 0
			global_command.w = 1 # Latching on, we shooting shit
			# SEND IT
			print('sending SHOOT command: ',global_command)
			command_pub.publish(global_command) #bangers
			crossed=True

			pause_active=True
			pause_start_time=time.time()



		else:
			# factor=.75
			# if dist_real<.43:
			# 	factor=.9
			factor=1.3

			if bridge_description[2]> (3.14/2):
				angle=bridge_description[2]-3.14 #this angle is +ve CW!!!!
			else:
				angle=bridge_description[2]

			#alright fuck it, picking the closest point on the line is weird
			#instead ill go to a point on the line, that is some dist meter away from the bridge, that is closer to me
			vector2bridge= np.array([A[0]-160,120-A[1]])#-A[:2]# if 0,0 in top left corner
			print 'vector2bridge: ',vector2bridge
			deg_offsets= (vector2bridge/(2*np.array([160,120]))) * np.array([FOVx,FOVy])
			marker_loc=np.tan(deg_offsets*np.pi/180)*global_pos.position.z

			distfrombridge=.35 #meters
			#point 1
			P1 = np.array([marker_loc[0]+distfrombridge*np.cos(angle),marker_loc[1]-distfrombridge*np.sin(angle)])
			P2 = np.array([marker_loc[0]-distfrombridge*np.cos(angle),marker_loc[1]+distfrombridge*np.sin(angle)])
			M1=np.linalg.norm(P1)
			M2=np.linalg.norm(P2)

			print 'marker_loc: ',marker_loc
			print 'P1: ',P1
			print 'P2: ',P2

			if M1<M2:
				waypoint_loc=P1
			else:
				waypoint_loc=P2

			#dirvec= np.array([dist*np.sin(angle), dist*np.cos(angle)])
			# print 'dirvec: ',dirvec
			# deg_offsets= (dirvec/(2*np.array([160,120]))) * np.array([FOVx,FOVy])
			# waypoint_loc= np.tan(deg_offsets*np.pi/180)*global_pos.position.z

			#moveto_body(.5*marker_loc[1],-.5*marker_loc[0],0)

			if np.linalg.norm(factor*waypoint_loc) < .1:
				waypoint_loc= waypoint_loc/(np.linalg.norm(waypoint_loc)) * (.18/factor)


			global_command.x = factor*waypoint_loc[1]
			global_command.y = -1*factor*waypoint_loc[0]
			global_command.z = 0
			global_command.w = 0 # Latching cuz we think way faster than the quad moves, 
			# SEND IT
			print('sending lineup command: ',global_command)
			command_pub.publish(global_command)


if __name__ == '__main__':
	try:
		
		find_bridge_main()
	except rospy.ROSInterruptException:
		pass
