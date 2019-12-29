#!/usr/bin/env python2
import cv2
import numpy as np
from matplotlib import pyplot as plt
#from mpl_toolkits.mplot3d import axes3d, Axes3D 
#from scipy.stats import multivariate_normal
#from scipy.cluster.vq import kmeans, whiten, kmeans2
import imutils
#Image Select
#import Tkinter, tkFileDialog
import time
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import rospy
import copy

from sklearn import linear_model, datasets

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry # We need this message type to read position and attitude from Bebop nav_msgs/Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

#import FixedGMM

shutdown_flag = 0
bridge = CvBridge()

# Initialize GMM Feet params
GMMin = './WoodenFeetGMM_3.npz'
thresh = .15#1e-4#1e-5
npzfile = np.load(GMMin)
k = npzfile['arr_0']
mean = npzfile['arr_1']
cov = npzfile['arr_2']
pi = npzfile['arr_3']

# Initialize rolling avg params
mvAvgNum = 4
numFeetArr = np.zeros((1,mvAvgNum))
XPixelErr= np.zeros((1,mvAvgNum))

#img_pub = rospy.Publisher("/foot_mask",Image,queue_size=1)
command_pub = rospy.Publisher("/moveto_cmd_body",Quaternion,queue_size=1)
test_pub = rospy.Publisher("/test",Quaternion,queue_size=1)
def run():
    rospy.init_node('FeetTracker', anonymous=True)
    #rospy.Subscriber("/image_raw", Image, detectFeet,queue_size=1)
    # rospy.Subscriber("/image_raw_throttle", Image, detectFeet,queue_size=1)
    rospy.Subscriber("/image_raw", Image, detectFeet, buff_size=2**24, queue_size=1)
    

    rospy.spin()
def GMM(img,thresh,k,mean,cov,pi):
	#img_thresh = np.zeros(img.shape) # initialize black image
	#print('Running GMM')
	icov = np.linalg.inv(cov)
	#p0 = 1/(((2*3.14159)**3*np.linalg.det(cov)))**(0.5)
	#pmax = np.zeros(k)
	#for j in range(0,k):
	#	pmax[j] =  p0[j]*np.exp(-.5*np.linalg.multi_dot([[0,0,0],icov[j],[0,0,0]]))

	# Loop through pixels and compare each value to the threshold.
	#x=0
	#y=0
	
	#po =  np.zeros(img.shape[:2])
	pmax = np.sum(pi[:])
	# for col in img:
	# 	for pixel in col:
	# 		p = np.zeros(k)
	# 		for j in range(0,k):
	# 			if (pixel == np.zeros(3)).all():
	# 				p[j] = 0
	# 			else:
	# 				temp = pixel-mean[j]
	# 				p[j] = pi[j]*np.exp(-.5*np.linalg.multi_dot([temp,icov[j],temp]))
	# 		po[y,x] = np.sum(p)
	# 		x = x+1
	# 	x = 0
	# 	y = y+1
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

	kernel = np.ones((3,3),np.uint8)
	#mask = cv2.dilate(mask,kernel,iterations=1)
	#mask = cv2.erode(mask,kernel,iterations = 1)
	
	#pnorm = po/pmax#(po/np.max(po))

	#mask = (pnorm>=thresh).astype('uint8')*255
	#res = cv2.bitwise_and(img_thresh,img_thresh,mask = mask)
	return mask


def detectFeet(data):
	# Set global parameters for GMM inputs
	global thresh,k,mean,cov,pi
	global numFeetArr, XPixelErr, mvAvgNum
	global img,mask,kernel,erosion,betterMask, cnts
	global shutdown_flag
	# Pass image through CV bridge
	img = bridge.imgmsg_to_cv2(data,"passthrough")

	start_t=time.time()

	imgshape = img.shape
	#print imgshape
	x_im = int(imgshape[1]/2)
	
	
	img = img[img.shape[0]/2:img.shape[0],:,:]

	# Execute GMM
	mask = GMM(img,thresh,k,mean,cov,pi)

	# Apply erode and dilate
	kernel = np.ones((5,5),np.uint8)
	erosion = cv2.erode(mask,kernel,iterations = 1)
	betterMask = cv2.dilate(erosion,kernel,iterations=3)
	contour_img=0*np.ones(np.shape(betterMask))
	# Now, select only the two biggest features
	# Using contours
	cnts = cv2.findContours(betterMask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	cX = 0
	cY = 0
	# Find top two contours
	i = 0
	A = np.zeros((len(cnts),2))
	#print(len(cnts))
	for c in cnts:
		A[i,:] = np.array([cv2.contourArea(c),i])
		i = i+1
	sortA = np.sort(A,axis = 0)
	delta = 0

	if len(cnts)>=2:
		max_contour = cnts[sortA[0,1].astype(int)]
		second_contour = cnts[sortA[1,1].astype(int)]
		contour_img=cv2.drawContours(contour_img, [max_contour], -1, (255, 255, 255), -1)
		contour_img=cv2.drawContours(contour_img, [second_contour], -1, (255, 255, 255), -1)
		numFeet = 2
		# Extract center of contours
		M1 = cv2.moments(max_contour)
		cX1 = int(M1["m10"] / M1["m00"])
		cY1 = int(M1["m01"] / M1["m00"])
		M2 = cv2.moments(second_contour)
		cX2 = int(M2["m10"] / M2["m00"])
		cY2 = int(M2["m01"] / M2["m00"])
		# Set average of centers
		cX = (cX1+cX2)/2
		cY = (cY1+cY2)/2
		delta = np.abs(cX2-cX1)

		print x_im - cX

		# Compute center of the two contours
	elif len(cnts) ==1:
		max_contour = cnts[sortA[0,1].astype(int)]
		contour_img=cv2.drawContours(contour_img, [max_contour], -1, (255, 255, 255), -1)
		numFeet = 1
		M = cv2.moments(max_contour)
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
	else:
		contour_img=0*np.ones(np.shape(betterMask))
		numFeet = 0
	pubimg = np.uint8(contour_img)
	#cv2.cvtColor(contour_img,cv2.COLOR_BGR2RGB)
	#img_pub.publish(bridge.cv2_to_imgmsg(pubimg, "mono8"))

	numFeetArr[0,0:mvAvgNum-1]=numFeetArr[0,1:mvAvgNum]
	numFeetArr[0,mvAvgNum-1]=numFeet
	XPixelErr[0,0:mvAvgNum-1]=XPixelErr[0,1:mvAvgNum]
	XPixelErr[0,mvAvgNum-1]=cX
	print numFeetArr
	print("--- %s full operation ---" % (time.time() - start_t))

	# plt.figure(1)
	# plt.imshow(img)
	
	# plt.figure(2)
	# plt.imshow(contour_img)
	# plt.pause(0.02)

	flag = 0
	error = x_im - cX
	if all(numFeetArr[0,:]==2): #np.mean(numFeetArr) == 2:
		outCenter = np.mean(XPixelErr)
		flag = 2
		print('flag 2, good solution',error)
	elif all(numFeetArr[0,:]==1):
		outCenter = np.mean(XPixelErr)
		flag = 1
		print('flag 1, might need to yaw',error)
	else:
		outCenter = 0
		flag = 0
		print('flag 0, iffy',error)


	# Controller Here
	command=Quaternion()
	Px = 0.004
	FOVxPerPixel = 50.0/x_im# Half angle of FOV
	#print(FOVxPerPixel)
	Pyaw = 50.0/x_im # At most, yaw 30 degrees
	error = x_im - outCenter # Positive left, physically
	error_now = x_im - XPixelErr[0,mvAvgNum-1]
	if flag == 2: # Move to centered 
		if error < 100 and delta > 100:
			print('GOING THROUGH WALL')
			command.x = 2.0*np.cos(error_now*FOVxPerPixel*3.14/180)
			command.y = 2.0*np.sin(error_now*FOVxPerPixel*3.14/180)
			command.z = 0
			command.w = 1 # Latching enabled
			print('Cmd: ',command)
			command_pub.publish(command)
			#test_pub.publish(command)
			shutdown_flag = 1
			time.sleep(1)
			command_pub.publish(command)
			time.sleep(1)
			command_pub.publish(command)
			time.sleep(1)


			rospy.signal_shutdown('Woohoo')
			
		elif error < 100 and delta < 100:
			print('Stepping towards wall slowly')
			command.x = 0.5*np.cos(error*FOVxPerPixel*3.14/180)
			command.y = 0.5*np.sin(error*FOVxPerPixel*3.14/180)
			command.z = 0
			command.w = 1 # Latching enabled
			print('Cmd: ',command)
			command_pub.publish(command)
			test_pub.publish(command)
			
			
		else:
			print('Good Solution, but far off Center: Yawing')
			command.x = 0
			command.y = 0
			command.z = 0
			command.w = 0.5*FOVxPerPixel*error # 
			command_pub.publish(command) 
			#test_pub.publish(command)
			time.sleep(2)
			print('Cmd: ',command)
		# if np.abs(error_now) < 25 and delta > 100 and np.abs(error) < 35:
 	# 		print('GOING THROUGH WALL')
		# 	command.x = 2.0*np.cos(error*FOVxPerPixel*3.14/180)
		# 	command.y = 2.0*np.sin(error*FOVxPerPixel*3.14/180)
		# 	command.z = 0
		# 	command.w = 1 # Latching enabled
		# 	print('Cmd: ',command)
		# 	command_pub.publish(command)
		# else: # move to center the aircraft
		# 	print('Centering')
		# 	command.x = 0
		# 	command.y = Px*error
		# 	command.z = 0
		# 	command.w = 0 # Latching disabled   
		# 	print('Cmd: ',command)
		# 	command_pub.publish(command)
	elif flag == 1: # we need to yaw
		print('Not sure what to do.  I only see one foot.  Yawing')
		command.x = 0
		command.y = 0
		command.z = 0
		command.w = 0.5*Pyaw*error_now # 
		command_pub.publish(command) 
		print('Cmd: ',command)
	else:
		print('Not Sure Yet')



if __name__ == '__main__':
    try:
        
        run()
    except rospy.ROSInterruptException:
        pass