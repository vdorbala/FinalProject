#!/usr/bin/env python

import cv2 
import numpy as np
import imutils
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import rospy
# from multiprocessing import Pool,Process, Queue


bridge = CvBridge()
img_pub = rospy.Publisher("/center_point_img",Image,queue_size=1)
bridge_pub = rospy.Publisher("/bridge_details",Point,queue_size=1)

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

					
					hull_area = cv2.contourArea(cv2.convexHull(c))
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


	print '--------------'

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



	mask2=double_threshold(image_raw,60,60,[215,260],[170,260])
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

			

	if got_bridge or (first_hole_angle is not None):

		print 'Got bridge'
		
		angle=(first_hole_angle+90) * 3.14/180

		x2 = int(round(cx0 + 1000* np.cos(angle)))
		y2 = int(round(cy0 + 1000 * np.sin(angle)))
		x1 = int(round(cx0 - 1000* np.cos(angle)))
		y1 = int(round(cy0 - 1000 * np.sin(angle) ))

		if got_bridge==True:
			cv2.line(image_raw,(x1,y1),(x2,y2),(255,255,0),5)
			cv2.circle(image_raw,(int(cx0),int(cy0)),3,(255,0,0),-1)
		else:
			print 'NOT SUPER SURE'
			cv2.line(image_raw,(x1,y1),(x2,y2),(0,255,0),1)
			cv2.circle(image_raw,(int(cx0),int(cy0)),3,(255,0,0),-1)

		# cv2.imshow('image_raw',image_raw)
	
	
	# print("--- %s seconds ---" % (time.time() - start_time))
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()
	plt.cla()
	plt.imshow(image_raw)
	plt.pause(.005)
	

	return got_bridge, image_raw, cx0, cy0, first_hole_angle

def img_callback(data):
	img = bridge.imgmsg_to_cv2(data, "bgr8")

	img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	#scale image down for speed, reduces accuracy
	imgScale=1

	gotit, img_center,x,y, angle = find_bridge(img)
	
	outt=Point()

	if gotit:
		outt.x= int(x*(1/imgScale))
		outt.y= int(y*(1/imgScale))
		outt.z= angle
	else:
		#if not super sure then iffy .5 solution
		outt.x= int(x*(1/imgScale))+.5
		outt.y= int(y*(1/imgScale))+.5
		outt.z= angle

	img_center = cv2.cvtColor(img_center,cv2.COLOR_GRAY2BGR)
	img_pub.publish(bridge.cv2_to_imgmsg(img_center, "bgr8"))
	bridge_pub.publish(outt)

def find_bridge_main():
	rospy.init_node('find_bridge', anonymous=True)
	img_sub = rospy.Subscriber("/duo3d/left/image_rect_throttle", Image, img_callback,queue_size=1,buff_size=52428800)
	rospy.spin()


if __name__ == '__main__':
	try:
		
		find_bridge_main()
	except rospy.ROSInterruptException:
		pass
