#!/usr/bin/env python

import numpy as np
import cv2
import os
import Tkinter, tkFileDialog
from time import time
import imutils


def applyCorners2Mask(mask,img):

	center, inner_corners, outer_corners = np.zeros((1,2)),np.zeros((4,2)),np.zeros((4,2))


	gray=mask.copy()
	gray = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)[1]
	gray = cv2.dilate(gray,np.ones((3,3), np.uint8),iterations=4)
	gray = cv2.erode(gray,np.ones((3,3), np.uint8),iterations=3)
	#gray = cv2.dilate(gray,np.ones((5,5), np.uint8),iterations=3)
	cv2.imshow('dialate2',gray)

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
		

	cv2.imshow('edges',edges)
	

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
			cv2.imshow('lines drawn',draw_lines_P(lines,img.copy()))
			# start_t=time()
			center, inner_corners, outer_corners = findCornersP(lines)
			# print("--- %s s find corners P ---" % (time() - start_t))

			if center is None or np.isnan(center).any() or np.isnan(inner_corners).any() or np.isnan(outer_corners).any():
				return np.array([cx0,cy0]),np.zeros((4,2)),np.zeros((4,2))
			else:
				return center, inner_corners, outer_corners
				
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

		print('inter after delete')
		print(intersections)

		label = 1*((intersections[:,0] > mid[0]) & (intersections[:,1] > mid[1]))+ 2*((intersections[:,0] < mid[0]) & (intersections[:,1] > mid[1])) + 3*((intersections[:,0] > mid[0]) & (intersections[:,1] < mid[1])) + 4* ((intersections[:,0] < mid[0]) & (intersections[:,1] < mid[1]))
		label=label-1

		print label


		if np.isin(np.array([0,1,2,3]),label).all(): #if all 4 corners are found
			print 'ineter where label=0'
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
	kernel = np.ones((3,3),np.uint8)

	return mask


GMMin = 'YellowGMM3_100.npz'
thresh = .3#1e-4#1e-5

npzfile = np.load(GMMin)
k = npzfile['arr_0']
mean = npzfile['arr_1']
cov = npzfile['arr_2']
pi = npzfile['arr_3']
GMMargs = (thresh,k,mean,cov,pi)

dirpath = os.getcwd()
print 'looking for files'
for subdir, dirs, files in os.walk(dirpath + '/Images/FrontCamera'):
	files.sort()
	# print np.arange(0,np.shape(files)[0],10).tolist()
	files=np.array(files)
	for file in files[np.arange(0,np.shape(files)[0],10)]:
		filepath = subdir + os.sep + file
		if filepath.endswith(".jpg") or filepath.endswith(".pgm") or filepath.endswith(".png") or filepath.endswith(".ppm"):
			print filepath
			imgname=filepath
			# load image

			img = cv2.imread(imgname)
			start_t=time()

			frame = img.copy()
			median = cv2.medianBlur(frame,5)
			# median = cv2.medianBlur(median,5)


			outmask = GMM(median,thresh,k,mean,cov,pi)
			
			
			mask= outmask
			center, inner_corners, outer_corners= applyCorners2Mask(mask,img)
			print("--- %s full operation ---" % (time() - start_t))

			fin_img=drawCorners(center, inner_corners, outer_corners,img)
			
			cv2.imshow('Raw',img)
			cv2.imshow('mask',outmask)
			cv2.imshow('fin img',fin_img)






			cv2.waitKey()
			cv2.destroyAllWindows()
