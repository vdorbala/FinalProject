#!/usr/bin/env python

from __future__ import print_function
import sys
import os
PY3 = sys.version_info[0] == 3
dirpath = os.getcwd()

if PY3:
    xrange = range
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
import rospy
from std_msgs.msg import Empty

import numpy as np
import cv2
import imutils
flag = 0
velocity = Quaternion()
bridge = CvBridge()
mask_pub = rospy.Publisher('/mask', Image, queue_size=1)
vel_pub = rospy.Publisher('/moveto_cmd_body', Quaternion, queue_size=1)
pub_land= rospy.Publisher('bebop/land',Empty,queue_size=1)
def find_circles(my_img):
    global flag 
    img = bridge.imgmsg_to_cv2(my_img, "bgr8")
    
    img_orig=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))

    img_orig = clahe.apply(img_orig)
    img = cv2.medianBlur(img_orig,3)
    ret,thresh_binary = cv2.threshold(img,210,255,cv2.THRESH_BINARY)
    dilate = cv2.dilate(thresh_binary, np.ones((3,3), np.uint8), iterations=1)

    im, cnts, hierarchy = cv2.findContours(dilate, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    drawing = np.zeros((dilate.shape[0], dilate.shape[1], 3), np.uint8)
 
    cnt_area = []
    cnt_num = []
    for c in cnts:
        cnt_area.append(cv2.contourArea(c))

    cnt_num = np.argsort(cnt_area)
    cnt_area.sort()
    # print(cnt_area)
    # large_cnts = np.zeros(np.shape(mask))
    fresh_im = np.zeros(np.shape(img_orig))
    size_im = np.shape(img_orig)
    y_im = int(size_im[0]/2)
    x_im = int(size_im[1]/2)
    for i in range(5): # in the 5 largest contours, check if cnt_area > 5000
        if cnt_area[len(cnt_area)-1-i] > 1000:

            cv2.drawContours(fresh_im, cnts, cnt_num[len(cnt_num)-1-i], (255, 255, 255), -1)
           
    mask = cv2.bitwise_and(img_orig, img_orig, mask = np.uint8(fresh_im))

    detected_circles = cv2.HoughCircles(mask,  
                   cv2.HOUGH_GRADIENT, 1, 20, param1 = 50, 
               param2 = 30, minRadius = 1, maxRadius = 40) 
    
    scale_x = 0.005
    scale_y = 0.005
    # Draw circles that are detected. 
    if detected_circles is not None: 
  
        # Convert the circle parameters a, b and r to integers. 
        detected_circles = np.uint16(np.around(detected_circles)) 
      
        for pt in detected_circles[0, :]: 
            a, b, r = pt[0], pt[1], pt[2] 
           
            # Draw the circumference of the circle. 
            cv2.circle(mask, (a, b), r, (0, 255, 0), 2) 
      
            # Draw a small circle (of radius 1) to show the center. 
            cv2.circle(mask, (a, b), 1, (0, 0, 255), 3)

            # first control in x dirn in image space (y in bebop space)
            del_x = x_im-a # if +ve go left (+y direction bebop)
            del_y = y_im-b
            print("del_x")
            print(del_x)
            print("del_y")
            print(del_y)
            # if the error is greater than 20 pixels, then only change motion in y direction 
            move_x = del_y*scale_y
            move_y = del_x*scale_x

            # if move_x < 0.1 and move_y < 0.1:
            #     velocity.w = 0
            #     velocity.x = 0
            #     velocity.y = 0
            #     velocity.z = 0
            #     print("reached goal")
            #     pub_land.publish()
            #     rospy.signal_shutdown('Node is finished, shut down')
            # else:

            if abs(del_x)>20:
                flag = 0
                velocity.w = 0
                velocity.x = 0
                velocity.y = move_y
                velocity.z = 0
                print("y")
            else:
                flag = 1 
                velocity.w = 0
                velocity.x = 0
                velocity.y = 0
                velocity.z = 0
                print("y done")
            if flag == 1:
                if abs(del_y)>20:   
                    velocity.w = 0
                    velocity.x = move_x
                    velocity.y = 0
                    velocity.z = 0
                    print("x")
                else:
                    flag = 2
                    velocity.w = 0
                    velocity.x = 0
                    velocity.y = 0
                    velocity.z = 0
                    print("x done")
            if flag == 2:
                if abs(del_x) > 10 and abs(del_y)>10:

                    velocity.w = 0
                    velocity.x = move_x
                    velocity.y = move_y
                    velocity.z = 0
                    print("xy")
                else:
                    velocity.w = 0
                    velocity.x = 0
                    velocity.y = 0
                    velocity.z = 0
                    print("done")
                    pub_land.publish()
                    rospy.signal_shutdown('Node is finished, shut down')
            break 
            # cv2.imshow("Detected Circle", mask) 
            # cv2.waitKey(0) 
    mask_pub.publish(bridge.cv2_to_imgmsg(mask, "mono8")) 
    vel_pub.publish(velocity)
    
def main():
    rospy.init_node('bullseye_detection', anonymous=False)
    rospy.Subscriber('/duo3d/left/image_rect', Image, find_circles)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
