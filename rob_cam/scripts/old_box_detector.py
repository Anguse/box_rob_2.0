#!/usr/bin/env python
import rospy
import cv2
import time
import numpy as np
import imutils
import std_msgs
from collections import deque
from imutils.video import VideoStream
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


#time.sleep(2.0)

rospy.init_node('tracker', anonymous=True)
pub = rospy.Publisher('box_digit', String, queue_size = 10)
box_pub = rospy.Publisher('box_location', String, queue_size=10)
cam_raw_pub = rospy.Publisher('camera_raw', Image, queue_size=10)
cam_box_pub = rospy.Publisher('camera_box', Image, queue_size=10)
collected_pub = rospy.Publisher('box_collected', String, queue_size=10)
bridge = CvBridge()

rate = rospy.Rate(10) # 10hz
cap = VideoStream(src=0).start()
#cap = cv2.VideoCapture(0)
#Width and height of frame
#frame_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
#frame_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
frame_width = 640
frame_height = 480
#Determine if width and height really is 640x480 center of image.
frameCentX = int(frame_width/2)
frameCentY = int(frame_height/2)


while not rospy.is_shutdown():
   # _, frame = cap.read()
    #rospy.loginfo("read image")
    frame = cap.read()
    frame = cv2.flip(frame, -1)
    frame = frame[20:640, 0:480]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kernel = np.ones((9,9), np.uint8)
    

    lower_blue = np.array([68, 80, 50]) # early=[68, 80, 50], mid = []
    upper_blue = np.array([148, 255, 255]) # early=[112, 255, 255], mid = [148, 255, 255]

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    res = cv2.bitwise_and(frame,frame, mask= blue_mask)
    
    kernel = np.ones((15,15),np.float32)/225
    smoothed = cv2.filter2D(res,-1,kernel)
    median = cv2.medianBlur(res,15)
    median_gray = cv2.medianBlur(gray, 15)
    edges = cv2.Canny(median, 30, 200)
    #cam_raw_pub.publish(bridge.cv2_to_imgmsg(frame, encoding="passthrough"))
    #cam_box_pub.publish(bridge.cv2_to_imgmsg(median, encoding="passthrough"))
    
    

    _, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 0:
        c = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)
        #Moments of contour, to calculate distance from center for the contour
        if cv2.contourArea(c) > 10000:
           collected_pub.publish('collected')
        M = cv2.moments(c)
        #
        cont_area = M["m00"]
	if cont_area <= 0:
	    continue
        #Calculate x and y value of contour
        #if cont_area > 400:
    	cX = int(M["m10"] / cont_area)
        cY = int(M["m01"] / cont_area)
        #How far from the center is the object located
        distFromCen = frameCentX - cX
	rospy.loginfo("cont_area: "+str(cont_area))
	rospy.loginfo("distFromCen: "+str(distFromCen))
	rospy.loginfo("contourArea(c): "+str(cv2.contourArea(c)))
        #Publish to topic where box is located, will use Centered box as transition to Collect.
        if distFromCen > 80:
            box_pub.publish('Box to the left')
        elif distFromCen < -80:
            box_pub.publish('Box to the right')
        else:
            box_pub.publish('Centered box')

        
        if cv2.contourArea(c) > 500:
            try:
                circles = cv2.HoughCircles(median_gray,cv2.HOUGH_GRADIENT,1,80,
                                    param1=200,param2=35,minRadius=20,maxRadius=250)

                circles = np.uint16(np.around(circles))
                for i in circles[0,:]:
                    # draw the outer circle
                    cv2.circle(median,(i[0],i[1]),i[2],(0,255,0),2)
                    # draw the center of the circle
                    cv2.circle(median,(i[0],i[1]),2,(0,0,255),3)
                pub.publish('zero')
            except:
                pub.publish('one')
    else:
	box_pub.publish("No box visible")
	
    #cv2.imshow('Median Blur',median)
    
   # k = cv2.waitKey(5) & 0XFF
   # if k == 27:
   #     break

cv2.destroyAllWindows()
cap.release()
