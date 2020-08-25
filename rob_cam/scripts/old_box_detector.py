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

'''========================================================================
*
*This one works really well on a version of the state machine 18/8/2020
* where it refuses to pick up zeros. So Camera is complete(except for tuning)
=========================================================================
'''


while not rospy.is_shutdown():
   # _, frame = cap.read()
    #rospy.loginfo("read image")
    frame = cap.read()
    frame = cv2.flip(frame, -1)
    frame = frame[50:480, 0:640]
    t_zero = False
    hough_zero = False

    contour_area_list = []

    frame = cv2.GaussianBlur(frame, (5,5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([68, 80, 50])
    upper_blue = np.array([120, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask1 = cv2. erode(blue_mask, None, iterations=2)
    mask1 = cv2.dilate(mask1, None, iterations=4)

    mask_inv = cv2.bitwise_and(mask1, mask1,mask=blue_mask )
    mask_inv_hough = cv2.bitwise_and(mask1, mask1, mask=blue_mask)
    mask_inv_hough = mask_inv_hough[0:480, 180:460]
    cnts = cv2.findContours(mask_inv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(cnts) != 0:
        
        nearest_contour = max(cnts, key = cv2.contourArea)

        x, y, w, h = cv2.boundingRect(nearest_contour)

        if cv2.contourArea(nearest_contour) > 17500:
            collected_pub.publish('collected')



        #cv2.rectangle(frame, (x+10,y),(x+w-10,y+h),(0,255,0),2)

        roi = mask_inv[y+10:y + h-5, x+10:x + w-10]
        roi_hough = mask_inv_hough[y+10:y + h-5, x+10:x + w-10]


        cX = 0 
        cY = 0
        dist_from_cen = 0

        M = cv2.moments((nearest_contour))
        cont_area = M["m00"]
        if cont_area <= 0:
            continue
        if  cont_area > 400:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])    
            #How far from the center is the object located
            distFromCen = frameCentX - cX
            #Publish to topic where box is located, will use Centered box as transition to Collect.
            
            if distFromCen > 70:
                box_pub.publish('Box to the left')
            elif distFromCen < -70:
                box_pub.publish('Box to the right')
            else:
                box_pub.publish('Centered box')
            

        if roi.shape[0]>0 and roi.shape[1]>0:
##                    cv2.imshow('roi', cv2.resize(roi,(640,480)))
                    circles = cv2.HoughCircles(mask_inv_hough, cv2.HOUGH_GRADIENT, 1,100, param1=210,param2=16,minRadius=10,maxRadius=300)
                    
                    #circles = np.uint16(np.around(circles))
                  

                    if circles is not None:
                        hough_zero= True
                        pub.publish('zero')
                        for i in circles[0,:]:
                        # draw the outer circle
                            cv2.circle(mask_inv_hough,(i[0],i[1]),i[2],(0,250,0),2)
                        # draw the center of the circle
                            cv2.circle(mask_inv_hough,(i[0],i[1]),2,(0,0,255),3)
                    else:
                        hough_zero= False 
                        pub.publish('one')
                   
    else:
        box_pub.publish("No box visible")
   
                
    #cv2.imshow("Frame", mask_inv_hough)
    #key = cv2.waitKey(5) & 0xFF
    #if key == 27:
    #    break   
    #time.sleep(0.1)

#cv2.destroyAllWindows()
#cap.release()
