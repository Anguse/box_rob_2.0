#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

#Width and height of frame
frame_width = 410
frame_height = 308
#Determine if width and height really is 640x480 center of image.
frameCentX = int(frame_width/2)
frameCentY = int(frame_height/2)

# publishers
pub = rospy.Publisher('box_digit', String, queue_size = 10)
box_pub = rospy.Publisher('box_location', String, queue_size=10)
collected_pub = rospy.Publisher('box_collected', String, queue_size=10)
bridge = CvBridge()

def cam_cb(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #frame = bridge.imgmsg_to_cv2(msg.data, desired_encoding='passthrough')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kernel = np.ones((9,9), np.uint8)
    

    lower_blue = np.array([90, 125, 0])
    upper_blue = np.array([180, 255, 255])

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    res = cv2.bitwise_and(frame,frame, mask= blue_mask)
    
    kernel = np.ones((15,15),np.float32)/225
    smoothed = cv2.filter2D(res,-1,kernel)
    median = cv2.medianBlur(res,15)
    median_gray = cv2.medianBlur(gray, 15)
    edges = cv2.Canny(median, 30, 200)
    
    

    _, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 0:
        c = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)
        #Moments of contour, to calculate distance from center for the contour
	if cv.contourArea(c) > 45000:
	    collected_pub.publish("collected")
        M = cv2.moments(c)
        #
        cont_area = M["m00"]
        #Calculate x and y value of contour
        cX = 0 
        cY = 0
        if cont_area > 400:
            cX = int(M["m10"] / cont_area)
            cY = int(M["m01"] / cont_area)
        #How far from the center is the object located
        distFromCen = frameCentX - cX
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
                                    param1=200,param2=26,minRadius=10,maxRadius=250)

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
	pub.publish('none')
	box_pub.publish('No box visible')
    
    
    


def main():
    rospy.init_node('cam_sub')
    cam_sub = rospy.Subscriber('/raspicam_node/image/compressed/', CompressedImage, cam_cb)
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
	r.sleep()

if __name__ == '__main__':
    main()
