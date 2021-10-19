#!/usr/bin/env python3

from std_msgs.msg import Float32
import cv2
import rospy
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
#cap.set(cv2.CAP_PROP_BRIGHTHNESS, 0)
cap.set(cv2.CAP_PROP_CONTRAST, 0)

if __name__ == "__main__":

    rospy.init_node('card_detect')
    pub = rospy.Publisher('/card_detect', Float32, queue_size=10)
    rate = rospy.Rate(10)

    while(cap.isOpened() and not rospy.is_shutdown()):
        ret, frame = cap.read()
        if ret == True:
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            lower_red = np.array([110, 0 , 0])
            upper_red = np.array([130, 255, 255])
            mask = cv2.inRange(hsv, lower_red, upper_red)
            #print(hsv[240][320])
            
            lower_red = np.array([45, 0, 0])
            upper_red = np.array([70, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red, upper_red)
            #cv2.imshow('Frame', mask2)
            #cv2.waitKey(1) 
            
            red = np.sum(mask.astype('bool')) / np.shape(mask)[0] / np.shape(mask)[1]
            green = np.sum(mask2.astype('bool')) / np.shape(mask2)[0] / np.shape(mask2)[1]
            #print('Red: ', red)
            #print('Green: ', green)
            pub.publish(red - green)
        else: 
            break
            
        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

