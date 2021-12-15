#!/usr/bin/env python3

from std_msgs.msg import Float32
import cv2
import rospy
import numpy as np

cap = cv2.VideoCapture(2)  # /dev/video4

#cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
#cap.set(cv2.CAP_PROP_BRIGHTHNESS, 0)
#cap.set(cv2.CAP_PROP_CONTRAST, 0)

if __name__ == "__main__":

    rospy.init_node('card_detect')
    pub_red = rospy.Publisher('/card_detect/red', Float32, queue_size=10)
    pub_green = rospy.Publisher('/card_detect/green', Float32, queue_size=10)
    rate = rospy.Rate(10)

    while (cap.isOpened() and not rospy.is_shutdown()):
        ret, frame = cap.read()

        if not ret:
            break


        ret, frame = cap.read()

        cv2.imshow('frame', frame)

        # RED
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        lower_red = np.array([118, 0, 0])
        upper_red = np.array([128, 255, 255])
        red_mask = cv2.inRange(hsv, lower_red, upper_red)
        cv2.imshow('RED mask', red_mask)
        #print(np.shape(hsv))
        square = np.shape(hsv)[0] * np.shape(hsv)[1]
        print(hsv[240][320])

        # GREEN
        lower_green = np.array([42, 100, 20])
        upper_green = np.array([60, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        cv2.imshow('GREEN mask', green_mask)

        # Calculate count
        red_sum = np.sum(red_mask.astype("bool")) / square
        green_sum = np.sum(green_mask.astype("bool")) / square
        red = red_sum > 0.1
        green = green_sum > 0.1
        pub_red.publish(red_sum)
        pub_green.publish(green_sum)

        # Publish to ROS topic
        print('Red: ', red_sum)
        print('Green: ', green_sum)
        """
        if red and green:
            if red_sum > green_sum:
                pub.publish(-1)
            else:
                pub.publish(1)
        elif red:
            pub.publish(-1)
        elif green:
            pub.publish(1)
        else:
            pub.publish(0)
        """
        # Display image
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()
