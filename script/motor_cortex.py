#!/usr/bin/env python3

import threading
import time
from std_msgs.msg import Int8, String
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897
velocity_publisher = ''

def dog_happy():
    # Receiveing the user's input
    print("Happy dog pattern activated!")
    speed = 50 #input("Input your speed (degrees/sec):")
    angle = 50 #input("Type your distance (degrees):")
    clockwise = True #input("Clockwise?: ") #True or false

    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    #We wont use linear components
    vel_msg = Twist()

    repeat = 5

    for r in range(0, repeat):
        # Clockwise
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        vel_msg = Twist()
        vel_msg.angular.z = -abs(angular_speed)
        while(current_angle < relative_angle):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        # Return to Start position
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        vel_msg = Twist()
        vel_msg.angular.z = abs(angular_speed)
        while(current_angle < relative_angle):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        # Anti-Clockwise
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        vel_msg = Twist()
        vel_msg.angular.z = abs(angular_speed)
        while(current_angle < relative_angle):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        # Return to Start position
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        vel_msg = Twist()
        vel_msg.angular.z = -abs(angular_speed)
        while(current_angle < relative_angle):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

    # Rotating
    speed = 70
    angle = 360
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    vel_msg = Twist()
    vel_msg.angular.z = -abs(angular_speed)
    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    #Forcing our robot to stop
    vel_msg = Twist()
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    #rospy.spin()


def mt_clbk(data):
    rospy.loginfo("Pattern = %s", data.data)


if __name__ == '__main__':
    try:
        #Starts a new node
        rospy.init_node('motor_cortex', anonymous=True)
        velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/mt_pattern", String, mt_clbk)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass