#!/usr/bin/env python3

import threading
import rospy as ros
import time
from std_msgs.msg import Int8


data_ready = threading.Event()
kill_flag = threading.Event()

key_pressed = [""]


def keyboard_poller():
    loop = True

    while loop:
        time.sleep(0.1)

        if kill_flag.isSet():
            loop = False

        ch = input(">")
        if ch:
            key_pressed[0] = ch
            data_ready.set()


def main():
    ros.init_node('digestion')
    pub = ros.Publisher('/digestion', Int8, queue_size=10)
    rate = ros.Rate(10)
    curr_millis = time.time() * 1000
    prev_millis = curr_millis

    poller = threading.Thread(target=keyboard_poller)
    poller.start()

    loop = True
    coin = 0

    while loop and not ros.is_shutdown():
        curr_millis = time.time() * 1000
        if (curr_millis - prev_millis) >= 1000:
            prev_millis = curr_millis

        if data_ready.isSet():
            if key_pressed[0].lower() == "q":
                kill_flag.set()
                loop = False
            else:
                try:
                    coin = int(key_pressed[0])
                except ValueError:
                    print(f'Invalid input "{key_pressed[0]}"')
                else:
                    print("New signal: " + key_pressed[0])
            data_ready.clear()
        pub.publish(coin)
        rate.sleep()


if __name__ == "__main__":  # for exit input q then ctrl + c
    main()
