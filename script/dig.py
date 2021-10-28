#!/usr/bin/env python

import threading
import rospy as ros
import time
from std_msgs.msg import Int8

def thread1():
    lock = threading.Lock()
    while True:
        with lock:
            s = input()
            if s:
                print('Key:', s)


#threading.Thread(target = thread1).start()
import time
import threading


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

    while loop and not ros.is_shutdown():
        curr_millis = time.time() * 1000
        if (curr_millis - prev_millis) >= 1000:
            print("Another second passed..." + str(curr_millis) + "\r")
            prev_millis = curr_millis
            # Do some extra stuff here

        if data_ready.isSet():
            if key_pressed[0].lower() == "q":
                kill_flag.set()
                loop = False
            else:
                print("You pressed: " + key_pressed[0])
            data_ready.clear()
        if key_pressed[0]:
            if key_pressed[0] == '':
                coin = 0
            else:
                coin = int(key_pressed[0])
        else:
            coin = 0
        print(coin)
        pub.publish(coin)
        rate.sleep()

'''
class KeyboardInput(threading.Thread):
    def init(self, input_cbk=None, name='keyboard-input-thread'):
        super(KeyboardInput, self).init(name=name)
        self.input_cbk = input_cbk
        self.start()

    def run(self):
        while True:
            self.input_cbk(input())'''


global s
def callback(key):
    s = key
    print('Key:', key)


s = ""
if __name__ == "__main__":
    main()
    #keyboard = KeyboardInput()