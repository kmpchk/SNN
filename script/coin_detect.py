#!/usr/bin/env python3

import rospy as ros
from std_msgs.msg import Int8

if __name__ == "__main__":
    ros.init_node('coin_acceptor')
    pub = ros.Publisher('/coin_acceptor', Int8, queue_size=10)
    rate = ros.Rate(10)
    # rate_secs = ros.Duration(5)
    duration = 1 # ms
    fake_coin = 0
    real_coin = 1
    while not ros.is_shutdown():
        # coin = int(input("Enter a coin:\n"))
        coin = real_coin
        # print("Coin detected: ", coin)
        pub.publish(coin)
        rate.sleep()
        # ros.sleep(duration)
