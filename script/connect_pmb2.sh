#!/bin/bash

## Set rosmaster IRI
echo "Connecting to PMB2..."
export ROS_MASTER_URI=http://192.168.212.67:11311

## Getting local IP of ENO1 interface
eno1_addr=$(ifconfig eno1 | sed -En -e 's/.*inet ([0-9.]+).*/\1/p')
printf "Local Machine IP = %s\n" "$eno1_addr"

## Set ROS_IP
export ROS_IP=$eno1_addr
echo "Status = Connected"