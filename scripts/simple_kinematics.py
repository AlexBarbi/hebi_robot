#!/usr/bin/env python

import xacro
import tempfile
import pinocchio as pin
from pathlib import Path

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import numpy as np

from utils.common_functions import *
from utils.ros_publish import RosPub

import simple_conf as conf

# Init
os.system("killall rosmaster rviz")
ros_pub = RosPub("HEBI")
robot = getRobotModel("HEBI")
data = robot.createData()

# Get transformation of the end-effector
legs = ["lf", "lr", "rf", "rr"]
wheel_pos = ["front", "rear"]

for leg in legs:
    for pos in wheel_pos:
        wheel = f"{leg}_wheel_{pos}" 
        wheel_id = robot.getFrameId(wheel)
        wheel_pose = data.oMf[wheel_id]

        print("End-effector pose:\n", wheel_pose)
        print("Position (xyz):", wheel_pose.translation)
        print("Orientation (rotation matrix):\n", wheel_pose.rotation)

ros_pub.deregister_node()