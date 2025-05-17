#!/usr/bin/env python

#common stuff
from __future__ import print_function
import pinocchio as pin
from numpy import nan
import math
import time as tm

from utils.common_functions import *
from utils.ros_publish import RosPub

import simple_conf as conf

# Init
os.system("killall rosmaster rviz")
ros_pub = RosPub("HEBI")
robot = getRobotModel("HEBI")

# Get transformation of the end-effector
legs = ["lf", "lr", "rf", "rr"]
wheel_pos = ["front", "rear"]

# Dynamics
q = conf.q0.copy()
qd = conf.qd0.copy()
robot.computeAllTerms(q, qd)

for leg in legs:
    for pos in wheel_pos:
        wheel = f"{leg}_wheel_{pos}" 
        wheel_id = robot.model.getFrameId(wheel)
        wheel_pose = robot.data.oMf[wheel_id]

        print(f"\n{wheel} pose:\n", wheel_pose)
        print("Position (xyz):\n", wheel_pose.translation)
        print("Orientation (rotation matrix):\n", wheel_pose.rotation)

ros_pub.deregister_node()