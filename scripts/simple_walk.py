#!/usr/bin/env python3

#common stuff
from __future__ import print_function
import pinocchio as pin
from numpy import nan
import math
import time as tm
import rospy
from sensor_msgs.msg import JointState

from utils.common_functions import *
from utils.ros_publish import RosPub

import L2_conf as conf

# Init
os.system("killall rosmaster rviz")
ros_pub = RosPub("HEBI")
robot = getRobotModel("HEBI")

# Initialize ROS node
rospy.init_node('simple_walk', anonymous=True)

# Gazebo JointState publisher
joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

time = 0.0
q = conf.q0.copy()
qd = conf.qd0.copy()
qdd = conf.qdd0.copy()
q_des = conf.q0.copy()
qd_des = np.zeros(24)
qdd_des = np.zeros(24)

# Joint limits
joint_min = np.array([-1.5] * len(q))
joint_max = np.array([ 1.5] * len(q))

# Assuming the prismatic joints are at indices 3, 9, 15, and 21
prismatic_joint_indices = [1, 7, 13, 19]  # Adjusted indices for prismatic joints

# Set prismatic joint limits (min: 0.0, max: 0.5)
for idx in prismatic_joint_indices:
    joint_min[idx] = 0.0
    joint_max[idx] = 0.075

# Gait parameters
omega = 2 * np.pi * 0.5       # 0.5 Hz
amp = 0.4                     # amplitude
phase_shift = np.pi          # for trot gait

# Leg joint indices
# Shoulder, Fixed, Prismatic, Pitch, Fixed, Roll
FL,FL1 = [0],[1]   # Front Left
FR,FR1 = [6],[7]   # Front Right
RL,RL1 = [12],[13]   # Rear Left
RR,RR1 = [18],[19]   # Rear Right

while time < conf.exp_duration:
    # Desired shoulder trajectories
    for leg, phase in zip([FL, RR], [0.0, 0.0]):
        for j in leg:
            q_des[j]  = conf.q0[j] + amp * np.sin(omega * time + phase)
            qd_des[j] = amp * omega * np.cos(omega * time + phase)
            qdd_des[j] = -amp * omega**2 * np.sin(omega * time + phase)

    for leg, phase in zip([FR, RL], [phase_shift, phase_shift]):
        for j in leg:
            q_des[j]  = conf.q0[j] + amp * np.sin(omega * time + phase)
            qd_des[j] = amp * omega * np.cos(omega * time + phase)
            qdd_des[j] = -amp * omega**2 * np.sin(omega * time + phase)
    # Desired prismatic trajectories
    for leg, phase in zip([FL1, RR1], [0.0, 0.0]):
        for j in leg:
            q_des[j]  = conf.q0[j] + amp * np.sin(omega / 2 * time + phase)
            qd_des[j] = amp * omega * np.cos(omega /2 * time + phase)
            qdd_des[j] = -amp * omega**2 * np.sin(omega / 2 * time + phase)
    for leg, phase in zip([FR1, RL1], [phase_shift, phase_shift]):
        for j in leg:
            q_des[j]  = conf.q0[j] + amp * np.sin(omega / 2 * time + phase)
            qd_des[j] = amp * omega * np.cos(omega / 2 * time + phase)
            qdd_des[j] = -amp * omega**2 * np.sin(omega / 2 * time + phase)

    # Enforce joint limits on desired position
    q_des = np.clip(q_des, joint_min, joint_max)

    # Dynamics
    robot.computeAllTerms(q, qd)
    M = robot.mass(q)
    h = robot.nle(q, qd)

    # Computed torque control
    tau = M.dot(qdd_des + conf.kp.dot(q_des - q) + conf.kd.dot(qd_des - qd)) + h

    # Forward Dynamics
    qdd = np.linalg.inv(M).dot(tau - h)

    # Euler Integration
    qd += qdd * conf.dt
    q += qd * conf.dt + 0.5 * conf.dt**2 * qdd

    # Enforce joint limits on actual position (safety)
    q = np.clip(q, joint_min, joint_max)

    # Send to RViz
    ros_pub.publish(robot, q, qd, tau)

    # Publish to Gazebo
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = robot.model.names[1:]  # Exclude the base joint
    joint_state_msg.position = q.tolist()
    joint_state_msg.velocity = qd.tolist()
    joint_state_msg.effort = tau.tolist()
    joint_state_pub.publish(joint_state_msg)

    tm.sleep(conf.dt * conf.SLOW_FACTOR)
    time += conf.dt

    if ros_pub.isShuttingDown():
        print("Shutting down")
        break

ros_pub.deregister_node()