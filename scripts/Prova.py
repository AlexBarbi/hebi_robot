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

# 1. Convert Xacro to URDF string
xacro_path = '/home/moska002/itr_ws/src/hebi_robot/urdf/HEBI.urdf.xacro'
doc = xacro.process_file(xacro_path)
urdf_xml = doc.toprettyxml(indent='  ')

# 2. Save the URDF string to a temporary file
with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as urdf_file:
    urdf_file.write(urdf_xml.encode('utf-8'))
    urdf_path = urdf_file.name

# 3. Load the model in Pinocchio
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

# 4. (Optional) Display model info
print("Model name:", model.name)
print("Number of joints:", model.njoints)

# Step 2: Initialize Node and Publisher
rospy.init_node("fk_visualizer")
joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

# Define joints you want to animate
joint_names = [model.names[i] for i in range(model.njoints)]  # Replace with your robot's joint names
print(joint_names)

q = np.array([0.0, -1.57, 1.57, 0.0, 1.57, 0.0, 0.0, -1.57, 1.57, 0.0, 1.57, 0.0, 0.0, -1.57, 1.57, 0.0, 1.57, 0.0, 0.0, -1.57, 1.57, 0.0, 1.57, 0.0])  # in radians
# pin.forwardKinematics(model, data, q)

ee_list = []
for ee in ee_list:
    ee_id = model.getFrameId(ee)
    ee_pose = data.oMf[ee_id]

joint_state = JointState()
joint_state.header = Header()
joint_state.header.stamp = rospy.Time.now()
joint_state.name = joint_names
print(len(joint_names))

joint_state.position = [0.2] * 25
joint_state.velocity = [0.0]
joint_state.effort = [0.0]

joint_pub.publish(joint_state)