import pinocchio as pin
# from pinocchio.robots import RobotWrapper
# from pinocchio.utils import zero
import numpy as np

import xacro
import tempfile
import pinocchio as pin
from pathlib import Path

# 1. Convert Xacro to URDF string
xacro_path = '/root/itr_ws/src/intro_robotics_labs/urdf/HEBI.urdf.xacro'
doc = xacro.process_file(xacro_path)
urdf_xml = doc.toprettyxml(indent='  ')

# 2. Save the URDF string to a temporary file
with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as urdf_file:
    urdf_file.write(urdf_xml.encode('utf-8'))
    urdf_path = urdf_file.name

# 3. Load the model in Pinocchio
robot = pin.RobotWrapper.BuildFromURDF(urdf_path, [str(Path(xacro_path).parent)])

# Save the URDF to a file for future use
saved_urdf_path = '/root/itr_ws/src/intro_robotics_labs/urdf/generated_HEBI.urdf'
with open(saved_urdf_path, 'w') as saved_urdf_file:
    saved_urdf_file.write(urdf_xml)

print(f"URDF saved to: {saved_urdf_path}")



model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

# 4. (Optional) Display model info
print("Model name:", model.name)
print("Number of joints:", model.njoints)
print("Joint names:", [model.names[i] for i in range(model.njoints)])

urdf_model_path = urdf_path  # Path to the URDF file

# Load the model
model = pin.buildModelFromUrdf(urdf_model_path)
data = model.createData()

# Set joint configuration (q) – must match model.nq
# Example: 6 DOF manipulator
q = np.zeros(24)  # in radians

# Compute forward kinematics
pin.forwardKinematics(model, data, q)

# Get transformation of the end-effector
ee_frame_name = "lr_foot"  # Replace with your robot's EE frame name
ee_id = model.getFrameId(ee_frame_name)
ee_pose = data.oMf[ee_id]

print("End-effector pose:\n", ee_pose)
print("Position (xyz):", ee_pose.translation)
print("Orientation (rotation matrix):\n", ee_pose.rotation)