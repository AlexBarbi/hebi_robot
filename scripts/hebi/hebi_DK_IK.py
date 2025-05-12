import numpy as np
from roboticstoolbox import Robot
from spatialmath import SE3
import matplotlib.pyplot as plt

# Load the URDF
robot = Robot.URDF('/root/itr_ws/src/intro_robotics_labs/urdf/HEBI.urdf')  # Make sure the URDF file is in your working directory
print('giacomo se ne è andato in marocoooooo')
# Print basic robot info
print(f"Loaded robot: {robot.name}")
print(f"Number of joints: {robot.n}")
print("Joint names:", robot.links.joint)

# Define a more realistic initial configuration
# Assuming the order is: shoulder1, suspension, foot_pitch, foot_roll for each leg
q_home = np.array([
    0.1,   # lf_shoulder1_joint
    0.02,  # lf_suspention_ext_joint
    0.1,   # lf_foot_joint_pitch
    0.05,  # lf_foot_joint_roll
    
    -0.1,  # lr_shoulder1_joint
    0.02,  # lr_suspention_ext_joint
    -0.1,  # lr_foot_joint_pitch
    0.05,  # lr_foot_joint_roll
    
    0.1,   # rf_shoulder1_joint
    0.02,  # rf_suspention_ext_joint
    0.1,   # rf_foot_joint_pitch
    0.05,  # rf_foot_joint_roll
    
    -0.1,  # rr_shoulder1_joint
    0.02,  # rr_suspention_ext_joint
    -0.1,  # rr_foot_joint_pitch
    0.05   # rr_foot_joint_roll
])

# ---- Forward Kinematics ----
# Calculate FK for all end effectors (feet)
ee_links = ['lf_foot', 'lr_foot', 'rf_foot', 'rr_foot']
for link in ee_links:
    T = robot.fkine(q_home, end=link)
    print(f"\nForward Kinematics for {link}:")
    print(T)

# ---- Inverse Kinematics ----
# Let's try to move the left-front foot to a new position
target_pose = SE3(0.2, 0.15, -0.1) * SE3.Rx(np.pi/4)

# Solve IK for the left-front leg (first 4 joints)
sol = robot.ikine_LM(
    target_pose,
    end='lf_foot',
    q0=q_home[:4],  # Initial guess for this leg
    mask=[1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # Only solve for first 4 joints
)

if sol.success:
    print("\nInverse Kinematics Solution for lf_foot:")
    print(sol.q)
    q_ik = q_home.copy()
    q_ik[:4] = sol.q  # Update only the left-front leg joints
    
    # Visualize the robot
    robot.plot(q_ik, backend='pyplot', block=False)
    plt.title('Robot with IK Solution for Left-Front Foot')
    plt.show()
else:
    print("Inverse kinematics failed to find a solution.")

# ---- Robot Visualization ----
# Plot the robot in home configuration
robot.plot(q_home, backend='pyplot', block=False)
plt.title('Robot in Home Configuration')
plt.show()

# ---- Joint Control Example ----
# Animate a simple walking motion
q_traj = []
for t in np.linspace(0, 2*np.pi, 50):
    q = q_home.copy()
    # Add sinusoidal motion to the shoulders
    q[0] = 0.2 * np.sin(t)  # lf_shoulder
    q[4] = 0.2 * np.sin(t + np.pi)  # lr_shoulder
    q[8] = 0.2 * np.sin(t)  # rf_shoulder
    q[12] = 0.2 * np.sin(t + np.pi)  # rr_shoulder
    q_traj.append(q)

robot.plot(q_traj, backend='pyplot', dt=0.1)