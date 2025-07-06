import numpy as np
from tf_transformations import euler_from_quaternion

def get_normalized_pose2D(initial_pose, current_pose):
    # Check if the initial pose is set
    if initial_pose:
        x, y, yaw = current_pose
        init_x, init_y, init_yaw = initial_pose

        # Markov assumption, Just return the previous state
        x -= init_x
        y -= init_y

        # Adjust orientation to lief between -180 to 180 (Euler angle wrapping)
        yaw -= init_yaw
        yaw = normalize_angle(yaw) 

        return (x, y, yaw)
    else:
        return (0.0, 0.0, 0.0)  # Default pose if initial pose not set
    

def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

def get_yaw_from_quaternion(quaternion):
    rpy = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return rpy[2]

odom_pose = lambda odom: (
    odom.pose.pose.position.x,
    odom.pose.pose.position.y,
    get_yaw_from_quaternion(odom.pose.pose.orientation)
)
