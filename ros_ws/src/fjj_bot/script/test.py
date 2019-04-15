
import numpy as np

def pose_to_angular(pose):
    r = np.sqrt(pose[0]**2 + pose[1]**2)
    th = np.arctan2(pose[1], pose[0])
    z = pose[2]
    return (r, th, z)

def angular_to_pose(angular):
    x = angular[0]*np.cos(angular[1])
    y = angular[0]*np.sin(angular[1])
    z = angular[2]
    return (x, y , z)

print(angular_to_pose(pose_to_angular((1,2,3))))