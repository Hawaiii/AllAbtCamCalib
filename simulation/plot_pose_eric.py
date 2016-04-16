import imu
import exp_util as util
import vis

import numpy as np
import cv2
import math
import matplotlib.pyplot as plt

imu_motion = imu.read_motion('results/eric_pose.csv', 1000)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax = vis.plot_poses(imu_motion, fax=ax, invert=True, clr='g')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()