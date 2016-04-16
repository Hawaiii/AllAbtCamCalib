import imu
import exp_util as util
import vis

import numpy as np
import cv2
import math
import matplotlib.pyplot as plt

imu_motion = imu.read_motion('results/eric_pose.csv', 100)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# ax = vis.plot_poses(imu_motion, fax=ax, invert=True, clr='g')
cm = plt.get_cmap('plasma')
for i, p in enumerate(imu_motion):
	ax = p.plot(ax, clr=cm(1.*i/(len(imu_motion)-1)), length=0.2)

# ax.set_aspect('equal')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_ylim(-0.7, -0.3)
ax.set_zlabel('z')

plt.show()