"""
Makes synthetic data for Kalibr.
"""
import sys
sys.path.append('../calib/')

import imu
import camera as cam
import targets
import exp_util as util
import board as bd
import vis

import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
from cycler import cycler

"""
IMU
"""
imu_motion = imu.circle_motion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# cm = plt.get_cmap('Blues')
# for i, p in enumerate(imu_motion):
# 	ax = p.plot(ax, clr=cm(1.*i/(len(imu_motion)-1)), length=0.02)
# ax.set_aspect('equal')
# plt.show()

