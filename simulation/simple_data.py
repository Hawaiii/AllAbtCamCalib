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

def circle_motion():
	"""
	A hard coded motion.
	Returns a list of time-stamped poses.
	"""
	poses = []
	imu_sample_rate = 100 #100Hz
	time_length = 30

	start_a = -8 * np.pi
	end_a = 8 * np.pi
	theta = np.linspace(start_a, end_a, imu_sample_rate*time_length) # 15 seconds of data

	timestamp = [int(1.46065*(10**18) + (10**9)/imu_sample_rate*t) for t in range(len(theta))]

	r = 0.4
	x = r * np.cos(theta)
	y = r * np.sin(theta)
	# z = -2*r * np.ones(theta.shape)
	z = np.linspace(-3,-0.5, imu_sample_rate*time_length)

	ra = math.pi/8
	dtheta_dt = (end_a-start_a)/time_length

	for i in xrange(len(theta)):
		loc = np.array( [x[i], y[i], z[i]] ).reshape(3,1)
		ori = np.array([[np.cos(ra)*np.cos(theta[i]), np.cos(ra)*np.sin(theta[i]), np.sin(ra)], \
						[-np.sin(theta[i]), np.cos(theta[i]), 0], \
						[-np.sin(ra)*np.cos(theta[i]), -np.sin(ra)*np.sin(theta[i]), np.cos(ra)]]).T
		# lvel = np.array( [-y[i], x[i], 0] ).reshape(3,1)
		lvel = np.array( [-y[i], x[i], -1.5-(-3)/time_length] ).reshape(3,1)
		
		#angular velocity
		avel = np.array( [0, 0, dtheta_dt] )
		lacc = np.array( [-x[i], -y[i], 0] ).reshape(3,1)
		aacc = np.array( [0, 0, 0]).reshape(3,1)
		pose = util.Pose(loc, ori, timestamp[i], lvel=lvel, avel=avel, lacc=lacc, aacc=aacc)
		poses.append(pose)

	# ra = math.pi/6
	# rx = -ra * np.sin(theta)
	# ry = -ra * np.cos(theta)
	# rz = np.zeros(theta.shape)

	# for i in xrange(len(theta)):
	# 	loc = np.array( [x[i], y[i], z[i]] ).reshape(3,1)
	# 	ori = np.array( [rx[i], ry[i], rz[i]] ).reshape(3,1)
	# 	lvel = np.array( [-y[i], x[i], 0] ).reshape(3,1)
	# 	avel = np.array( [ry[i], -rx[i], 0] ).reshape(3,1)
	# 	lacc = np.array( [-x[i], -y[i], 0] ).reshape(3,1)
	# 	aacc = np.array( [-rx[i], -ry[i], 0]).reshape(3,1)
	# 	pose = util.Pose(loc, ori, timestamp[i], lvel=lvel, avel=avel, lacc=lacc, aacc=aacc)
	# 	poses.append(pose)

	return poses

"""
IMU
"""
imu_motion = circle_motion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
cm = plt.get_cmap('Blues')
for i, p in enumerate(imu_motion):
	ax = p.plot(ax, clr=cm(1.*i/(len(imu_motion)-1)), length=0.2)
# ax.set_aspect('equal')

gravity_in_world = np.array([0,9.81,0])
imu.get_imu_readings(imu_motion, gravity_in_world, save_name='results/imu0.csv')

"""
Camera
"""
rel_loc = np.array([0, 0, 0]).reshape(3,1)
rel_ori = np.array([0, 0, 0]).reshape(3,1)
rel_pose = util.Pose(rel_loc, rel_ori, time=0)
cam_sampling_ratio = 10 # camera samples once when imu samples 10 times
cam_motion = imu.transform_motion(imu_motion, rel_pose, cam_sampling_ratio)
cm = plt.get_cmap('Oranges')
for i, p in enumerate(cam_motion):
	ax = p.plot(ax, clr=cm(1.*i/(len(cam_motion)-1)), length=0.2)

camera = cam.Camera.make_pinhole_camera()
camera.intrinsics.radial_dist = np.zeros((1,3))
camera.intrinsics.tang_dist = np.zeros((1,2))

"""
Board
"""
board_dim = (2, 2)
board_loc = np.array([-0.4, 0.4, 0]).reshape(3,1)
board_ori = np.array([math.pi, 0, 0]).reshape(3,1)
board = bd.Board.gen_calib_board(board_dim, 0.8, board_loc, board_ori, 0)
board.plot(ax, clr='y')

board_img = cv2.imread('data/april_6x6.png')
Hs = camera.calc_homography(cam_motion, board, (board_img.shape[1],board_img.shape[0]))
for i in xrange(len(Hs)):
		targets.render_chessboard_homo(board_img, Hs[i], camera.scale_size(1), save_name='results/cam0/'+str(cam_motion[i].time)+'.png')
ax.set_xlabel('x')
ax.set_xlim([-0.5,0.5])
ax.set_ylim([-0.5,0.5])
ax.set_zlim([-1.3,-0.3])
plt.show()
