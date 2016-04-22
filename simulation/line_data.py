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
#from cycler import cycler

def line_motion():

	poses = []
	imu_sample_rate = 1000 #100Hz
	time_length = 5 

	start_a = -np.pi/3
	end_a = np.pi/3
	theta = np.linspace(start_a, end_a, imu_sample_rate*time_length)
	dtheta_dt = (end_a - start_a)/time_length
	sine = np.sin(theta)
	cosine = np.cos(theta)

	timestamp = [int(1.46065*(10**18) + (10**9)/imu_sample_rate*t) for t in range(imu_sample_rate*time_length)]

	for i in xrange(len(theta)):
		loc = np.array([0,0,-0.5]).reshape(3,1)
		ori = np.array([[cosine[i], 0, sine[i]],[0,1,0],[-sine[i],0,cosine[i]]])
		lvel = np.zeros((3,1))
		
		#angular velocity
		avel = np.array( [0, dtheta_dt, 0] )
		lacc = np.zeros((3,1))
		aacc = np.zeros((3,1))
		pose = util.Pose(loc, ori, timestamp[i], lvel=lvel, avel=avel, lacc=lacc, aacc=aacc)
		poses.append(pose)

	return poses

"""
IMU
"""
imu_motion = line_motion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
cm = plt.get_cmap('Blues')
#for i, p in enumerate(imu_motion):
#	ax = p.plot(ax, clr='b')
# ax.set_aspect('equal')

gravity_in_world = np.array([0,9.81,0])
imu.get_imu_readings(imu_motion, gravity_in_world, save_name='results/imu0.csv')

"""
Camera
"""
rel_loc = np.array([0, 0, 0]).reshape(3,1)
rel_ori = np.array([0, 0, 0]).reshape(3,1)
rel_pose = util.Pose(rel_loc, rel_ori, time=0)
cam_sampling_ratio = 20 # camera samples once when imu samples 5 times
cam_motion = imu.transform_motion(imu_motion, rel_pose, cam_sampling_ratio)
cm = plt.get_cmap('Oranges')
#for i, p in enumerate(cam_motion):
#	ax = p.plot(ax, clr='r')

camera = cam.Camera.make_pinhole_camera()
camera.intrinsics.radial_dist = np.zeros((1,3))
camera.intrinsics.tang_dist = np.zeros((1,2))

"""
Board
"""
board_dim = (2, 2)
board_loc = np.array([-0.3564, 0.3564, 0]).reshape(3,1)
board_ori = np.array([math.pi, 0, 0]).reshape(3,1)
board = bd.Board.gen_calib_board(board_dim, 0.7128, board_loc, board_ori, 0)
#board.plot(ax, clr='y')

board_img = cv2.imread('data/april_6x6.png')
#Hs = camera.calc_homography(cam_motion, board, (board_img.shape[1],board_img.shape[0]))
Hs = camera.calc_homography(cam_motion, board, (board_img.shape[1],board_img.shape[0]), board_img, 2, 'results/cam0/')
ax.set_xlabel('x')
# ax.set_xlim([-0.5,0.5])
# ax.set_ylim([-0.5,0.5])
# ax.set_zlim([-1.3,-0.3])
plt.show()
