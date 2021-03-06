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

def circle_motion():
	"""
	A hard coded motion.
	Returns a list of time-stamped poses.
	"""
	poses = []
	imu_sample_rate = 100 #100Hz
	time_length = 30

	start_a = -16* np.pi
	end_a = 16* np.pi
	#padding = np.pi / 8

	theta = np.linspace(start_a, end_a, imu_sample_rate*(time_length))
	#theta = np.linspace(start_a, end_a, imu_sample_rate*(time_length-2))
	slope = (end_a-start_a)/(time_length)
	print 'speed', slope
	#front = np.linspace(start_a-padding, start_a, imu_sample_rate)
	#front = front*front*slope/2
	#tend = np.linspace(end_a, end_a+padding, imu_sample_rate)
	#tend = (tend-time_length)*(tend-time_length)*(-slope/2)
        #theta = np.concatenate((front, theta, tend))

	timestamp = [int(1.46065*(10**18) + (10**9)/imu_sample_rate*t) for t in range(len(theta))]

	r = 1 
	zstart = 2
	zend = 0.5
	x = r * np.cos(theta)
	y = r * np.sin(theta)
	# z = -2*r * np.ones(theta.shape)
	z = np.linspace(zstart,zend, imu_sample_rate*time_length)
	offset = 0.3564
	dz_dt = (zend-zstart)/time_length

	ra = math.pi/6
	dtheta_dt = (end_a-start_a)/(time_length) * np.ones((time_length * imu_sample_rate, ))
	#dtheta_dt = (end_a-start_a)/(time_length-2) * np.ones(((time_length-2)*imu_sample_rate,))
	#dtheta_front = slope * np.linspace(0, 1, imu_sample_rate)
	#dtheta_end = slope * np.linspace(1, 0, imu_sample_rate)
	#dtheta_dt = np.concatenate((dtheta_front,dtheta_dt,dtheta_end))
	ddtheta_dt = np.zeros((imu_sample_rate*(time_length), ))
	#ddtheta_dt = np.zeros((imu_sample_rate*(time_length-2), ))
	#ddtheta_front = slope * np.ones((imu_sample_rate,))
	#ddtheta_end = -slope * np.ones((imu_sample_rate,))
	#ddtheta_dt = np.concatenate((ddtheta_front, ddtheta_dt, ddtheta_end))

	for i in xrange(len(theta)):
		loc = np.array( [x[i]+offset, y[i]+offset, z[i]] ).reshape(3,1)
		ori = np.array([[np.cos(ra)*np.cos(theta[i]), np.cos(ra)*np.sin(theta[i]), -np.sin(ra)], \
						[np.sin(theta[i]), -np.cos(theta[i]), 0], \
						[-np.sin(ra)*np.cos(theta[i]), -np.sin(ra)*np.sin(theta[i]), -np.cos(ra)]]).T
		# lvel = np.array( [-y[i], x[i], 0] ).reshape(3,1)
		lvel = np.array( [-y[i]*dtheta_dt[i], x[i]*dtheta_dt[i], dz_dt] ).reshape(3,1)
		
		#angular velocity
		avel = np.array( [0, 0, dtheta_dt[i]] )
		lacc = np.array( [-x[i], -y[i], 0] ).reshape(3,1) * (dtheta_dt[i]**2) #TODO
		aacc = np.array( [0, 0, 0]).reshape(3,1)
		pose = util.Pose(loc, ori, timestamp[i], lvel=lvel, avel=avel, lacc=lacc, aacc=aacc)
		poses.append(pose)

	return poses
"""
IMU
"""
imu_motion = circle_motion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

imu_loc_x = [p.loc[0,0] for p in imu_motion]
imu_loc_y = [p.loc[1,0] for p in imu_motion]
imu_loc_z = [p.loc[2,0] for p in imu_motion]
ax.plot(imu_loc_x, imu_loc_y, imu_loc_z, color='b')
#cm = plt.get_cmap('Blues')
#for i, p in enumerate(imu_motion):
#	ax = p.plot(ax, clr=cm(1.*i/(len(imu_motion)-1)) )
# ax.set_aspect('equal')
board_rot = cv2.Rodrigues(np.array([0., 0,0]))[0]
gravity_in_world = board_rot.dot(np.array([0,-9.81,0]).reshape(3,1))
#gravity_in_world = np.array([0,0,-9.81])
imu.get_imu_readings(imu_motion, gravity_in_world, save_name='results/imu0.csv')

"""
Camera
"""
rel_loc = np.array([0, 0, 0]).reshape(3,1)
rel_ori = np.array([0, 0, np.pi/3]).reshape(3,1)
rel_pose = util.Pose(rel_loc, rel_ori, time=0)
cam_sampling_ratio = 10 # camera samples once when imu samples 10 times
cam_motion = imu.transform_motion(imu_motion, rel_pose, cam_sampling_ratio)
#cm = plt.get_cmap('Oranges')
#for i, p in enumerate(cam_motion):
#	ax = p.plot(ax, clr=cm(1.*i/(len(cam_motion)-1)), length=0.2)
cam_loc_x = [p.loc[0,0] for p in cam_motion]
cam_loc_y = [p.loc[1,0] for p in cam_motion]
cam_loc_z = [p.loc[2,0] for p in cam_motion]
ax.plot(cam_loc_x, cam_loc_y, cam_loc_z, color='r')


camera = cam.Camera.make_pinhole_camera()
camera.intrinsics.radial_dist = np.zeros((1,3))
camera.intrinsics.tang_dist = np.zeros((1,2))

"""
Board
"""
board_dim = (2, 2)
board_loc = np.array([0, 0, 0]).reshape(3,1)
board_ori = np.array([0, 0, 0]).reshape(3,1)
#board = bd.Board.gen_calib_board(board_dim, 0.7128, board_loc, board_rot, 0)
board = bd.Board.gen_calib_board(board_dim, 0.7128, board_loc, board_ori, 0)
board.plot(ax, clr='y')

board_img = cv2.imread('data/april_6x6.png')
#Hs = camera.calc_homography(cam_motion, board, (board_img.shape[1],board_img.shape[0]), board_img, 2, 'results/cam0/')
Hs = camera.calc_homography(cam_motion, board, (board_img.shape[1],board_img.shape[0]), board_img, 2, 'results/cam0/')
#for i in xrange(len(Hs)):
#		targets.render_chessboard_homo(board_img, Hs[i], camera.scale_size(1), save_name='results/cam0/'+str(cam_motion[i].time)+'.png')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
#ax.set_xlim([-0.5,0.5])
#ax.set_ylim([-0.5,0.5])
#ax.set_zlim([-1.3,-0.3])
plt.show()
