"""
Makes synthetic data for Kalibr.
"""
import sys
sys.path.append('../calib/')

import imu
import camera as cam
import targets
import vis
import exp_util as util
import board as bd

import numpy as np
import cv2
import math
import matplotlib.pyplot as plt

imu_motion = imu.read_motion('data/pose.csv')
gravity_in_target = np.array([0,0,-9.81])
imu.get_imu_readings(imu_motion, gravity_in_target, save_name='results/imu0.csv')

# all length in m
rel_pose = cam.Extrinsics.init_with_rotation_matrix(np.array([0.2,0.,0.]).reshape(3,1), np.eye(3), time_stamp=None)
cam_sampling_ratio = 50 # camera samples once when imu samples 50 times
cam_motion = imu.transform_motion(imu_motion, rel_pose, cam_sampling_ratio)

camera = cam.Camera.make_pinhole_camera()
camera.intrinsics.radial_dist = np.zeros((1,3))
camera.intrinsics.tang_dist = np.zeros((1,2))

board_dim = (2, 2)
board_loc = np.array([0, 0.01, -2.]).reshape(3,1)
board_orient = np.array([0,0, math.pi/2])
board = bd.Board.gen_calib_board(board_dim, 0.8, board_loc, board_orient, 0)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax = vis.plot_poses(cam_motion, invert=True, clr='r')
# ax = vis.plot_poses(imu_motion, invert=True, fax=ax, clr='g')
# ax = board.plot(ax, clr='b')

board0 = bd.Board.gen_calib_board(board_dim, 0.8, board_loc, np.zeros((1,3)), 0)
ax = board0.plot(ax, clr='r')
board_o1 = np.array([0,math.pi/2,0])
board1 = bd.Board.gen_calib_board(board_dim, 0.8, board_loc, board_o1, 0)
# ax = board1.plot(ax, clr='y')



ax.set_aspect('equal')

# Generate camera images
# board_img = cv2.imread('data/april_6x6.png')
# Hs = camera.calc_homography(cam_motion, board, board_dim, \
# 	# np.concatenate( (np.concatenate((np.eye(3), np.array([1500,3000,-1500]).reshape(3,1)), axis=1), \
# 					# np.array([0.,0.,0.,1]).reshape(1,4)), axis=0) )
# 							None)
# for i in xrange(len(Hs)):
# 		targets.render_chessboard_homo(board_img, Hs[i], camera.scale_size(2), save_name='results/cam0/'+str(cam_motion[i].time_stamp)+'.png')
plt.show()



# board_dim = (2, 2)
# board = util.gen_calib_board(board_dim[0], board_dim[1], 800, np.array([-1500,-1500,1500]).reshape(3,1), np.zeros([3,1]), 0)
# # board_dim = (8, 5) #DONT CHANGE as this is the dimension of data/chess.png
# # board = util.gen_calib_board(board_dim[0], board_dim[1], 23, np.array([0,0,600]).reshape(3,1), np.zeros([3,1]), 0)

# camera = cam.Camera.make_pinhole_camera()
# camera.intrinsics.radial_dist = np.zeros((1,3))
# camera.intrinsics.tang_dist = np.zeros((1,2))
# print camera

# cam_motion = imu.read_motion('data/pose.csv')
# #cam_motion = imu.spiral_motion(board, board_dim, camera)
# cam_sampling_ratio = 50 # camera samples once when imu samples 10 times


# # Generate camera images
# board_img = cv2.imread('data/april_6x6.png')
# # board_img = cv2.imread('data/chess.png')
# #Hs = camera.calc_homography(cam_motion, board, board_dim, \
# #	np.concatenate( (np.concatenate((np.eye(3), np.array([0,0,-600]).reshape(3,1)), axis=1), \
# #					np.array([0.,0.,0.,1]).reshape(1,4)), axis=0) )
# #for i in xrange(len(Hs)):
# #	if i % cam_sampling_ratio == 0:
# 		# targets.render_chessboard_homo(board_img, Hs[i], camera.scale_size(2), save_name='results/cam0/'+str(cam_motion[i].time_stamp)+'.png')
# #		targets.render_chessboard_homo(board_img, Hs[i], camera.scale_size(2), save_name='results/cam0/'+str(cam_motion[i].time_stamp)+'.png')

# # Relative pose from camera to imu
# rel_pose = cam.Extrinsics.init_with_rotation_matrix(np.array([-0.,0.,-0.]).reshape(3,1), np.eye(3), time_stamp=None)

# # Generate imu motion
# imu_motion = imu.transform_motion(cam_motion, rel_pose)
# ax = vis.plot_poses(cam_motion, invert=True)
# ax = vis.plot_poses(imu_motion, invert=True, fax=ax)
# ax = vis.plot_calib_boards( [ board ], board_dim, fax=ax)
# ax.set_aspect('equal')
# plt.show()

# gravity_in_target = np.array([0,0,-9.81])
# imu.gen_imu_readings(imu_motion, gravity_in_target, save_name='results/imu0.csv')
