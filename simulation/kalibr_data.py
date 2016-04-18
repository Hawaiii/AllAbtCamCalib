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
imu_motion = imu.read_motion('data/pose.csv', sample_ratio=100)
# gravity_in_target = np.array([0,0,-9.81])
# imu.get_imu_readings(imu_motion, gravity_in_target, save_name='results/imu0.csv')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
cm = plt.get_cmap('Blues')
for i, p in enumerate(imu_motion):
	ax = p.plot(ax, clr=cm(1.*i/(len(imu_motion)-1)), length=0.02)

"""
Camera
"""
# all length in m
rel_loc = np.array([0, 0, 0]).reshape(3,1)
rel_ori = np.array([math.pi, 0, 0]).reshape(3,1) #head pose -z points in the camera view direction
rel_pose = util.Pose(rel_loc, rel_ori, time=0)
cam_sampling_ratio = 50 # camera samples once when imu samples 50 times
cam_motion = imu.transform_motion(imu_motion, rel_pose, cam_sampling_ratio)
cm = plt.get_cmap('Oranges')
for i, p in enumerate(cam_motion):
	ax = p.plot(ax, clr=cm(1.*i/(len(cam_motion)-1)), length=0.1)

camera = cam.Camera.make_pinhole_camera()
camera.intrinsics.radial_dist = np.zeros((1,3))
camera.intrinsics.tang_dist = np.zeros((1,2))

"""
Board 
"""
board_dim = (2, 2)
board_loc = np.array([0, 0, 0]).reshape(3,1)
board_ori = np.array([math.pi, 0, 0]).reshape(3,1)
board = bd.Board.gen_calib_board(board_dim, 0.8, board_loc, board_ori, 0)
board.plot(ax, clr='y')

# board_loc = np.array([0, 0.01, -2.]).reshape(3,1)
# board_dim = (8, 5)
# board_rot_mat = np.array([[-8.7036998831043544e-01, -9.0912931784559947e-02, 4.8393276628352877e-01],\
#  [-1.1424780991646996e-03, -9.8243204946945251e-01, -1.8661715601477852e-01],\
#  [4.9239697215994882e-01, -1.6297885448605659e-01, 8.5497550537902922e-01]])
# assert(abs(np.linalg.det(board_rot_mat) - 1) < 0.01 )
# board_trans_vec = np.array([7.6537016714681638e-01, -3.0420191600265117e-01, -4.5732500607382032e-01])
# board_ext_board_to_camera = cam.Extrinsics.init_with_rotation_matrix(board_trans_vec, board_rot_mat)
# board_pose_world = board_ext.pose()
# board = bd.Board.gen_calib_board(board_dim, 0.068, board_pose.loc, board_pose.ori, 0)

# Generate camera images
# board_img = cv2.imread('data/april_6x6.png')
# Hs = camera.calc_homography(cam_motion, board)
	# np.concatenate( (np.concatenate((np.eye(3), np.array([1500,3000,-1500]).reshape(3,1)), axis=1), \
					# np.array([0.,0.,0.,1]).reshape(1,4)), axis=0) )
# for i in xrange(len(Hs)):
# 		targets.render_chessboard_homo(board_img, Hs[i], camera.scale_size(2), save_name='results/cam0/'+str(cam_motion[i].time_stamp)+'.png')

ax.quiver([0,0,0],[0,0,0],[0,0,0],[1,0,0],[0,1,0],[0,0,1], pivot='tail')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_aspect('equal')
plt.show()

