import board
import camera as cam
import exp_util as util
import vis

import numpy as np
import math
# from mpl_toolkits.mplot3d.axes3d import Axes3D
import matplotlib.pyplot as plt

import unittest

class TestBoardMethods(unittest.TestCase):

	def test_gen_calib_board(self):
		# Board at (0,0,0) on x-y plane
		b0 = board.Board.gen_calib_board((3,2),1,np.array([0,0,0]), np.array([0,0,0]),0)
		print b0

		# Rotate 30 degrees around x axis
		# Points should be at (0,0,0), (1,0,0), (0,sqrt(3)/2,-1/2), (1,sqrt(3)/2,-1/2)
		bx30 = board.Board.gen_calib_board((3,2),1,np.array([0,0,0]), np.array([math.pi/6,0,0]),0)
		print bx30

		# Rotate 30 degrees around y axis
		# Points should be at (0,0,0), (sqrt(3)/2,0,-1/2), (0,1,0), (sqrt(3)/2,1,-1/2)
		by30 = board.Board.gen_calib_board((3,2),1,np.array([0,0,0]), np.array([0, math.pi/6,0]),0)
		print by30

		# Rotate 30 degrees around z axis
		# Points should be at (0,0,0), (sqrt(3)/2,-1/2,0), (1/2,sqrt(3)/2,0), (sqrt(6)/2,sqrt(2)/2,0)
		bz30 = board.Board.gen_calib_board((3,2),1,np.array([0,0,0]), np.array([0,0,math.pi/6]),0)
		print bz30

	# def test_gen_calib_board_with_plot(self):
	# 	fig = plt.figure()
	# 	ax = fig.add_subplot(111, projection='3d')

	# 	# Board at (0,0,0) on x-y plane
	# 	b0 = board.Board.gen_calib_board((3,2),1,np.array([0,0,0]), np.array([0,0,0]),0)
	# 	b0.plot(ax, clr='r')

	# 	# Rotate 30 degrees around x axis
	# 	# Points should be at (0,0,0), (1,0,0), (0,sqrt(3)/2,1/2), (1,sqrt(3)/2,1/2)
	# 	bx30 = board.Board.gen_calib_board((3,2),1,np.array([0,0,0]), np.array([math.pi/6,0,0]),0)
	# 	bx30.plot(ax, clr='y')

	# 	# Rotate 30 degrees around y axis
	# 	# Points should be at (0,0,0), (sqrt(3)/2,0,-1/2), (0,1,0), (sqrt(3)/2,1,-1/2)
	# 	by30 = board.Board.gen_calib_board((3,2),1,np.array([0,0,0]), np.array([0, math.pi/6,0]),0)
	# 	by30.plot(ax, clr='g')

	# 	# Rotate 30 degrees around z axis
	# 	# Points should be at (0,0,0), (sqrt(3)/2,-1/2,0), (1/2,sqrt(3)/2,0), (sqrt(6)/2,sqrt(2)/2,0)
	# 	bz30 = board.Board.gen_calib_board((3,2),1,np.array([0,0,0]), np.array([0,0,math.pi/6]),0)
	# 	bz30.plot(ax, clr='b')

	# 	# plt.show()

	def test_get_points(self):
		#@TODO
		pass

	def test_get_points_homo(self):
		#@TODO
		pass

	# def test_get_four_corners(self):
	# 	# Board at (0,0,0) on x-y plane
	# 	b0 = board.Board.gen_calib_board((3,2),1,np.array([0,0,0]), np.array([0,0,0]),0)
	# 	b0_corners = b0.get_four_corners()
	# 	assert(np.array_equal(b0_corners, np.array([[0,0,0],[0,2,0],[1,0,0],[1,2,0]]).T ))
		
	# 	# Rotate 30 degrees around x axis
	# 	# Points should be at (0,0,0), (1,0,0), (0,sqrt(3)/2,1/2), (1,sqrt(3)/2,1/2)
	# 	bx30 = board.Board.gen_calib_board((2,2),1,np.array([0,0,0]), np.array([math.pi/6,0,0]),0)
	# 	bx30_corners = bx30.get_four_corners()
	# 	assert(np.allclose(bx30_corners, np.array([[0,0,0],[0,math.sqrt(3)/2,0.5],\
	# 											[1,0,0],[1,math.sqrt(3)/2, 0.5]]).T ))
		
	# 	# Rotate 30 degrees around y axis
	# 	# Points should be at (0,0,0), (sqrt(3)/2,0,-1/2), (0,1,0), (sqrt(3)/2,1,-1/2)
	# 	by30 = board.Board.gen_calib_board((2,2),1,np.array([0,0,0]), np.array([0, math.pi/6,0]),0)
	# 	by30_corners = by30.get_four_corners()
	# 	assert(np.allclose(by30_corners, np.array([[0,0,0],[0,1,0],\
	# 											[math.sqrt(3)/2,0,-0.5],[math.sqrt(3)/2, 1, -0.5]]).T ))

	# 	#@TODO test non-zero translation cases
	# 	pass

	# def test_dict2array(self):
	# 	#@TODO
	# 	pass

	# def test_random_board(self):
	# 	depth_min = 1 #m
	# 	depth_max = 4 #m
	# 	bw = 8
	# 	bh = 5
	# 	bs = 0.23
	# 	noise3d = 0
	# 	noise2d = 0
	# 	B = board.Board.gen_calib_board((bw, bh), bs, \
	# 							np.zeros((3,1)), np.zeros((3,1)), noise3d)

	# 	true_cam = cam.Camera.make_pinhole_camera()
	# 	cam_loc = np.zeros((3,1))
	# 	cam_ori = np.zeros((3,1))
	# 	cam_extrin = util.Pose(cam_loc, cam_ori).extrinsics()

	# 	pxl_x = np.random.random_integers(0, true_cam.width()-1)
	# 	pxl_y = np.random.random_integers(0, true_cam.height()-1)

	# 	# choose a random depth on ray from pixel
	# 	pt3d, ray_vec = true_cam.ray_from_pixel((pxl_x, pxl_y), cam_extrin)
	# 	depth = np.random.rand() * (depth_max - depth_min) + depth_min
	# 	bd_loc = pt3d + depth*ray_vec
		
	# 	# choose a random orientation
	# 	bd_ori = util.random_rotation()

	# 	B.move_board(bd_loc, bd_ori)
	# 	img_pts = true_cam.capture_images(cam_extrin, [B.get_points()], noise2d)
	# 	print "pixel(", pxl_x, ",", pxl_y, ")"
	# 	print "3D location (", bd_loc[0,0], ",", bd_loc[1,0],",",bd_loc[2,0],")"
	# 	print "board orientation", bd_ori

	# 	vis.plot_camera_with_boards(cam_extrin, [B])
	# 	vis.plot_all_chessboards_in_camera(img_pts, true_cam.size, seperate_plot=False)

	def plot_boards_around_camera()
