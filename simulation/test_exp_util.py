# For testing individual functions developed for the simulation
import unittest
import exp_util as util
import numpy as np
import cv2
import math
import vis

class TestExpUtilMethods(unittest.TestCase):
	def test_random_rotation(self):
		z_vec = np.asarray([0,0,1])
		vecs = []
		for i in range(1000):
			rot = util.random_rotation()
			rot_mat, _ = cv2.Rodrigues(rot)
			vecs.append(rot_mat*z_vec)
		#vis.plot_directions(vecs)


	def test_gen_calib_board(self):
		#@TODO

		# Board at (0,0,0) on x-y plane
		# b0 = util.gen_calib_board(2,2,1,np.array([0,0,0]), np.array([0,0,1]),0)

		# Rotate 30 degrees around x axis
		# Points should be at (0,0,0), (1,0,0), (0,sqrt(3)/2,-1/2), (1,sqrt(3)/2,-1/2)
		bx30 = util.gen_calib_board(2,2,1,np.array([0,0,0]), np.array([0,-1./2, math.sqrt(3)/2]),0)

		# Rotate 30 degrees around y axis
		# Points should be at (0,0,0), (sqrt(3)/2,0,-1/2), (0,1,0), (sqrt(3)/2,1,-1/2)
		by30 = util.gen_calib_board(2,2,1,np.array([0,0,0]), np.array([1./2, 0, math.sqrt(3)/2]),0)

		# Rotate 30 degrees around z axis
		# Points should be at (0,0,0), (sqrt(3)/2,-1/2,0), (1/2,sqrt(3)/2,0), (sqrt(6)/2,sqrt(2)/2,0)
		bz30 = util.gen_calib_board(2,2,1,np.array([0,0,0]), np.array([0,1./2, math.sqrt(3)/2]),0)

	def test_move_board(self):
		bx30 = util.gen_calib_board(4,3,1,np.array([0,0,0]).reshape(3,1), np.array([0,-1./2, math.sqrt(3)/2]),0)
		moved = util.move_board(bx30, np.array([0.,0.,0.]).reshape(3,1), np.array([0, math.pi/3, 0]))
		vis.plot_calib_boards([bx30,moved], (4,3))