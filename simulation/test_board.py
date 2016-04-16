import board

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

	def test_gen_calib_board_with_plot(self):
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')

		# Board at (0,0,0) on x-y plane
		b0 = board.Board.gen_calib_board((3,2),1,np.array([0,0,0]), np.array([0,0,0]),0)
		b0.plot(ax, clr='r')

		# Rotate 30 degrees around x axis
		# Points should be at (0,0,0), (1,0,0), (0,sqrt(3)/2,-1/2), (1,sqrt(3)/2,-1/2)
		bx30 = board.Board.gen_calib_board((3,2),1,np.array([0,0,0]), np.array([math.pi/6,0,0]),0)
		bx30.plot(ax, clr='y')

		# Rotate 30 degrees around y axis
		# Points should be at (0,0,0), (sqrt(3)/2,0,-1/2), (0,1,0), (sqrt(3)/2,1,-1/2)
		by30 = board.Board.gen_calib_board((3,2),1,np.array([0,0,0]), np.array([0, math.pi/6,0]),0)
		by30.plot(ax, clr='g')

		# Rotate 30 degrees around z axis
		# Points should be at (0,0,0), (sqrt(3)/2,-1/2,0), (1/2,sqrt(3)/2,0), (sqrt(6)/2,sqrt(2)/2,0)
		bz30 = board.Board.gen_calib_board((3,2),1,np.array([0,0,0]), np.array([0,0,math.pi/6]),0)
		bz30.plot(ax, clr='b')

		plt.show()