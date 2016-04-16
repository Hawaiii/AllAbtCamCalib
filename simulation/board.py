import exp_util as util

import numpy as np
import cv2

from mpl_toolkits.mplot3d.axes3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

class Board:
	size = None #(board_width, board_height)
	pts = {} # a dictionry keyed by control point positions (col, row), whose values are 
			 # 3D points (3x1 numpy arrays)
	pose = None #Pose

	def __init__(self, size, pts, pose):
		self.size = size
		self.pts = pts
		self.pose = pose

	def __repr__(self):
		selfstr = 'Board of width: {0} height: {1}'.format(self.size[0], self.size[1])
		selfstr += ' with top-left point at ({0},{1},{2})'.format(\
			self.pts[(0,0)][0,0], self.pts[(0,0)][1,0], self.pts[(0,0)][2,0])
		return selfstr

	def width(self):
		return self.size[1]

	def height(self):
		return self.size[0]

	@classmethod
	def gen_calib_board(cls, board_dim, sqsize, \
					location, orientation, noise3d):
		"""
		Generate a calibration board placed at give location with given direction,
		with given number of points on a plane.
		Args:
			board_dim: (board_width, board_height), number of control points in 
						horizontal and vertical direction
			sqsize: positive number, size of each square, in m
			location: 3x1 numpy array, 3D location of the top-left corner of board
			orientation: 3x1 or 3x3 numpy array, 3D orientation of the board,
						 specified by Euler angles or rotation matrix
			noise3d: positive number, standard deviation of Gaussian noise 
					 added to board, in m

		Returns:
			board: a Board
		"""
		# Check inputs
		assert(location.size == 3)
		location = location.reshape(3,1)
		assert(orientation.size == 3 or orientation.size == 9)
		if orientation.size == 3:
			orientation = orientation.reshape(3,1)
		orientation = 1.0*orientation
		assert(len(board_dim)==2 and board_dim[0] > 0 and board_dim[1] > 0)


		board_pts = {}
		# Make board whose top-left point is at (0,0,0) and lies on x-y plane
		for x in xrange(board_dim[1]):
			for y in xrange(board_dim[0]):
				board_pts[(y, x)] = np.array([[x * sqsize, y * sqsize, 0]], np.float32).T

		# Rotate board to given orientation and move to given location
		if orientation.size == 3:
			rot_mat, _ = cv2.Rodrigues(orientation)
		else:
			rot_mat = orientation
		for pt in board_pts.keys():
			board_pts[pt] = np.dot(rot_mat, board_pts[pt]) + location

		# Add noise3d
		if noise3d > 0:
			noises = np.random.normal(0, noise3d, (board_height*board_width,3))
			for ipt in xrange(len(board_pts)):
				board[board_pts.keys()[ipt]] += noises[ipt, :]

		orig_pose = util.Pose(location, orientation)

		return cls(board_dim, board_pts, orig_pose)

	def move_board(self):
		#TODO
		print 'move_board not implemented!'

	def get_four_corners(self):
		#@TODO
		print 'get_four_corners not implemented!'

	def dict2array(self):
		"""
		Converts the dictionary representation of board into X,Y,Z array format

		Returns:
			X,Y,Z: each a 2D numpy array, specifying the location in the
				   corresponding dimension of each point
		"""
		X = np.empty((self.size[1], self.size[0]))
		Y = np.empty((self.size[1], self.size[0]))
		Z = np.empty((self.size[1], self.size[0]))
		for pt in self.pts.keys():
			x = pt[1]
			y = pt[0]
			X[x,y] = self.pts[pt][0,0]
			Y[x,y] = self.pts[pt][1,0]
			Z[x,y] = self.pts[pt][2,0]
		return X, Y, Z

	def plot(self, fax=None, clr='b'):
		"""
		Plots board in 3D

		Args:
			fax: Axes to draw on
			clr: color to draw with
		Returns:
			ax: Axes drawn on
		"""
		if fax:
			ax = fax
		else:
			fig = plt.figure()
			ax = fig.add_subplot(111, projection='3d')

		#clist = colors.cnames.keys()
		X, Y, Z = self.dict2array()
		ax.plot_wireframe(X, Y, Z, color=clr)
		
		if not fax:
			plt.show()
		return ax
	
