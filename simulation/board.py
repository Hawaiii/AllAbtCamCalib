
import exp_util as util

import numpy as np
import numpy.matlib as matlib
import cv2
import copy

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
		# selfstr += ' with top-left point at ({0},{1},{2})'.format(\
			# self.pts[(0,0)][0,0], self.pts[(0,0)][1,0], self.pts[(0,0)][2,0])
		selfstr += ' with {0}'.format(self.pose)
		return selfstr

	def width(self):
		return self.size[0]

	def height(self):
		return self.size[1]

	def num_points(self):
		return self.size[0]*self.size[1]

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

		# Add noise3d
		if noise3d > 0:
			noises = np.random.normal(0, noise3d, (board_dim[1]*board_dim[0],3))
			for ipt in xrange(len(board_pts)):
				board_pts[board_pts.keys()[ipt]] += noises[ipt, :].reshape(3,1)

		# # Rotate board to given orientation and move to given location
		# if orientation.size == 3:
		# 	rot_mat, _ = cv2.Rodrigues(orientation)
		# else:
		# 	rot_mat = orientation
		# for pt in board_pts.keys():
		# 	board_pts[pt] = np.dot(rot_mat, board_pts[pt]) + location
		orig_pose = util.Pose(location, orientation)

		return cls(board_dim, board_pts, orig_pose)

	def move_board(self, location, orientation):
		"""
		Moves the current board to a new given pose

		Args:
			location: 3x1 numpy array, desired 3D location of the top-left corner
					  of board
			orientation: 3x1 or 3x3 numpy array, desired 3D orientation of the board,
						 (rx,ry,rz)

		Returns:
			A dictionary keyed by point id, whose values are 3D points
		"""
		# return Board(self.size, self.pts, util.Pose(location, orientation))
		self.pose = util.Pose(location, orientation)
		return self

	def get_orig_points(self):
		npts = len(self.pts)
		pts_arr = np.empty((3, npts))
		for i in xrange(self.size[1]):
			for j in xrange(self.size[0]):
				pts_arr[:,i*self.width()+j] = self.pts[(j,i)].reshape(3)
				# print i,j,i*self.width()+j,pts_arr[:,i*self.width()+j]
		# pts_arr = np.array([self.pts[k] for k in self.pts]).reshape(3, npts)
		return pts_arr

	def get_points(self):
		"""
		Returns the actual location of the points (accounting for the pose) as a 
		3xN numpy array.
		"""
		npts = len(self.pts)
		pts_arr = self.get_orig_points()
		pts_arr = self.pose.ori.dot(pts_arr) + matlib.repmat(self.pose.loc, 1, npts)
		return pts_arr

	def get_points_homo(self):
		npts = np.prod(self.size)
		return np.concatenate((self.get_points(), np.ones((1, npts))), axis=0)

	def get_four_corners(self):
		"""
		Returns the actual location of the four corners points (accounting for 
		the pose) as 3x4 numpy
		"""
		pts_arr = np.array([self.pts[(0,0)], self.pts[(self.size[0]-1, 0)], \
			self.pts[(0, self.size[1]-1)], self.pts[(self.size[0]-1, self.size[1]-1)]])
		pts_arr = self.pose.ori.dot(pts_arr.T.reshape(3,4)) + matlib.repmat(self.pose.loc, 1, 4)

		return pts_arr

	def dict2array(self):
		"""
		Converts the dictionary representation of board into X,Y,Z array format

		Returns:
			X,Y,Z: each a 2D numpy array, specifying the location in the
				   corresponding dimension of each point
		"""
		pts_arr = self.get_points()
		X = pts_arr[0,:].reshape(self.size[1], self.size[0])
		Y = pts_arr[1,:].reshape(self.size[1], self.size[0])
		Z = pts_arr[2,:].reshape(self.size[1], self.size[0])
		# X = np.empty((self.size[1], self.size[0]))
		# Y = np.empty((self.size[1], self.size[0]))
		# Z = np.empty((self.size[1], self.size[0]))
		# cnt = 0
		# for pt in self.pts:
		# 	x = pt[1]
		# 	y = pt[0]
		# 	X[x,y] = pts_arr[0,cnt]
		# 	Y[x,y] = pts_arr[1,cnt]
		# 	Z[x,y] = pts_arr[2,cnt]
		# 	cnt += 1
		# print X
		return X, Y, Z

	def move_board_in_camera(self, camera, cam_extrin, pixel, depth, ori):
		"""
		Returns a board whose top-left point is observed at pixel in camera, 
		is at depth from camera, and is inside the camera's view (returns None
		if not all four corners are observed in the camera)
		Args:
			camera: a camera
			cam_extrin: Extrinsics
			pixel: (pixel_x, pixel_y), column followed by row
			depth: distance from the camera, in meters
			ori: 3x3 rotation matrix or 3x1 rotation vector
		Returns:
			a Board, or None if the board is not fully observable in camera
		"""
		# Find location
		pt3d, ray_vec = camera.ray_from_pixel(pixel, cam_extrin)
		bd_loc = pt3d + depth*ray_vec

		# Copy board
		new_board = copy.deepcopy(self)
		new_board.move_board(bd_loc, ori)
		
		# Check observable
		bd_edges = new_board.get_four_corners()
		if not camera.all_observable(cam_extrin, bd_edges) or not camera.all_observable(cam_extrin, new_board.get_points(), check_dist=False):
			return None
		else:
			return new_board

	def min_depth_fit_cam(self, camera):
		# @TODO
		pass

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

		X, Y, Z = self.dict2array()
		ax.plot_wireframe(X, Y, Z, color=clr)
		# pts_arr = self.get_points()
		# ax.plot_wireframe(pts_arr[0,:], pts_arr[1,:], pts_arr[2,:], color=clr)
		self.pose.plot(ax, clr=clr)
		
		if not fax:
			plt.show()
		return ax
	
	"""
	To be refactored
	"""
	@staticmethod
	def compare_board_estimations(esti_extrinsics, board, board_dim, \
								actual_boards, save_name=None):
		"""
		Plots true and estimated boards on the same figure
		Args:
			esti_extrinsics: dictionary, keyed by image number, values are Extrinsics
			board:
			board_dim: (board_width, board_height)
			actual_boards: list of dictionaries
			save_name: filename, string
		"""
		if save_name:
			pp = PdfPages(save_name)
		plt.clf()

		for i in xrange(len(actual_boards)):
			fig = plt.figure()
			ax = fig.add_subplot(111, projection='3d')

			act_board = actual_boards[i]
			aX, aY, aZ = util.board_dict2array(act_board, board_dim)
			ax.plot_wireframe(aX, aY, aZ, color='b')

			if i in esti_extrinsics:
				esti_loc = esti_extrinsics[i].trans_vec
				esti_board = util.move_board(board, esti_loc)
				eX, eY, eZ = util.board_dict2array(esti_board, board_dim)
				ax.plot_wireframe(eX, eY, eZ, color='r')

			if pp:
				pp.savefig()
			else:
				plt.show()
		if pp:
			pp.close()

	# def plot_calib_boards(boards, board_dim, fax=None):
# 	"""
# 	Plots a board in 3D

# 	Args:
# 		boards: a list of dictionaries, where each dictionary is a board
# 		board_dim: (board_width, board_height)
# 	"""
# 	if fax:
# 		ax = fax
# 	else:
# 		fig = plt.figure()
# 		ax = fig.add_subplot(111, projection='3d')

# 	clist = colors.cnames.keys()
# 	for i in xrange(len(boards)):
# 		board = boards[i]
# 		X, Y, Z = util.board_dict2array(board, board_dim)

# 		ax.plot_wireframe(X, Y, Z, color=clist[i])
	
# 	if not fax:
# 		plt.show()
# 	return ax

