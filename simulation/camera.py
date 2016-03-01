"""
Camera models, estimating camera parameters, etc. 
Camera related functions here.
"""
import cv2
import numpy as np

class Intrinsics:
	intri_mat = None #np array 3x3
	radial_dist = None #np array 3x1
	tang_dist = None #np array 2x1

	def __init__(self, intri_mat, radial_dist, tang_dist):
		self.intri_mat = intri_mat
		self.radial_dist = radial_dist
		self.tang_dist = tang_dist

	#@TODO

class Extrinsics:
	trans_vec = None #np array 1x3
	rot_vec = None #np array 1x3
	rot_mat = None #np array 3x3

	def __init__(self, trans_vec, rot_vec, rot_mat):
		self.trans_vec = trans_vec
		self.rot_vec = rot_vec
		self.rot_mat = rot_mat

	@classmethod
	def init_with_rotation_matrix(cls, trans_vec, rot_mat):
		return cls(trans_vec, None, rot_mat)
		# @TODO: convert rot_vec
	
	@classmethod
	def init_with_rotation_vec(cls, trans_vec, rot_vec):
		return cls(trans_vec, rot_vec, None)
		# @TODO: convert to rot_mat
	
	@classmethod
	def init_with_numbers(cls, x, y, z, rx, ry, rz):
		# return cls(np.array([x,y,z]), np.array([rx,ry,rz]), None)
		return cls.init_with_rotation_vec(np.array([x,y,z]), np.array([rx,ry,rz]))

class Camera:
	intriniscs = None #Intrinsics
	extrinsics = [] #list of extrinsics - change into dictionary keyed by time?
	size = None #(height, width) of image size, in pixels

	def project_points(points):
		# @TODO
		pass

	@classmethod
	def calibrate_camera(cls, img_pts, board, img_size):
		"""
		Given image coordinates of points and actual 3D points, return a list of 
		intrinsics and extrinsics of camera estimated from the point coordinates.
		"""
		# @TODO
		pass # return a camera

def make_pinhole_camera():
	"""
	Make a camera with given intrinsics and distortion coefficients
	"""
	# @TODO
	pass

