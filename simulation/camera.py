"""
Camera models, estimating camera parameters, etc.
Camera related functions here.
"""
import cv2
import numpy as np
import pdb

class Intrinsics:
	intri_mat = None #np array 3x3
	radial_dist = None #np array 1x3
	tang_dist = None #np array 1x2

	def __init__(self, intri_mat, radial_dist, tang_dist):
		self.intri_mat = intri_mat
		self.radial_dist = radial_dist
		self.tang_dist = tang_dist

	def __repr__(self):
		return 'intrinsic matrix: ' + str(self.intri_mat) + \
		'\nradial distortion: ' + str(self.radial_dist) + \
		'\ntangential distortion: ' + str(self.tang_dist)

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
		return cls.init_with_rotation_vec(np.array([x,y,z]), \
			np.array([rx,ry,rz]))

	def __repr__(self):
		return 'translation: ' + str(self.trans_vec) + ' rotation: ' + \
		str(self.rot_vec) + ', ' + str(self.rot_mat)

class Camera:
	intriniscs = None #Intrinsics
	extrinsics = [] #list of extrinsics - change into dictionary keyed by time?
	size = None #(height, width) of image size, in pixels
	name = None #A string that could be used for identifying the camera

	def __init__(self, intrinsics, extrinsics, img_size, name):
		self.intrinsics = intrinsics
		if extrinsics != None:
			self.extrinsics = extrinsics
		self.size = img_size
		self.name = name

	def project_points(self, points):
		# @TODO
		pass

	@staticmethod
	def calibrate_camera(img_pts, board, img_size):
		"""
		Given image coordinates of points and actual 3D points, return a list of
		intrinsics and extrinsics of camera estimated from the point coordinates.
		"""
		# @TODO
		print "calibrate_camera not yet implemented!"
		# print 'Eric start:'
		# ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(board, img_pts, img_size, None, None)
		# print 'OK'
		pass # return a camera

	def __repr__(self):
		cam_str = 'Camera ' + self.name + ':\n\tIntrinsics:\n' + \
		str(self.intrinsics) + '\n\tExtrinsics:\n'
		for ext in self.extrinsics:
			cam_str += '\n\t\t'
			cam_str += str(self.extrinsics)
		cam_str += '\nimage size: '
		cam_str += str(self.size)
		return cam_str


def make_pinhole_camera():
	"""
	Make a camera with given intrinsics and distortion coefficients
	"""
	intri_mat = np.array([[6.1578617661522037e+02, 0., 6.1058715036731826e+02], \
		[0., 6.1238350456764329e+02, 5.2959938173888008e+02],\
		[0., 0., 1.]])
	radial_dist = np.array([-1.8871087691827788e-02, 3.1035356528952687e-02, -1.2440956825625479e-02])
	tang_dist = np.array([1.1627819998974306e-03, -1.8172149748173956e-04])
	intri = Intrinsics(intri_mat, radial_dist, tang_dist)

	return Camera(intri, None, (1264, 1016),"pinhole")
