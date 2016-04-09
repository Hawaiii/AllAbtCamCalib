"""
Camera models, estimating camera parameters, etc.
Camera related functions here.
"""
import cv2
import numpy as np
import vis

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
		tmp,_ = cv2.Rodrigues(rot_vec)
		return cls(trans_vec, rot_vec, tmp)
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
	intrinsics = None #Intrinsics
	extrinsics = {} #dictionary of extrinsics keyed by img number
	size = None #(height, width) of image size, in pixels
	aov = None #(angle_of_view_vertical, angle_of_view_horizontal), in degree
	name = None #A string that could be used for identifying the camera

	def __init__(self, intrinsics, extrinsics, img_size, aov, name):
		self.intrinsics = intrinsics
		if extrinsics != None:
			self.extrinsics = extrinsics
		self.size = img_size
		self.aov = aov
		self.name = name

	def __repr__(self):
		cam_str = 'Camera ' + self.name + ':\n\tIntrinsics:\n' + \
		str(self.intrinsics) + '\n\tExtrinsics:\n'
		cam_str += '\tomitted because too long\n'
		# for ext in self.extrinsics:
		# 	cam_str += '\n\t\t'
		# 	cam_str += str(ext)
		# 	cam_str += ': '
		# 	cam_str += str(self.extrinsics[ext])
		cam_str += '\n\tAngle of view: '
		cam_str += str(self.aov)
		cam_str += '\n\tImage size: '
		cam_str += str(self.size)
		return cam_str

	def get_opencv_dist_coeffs(self):
		"""
		Returns OpenCV format distortion coefficients [k1, k2, p1, p2, k3]
		as a 1x5 numpy array.
		"""
		return np.asarray([[self.intrinsics.radial_dist[0],\
							self.intrinsics.radial_dist[1],\
							self.intrinsics.tang_dist[0],\
							self.intrinsics.tang_dist[1],\
							self.intrinsics.radial_dist[2]]])

	def capture_image(self, point):
		# @TODO
		pass

	def capture_images(self, points):
		# @TODO
		mapx,mapy = cv2.initUndistortRectifyMap(self.intrinsics.intri_mat, \
									np.concatenate( (self.intrinsics.radial_dist[0:2],self.intrinsics.tang_dist[:], np.asarray([ self.intrinsics.radial_dist[-1] ])) ,axis = 0), \
									np.eye(3), \
									self.intrinsics.intri_mat, \
									self.size,\
									cv2.CV_32FC1)

		img_pts = list()
		for chessboard in points:
			img_pts_per_chessboard = {};
			for point_id in chessboard:
				# self.
				# perpare extrinsics matrix
				ext = np.concatenate( (self.extrinsics[0].rot_mat, np.reshape(self.extrinsics[0].trans_vec, (-1, 1))), axis = 1)
				#points in camera frame
				pts = np.dot(ext, np.append(chessboard[point_id] ,1))
				#points in image frame (distortion still !!)
				pts = np.dot(self.intrinsics.intri_mat, pts)
				pts = np.divide(pts, pts[-1])
				#apply distortion
				if(pts[0] >=0 and pts[0] < self.size[1] and pts[1] >= 0 and pts[1] < self.size[0]  ):
					final_pts = np.ones((3,1))
					x_ = mapx[pts[0],pts[1]]
					y_ = mapy[pts[0],pts[1]]
					final_pts[0] = x_ #col
					final_pts[1] = y_ #row
					img_pts_per_chessboard[point_id] = final_pts
				pass
			img_pts.append( img_pts_per_chessboard )
			pass
		pass
		print "capture_images OK"
		return img_pts

	def ray_from_pixel(self, pixel, cam_extrin):
		"""
		Returns the light ray pixel is capturing on camera.

		Args:
			pixel: (x,y)
			cam_extrin: Extrinsics
		Returns:
			pt3d: 1x3 numpy array, the 3D location of camear center
			ray_vec: 1x3 numpy array, a unit vector pointing in the direction of 
			         the ray
			(pt3d + a * ray_vec, where a is a scalar, gives a point on the ray)
		"""
		# Map pixel to undistorted pixel location
		dist_loc = np.zeros((1,1,2), dtype=np.float32)
		dist_loc[0,0,0] = pixel[0]
		dist_loc[0,0,1] = pixel[1]
		nodist_loc = cv2.undistortPoints(dist_loc, self.intrinsics.intri_mat, \
										self.get_opencv_dist_coeffs())

		# Calculate ray from pixel from intrinsics

		print 'ray_from_pixel not fully implemented yet!'
		#@TODO		
		pass

	@staticmethod
	def calibrate_camera(img_pts, board, img_size):
		"""
		Given image coordinates of points and actual 3D points, return a list of
		intrinsics and extrinsics of camera estimated from the point coordinates.
		"""
		# Save all seen images to file
		#vis.plot_all_chessboards_in_camera(img_pts, img_size, save_name='debug_calibrate_camera.pdf')

		board_list = list()
		img_pts_list = list()
		view_id = list()
		for i in range(len(img_pts)):
			pts_id = img_pts[i].keys()
			if len(pts_id) < len(board):
				#print 'Cannot see the whole board in image', i
				continue
			view_id.append(i)
			board_list.append(  np.asarray( [ board[x] for x in pts_id ] , dtype=np.float32) )
			# board_list.append(  [ np.asarray(board[x].tolist()) for x in pts_id ]  )
			img_pts_list.append( np.reshape ( np.asarray( [img_pts[i][x][0:2].flatten().tolist() for x in pts_id], dtype=np.float32), (-1,1,2)) )
			# print str(board_list[-1].shape) + " == " + str(img_pts_list[-1].shape)

		# Inputs format:
		# board_list list of np(N, 3) float32
		# img_pts_list list of np(N, 1, 2) float32
		# (1260, 1080) (x, y)
		retval, cameraMatrix, distCoeffs, rvecs, tvecs  = cv2.calibrateCamera( board_list, img_pts_list, (img_size[1], img_size[0]), None, None)
		print 'Calibration RMS re-projection error', retval

		# package return vale
		intrinsics_ = Intrinsics(cameraMatrix, \
			np.concatenate( (distCoeffs[0][0:2], distCoeffs[0][4:5]), axis=0 ), \
			distCoeffs[0][2:4])
		extrinsics_ = dict()
		for i in range(len(rvecs)):
			rot_mat_, _ = cv2.Rodrigues(rvecs[i])
			extrinsics_[view_id[i]] = Extrinsics(tvecs[i][:,0], rvecs[i][:,0], rot_mat_)
		
		size = img_size
		aov = None
		name = "calibrated cam pov"
		print 'calibrate_camera OK'
		return Camera(intrinsics_, extrinsics_, size, aov, name)

	@staticmethod
	def make_pinhole_camera():
		"""
		Make a camera whose intrinsics and distortion coefficients are computed
		from a real experiment.
		//may be useful:
		cv2.remap(src, map1, map2, interpolation[, dst[, borderMode[, borderValue]]]) dst

		Returns:
			a Camera.
		"""
		intri_mat = np.array([[6.1578617661522037e+02, 0., 6.1058715036731826e+02], \
			[0., 6.1238350456764329e+02, 5.2959938173888008e+02],\
			[0., 0., 1.]])
		radial_dist = np.array([-1.8871087691827788e-02, 3.1035356528952687e-02, -1.2440956825625479e-02])
		tang_dist = np.array([1.1627819998974306e-03, -1.8172149748173956e-04])
		intri = Intrinsics(intri_mat, radial_dist, tang_dist)

		#@TODO: look up actual Ximea camera angle of view
		return Camera(intri, None, (1016,1264),(53.8,84.1),"pinhole")
