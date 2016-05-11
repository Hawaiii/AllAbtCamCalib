"""
Camera models, estimating camera parameters, etc.
Camera related functions here.
"""
import cv2
import exp_util as util
import vis

import numpy as np
import numpy.matlib as matlib
import scipy.io as sio

import pdb

class Intrinsics:
	intri_mat = None #np array 3x3
	radial_dist = None #np vector 1x3
	tang_dist = None #np vector 1x2

	def __init__(self, intri_mat, radial_dist, tang_dist):
		self.intri_mat = intri_mat
		self.radial_dist = radial_dist
		self.tang_dist = tang_dist

	def __repr__(self):
		return 'intrinsic matrix: ' + str(self.intri_mat) + \
		'\nradial distortion: ' + str(self.radial_dist) + \
		'\ntangential distortion: ' + str(self.tang_dist)

	def fx(self):
		return self.intri_mat[0,0]

	def fy(self):
		return self.intri_mat[1,1]

	def cx(self):
		return self.intri_mat[0,2]

	def cy(self):
		return self.intri_mat[1,2]

class Extrinsics:
	trans_vec = None #np array 3x1
	rot_vec = None #np vector 1x3
	rot_mat = None #np array 3x3
	time_stamp = None #in ns (10^-9s)
	linear_vel = None #np array 3x1
	linear_acc = None #np array 3x1
	ang_vel = None #np array 3x1
	ang_acc = None #np array 3x1

	def __init__(self, trans_vec, rot_vec, rot_mat, time_stamp=None, \
		lvel=None, lacc=None, avel=None, aacc=None):
		self.trans_vec = trans_vec.reshape(3,1)
		self.rot_vec = rot_vec
		self.rot_mat = rot_mat
		self.time_stamp = time_stamp
		if lvel is not None:
			self.linear_vel = lvel.reshape(3,1)
		if lacc is not None:
			self.linear_acc = lacc.reshape(3,1)
		if avel is not None:
			self.ang_vel = avel.reshape(3,1)
		if aacc is not None:
			self.ang_acc = aacc.reshape(3,1)

	@classmethod
	def init_with_rotation_matrix(cls, trans_vec, rot_mat, time_stamp=None, \
		lvel=None, lacc=None, avel=None, aacc=None):
		tmp,_ = cv2.Rodrigues(rot_mat)
		return cls(trans_vec, tmp.T, rot_mat, time_stamp, lvel, lacc, avel, aacc)

	@classmethod
	def init_with_rotation_vec(cls, trans_vec, rot_vec, time_stamp=None, \
		lvel=None, lacc=None, avel=None, aacc=None):
		tmp,_ = cv2.Rodrigues(rot_vec)
		return cls(trans_vec, rot_vec, tmp, time_stamp, lvel, lacc, avel, aacc)

	@classmethod
	def init_with_numbers(cls, x, y, z, rx, ry, rz, time_stamp=None, \
		lvel=None, lacc=None, avel=None, aacc=None):
		return cls.init_with_rotation_vec(np.array([x,y,z]), \
			np.array([rx,ry,rz]), time_stamp, lvel, lacc, avel, aacc)

	def __repr__(self):
		selfstr = ''
		if self.time_stamp:
			selfstr += 'time: ' + str(self.time_stamp) + ' '
		selfstr += 'translation: ' + str(self.trans_vec) + ' rotation: ' + \
		str(self.rot_vec) + ', ' + str(self.rot_mat)
		return selfstr

	def get_Rt_matrix(self):
		"""
		Returns 3x4 matrix [R|t]
		"""
		return np.concatenate((self.rot_mat, self.trans_vec), axis=1)

	def get_Rt_matrix_inv(self):
		"""
		Returns 4x3 matrix ([R'|-R't])'
		"""
		return np.concatenate((self.rot_mat, -self.trans_vec.T.dot(self.rot_mat)), axis=0)

	def get_homo_trans_matrix(self):
		"""
		Returns 4x4 matrix 
		[R|t]
		[0|1]
		"""
		return np.concatenate((self.get_Rt_matrix(), np.array([[0.,0.,0.,1.]])), axis=0)

	def get_homo_trans_matrix_inv(self):
		return np.linalg.inv(self.get_homo_trans_matrix())

	def get_inv_location(self):
		"""
		Returns -R't as a 3x1 numpy array
		"""
		return -self.rot_mat.T.dot(self.trans_vec)

	def pose(self):
		return util.Pose(self.get_inv_location(), self.rot_mat.T, self.time_stamp)


class Camera:
	intrinsics = None #Intrinsics
	extrinsics = {} #dictionary of extrinsics keyed by img number
	size = None #(width, height) of image size, in pixels
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
		str(self.intrinsics) 
		cam_str += '\n\tExtrinsics:\n'
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

	def width(self):
		return self.size[0]

	def height(self):
		return self.size[1]

	def scale_size(self, scale_factor=1):
		"""
		Returns (img_width*scale_factor, img_height*scale_factor).
		"""
		scaled = []
		for i in xrange(len(self.size)):
			scaled.append( int(round(self.size[i] * scale_factor)) )
		return tuple(scaled)

	def capture_images(self, extrin, points, noise2d=0.0):
		"""
		Args:
			extrin: Extrinsics
			points: list of dictionaries, each dictionary representing a board
			nosie2d: std. dev. of point detection
		Returns:
			a list of 2xN numpy array, each representing a captured board
		"""
		mapx,mapy = cv2.initUndistortRectifyMap(self.intrinsics.intri_mat, \
									#np.concatenate( (self.intrinsics.radial_dist[0:2],self.intrinsics.tang_dist[:], np.asarray([ self.intrinsics.radial_dist[-1] ])) ,axis = 0), \
									self.get_opencv_dist_coeffs(), \
									np.eye(3), \
									self.intrinsics.intri_mat, \
									self.size,\
									cv2.CV_32FC1)
		img_pts = []

		if isinstance(points[0], dict):
			print "capture images points argument bad input type", type(points), " of ", type(points[0])
			# for chessboard in points:
			# 	img_pts_per_chessboard = {}
			# 	for point_id in chessboard:
					
			# 		pts = self.project_point(extrin, chessboard[point_id])
			# 		#apply distortion
			# 		if(pts[0,0] >=0 and pts[0,0] < self.size[0] and pts[1,0] >= 0 and pts[1,0] < self.size[1] ):
			# 			# final_pts = np.zeros((2,1))
			# 			x_ = mapx[pts[0,0],pts[1,0]]
			# 			y_ = mapy[pts[0,0],pts[1,0]]
			# 			final_pts[0] = x_ #col
			# 			final_pts[1] = y_ #row
			# 			final_pts = np.array([x_, y_]).reshape(2,1)
			# 			# final_pts = np.array([pts[0,0], pts[1,0]]).reshape(2,1)					
			# 			img_pts_per_chessboard[point_id] = final_pts
			# 			# add noise
			# 			if noise2d > 0:
			# 				# noises = np.concatenate((np.random.normal(0, noise2d, (2,1)),\
			# 				# 						np.zeros((1,1))), axis=0)
			# 				noises = np.random.normal(0, noise2d, (2,1))
			# 				img_pts_per_chessboard[point_id] += noises
			# 	img_pts.append( img_pts_per_chessboard )
		elif isinstance(points[0], np.ndarray) and points[0].shape[0] == 3:
			for chessboard in points: #3xN numpy array
				#projectPoints expect Nx3 points
				img_pts_per_chessboard,_ = cv2.projectPoints(chessboard.T, \
						extrin.rot_vec, extrin.trans_vec, \
						self.intrinsics.intri_mat, self.get_opencv_dist_coeffs())
				if noise2d > 0:
					noises = np.random.normal(0, noise2d, img_pts_per_chessboard.shape)
					img_pts_per_chessboard += noises
				img_pts.append(img_pts_per_chessboard.reshape(-1,2).T)
		else:
			print "capture images points argument bad input type", type(points), " of ", type(points[0])
			import pdb; pdb.set_trace()

		return img_pts

	def project_point(self, extrin, point):
		"""
			Project 3D points onto camera.
			Args:
				extrin: Extrinsics
				point: 3xN numpy array, a 3D point
			Returns: Nx1x2 numpy array
		"""
		loc,_ = cv2.projectPoints(point.T, \
						extrin.rot_vec, extrin.trans_vec, \
						self.intrinsics.intri_mat, self.get_opencv_dist_coeffs())
		# loc = loc.T.astype(np.float32).reshape((-1, 1, 2))
		return loc
		# if not nodistortion:
		# 	print "project_point with distortion not implemented!"
		# else:
		# 	if point.shape[0] != 3:
		# 		print 'point', point, 'dimension is not 3xN!'
		# 		return None
		# 	if len(point.shape) <= 1:
		# 		point = point.reshape(3,1)
		# 	point = np.concatenate((point, np.ones((1,point.shape[1]))), axis=0)
		# 	loc = self.intrinsics.intri_mat.dot( extrin.get_Rt_matrix().dot(point) )
		# 	loc = loc / matlib.repmat(loc[-1,:], 3, 1)
		# 	if len(loc.shape) > 1 and loc.shape[1] > 0:
		# 		for i in xrange(loc.shape[1]):
		# 			if loc[-1,i] <= 0:
		# 				loc[:-1,i] = float('nan')
		# 				import pdb; pdb.set_trace()
		# 				print 'point', point,'projects to wrong side of camera'
		# 	return loc[0:2]

	def all_observable(self, extrin, pts_3d, check_dist=True):
		assert(pts_3d.shape[0] == 3)
		pts_2d = self.project_point(extrin, pts_3d).reshape(-1, 2).T
		assert(pts_2d.shape[0] == 2)
		if np.any(pts_2d[0,:] < 0) or np.any(pts_2d[0,:] >= self.width()):
			return False
		if np.any(pts_2d[1,:] < 0) or np.any(pts_2d[1,:] >= self.height()):
			return False

		# Make sure they are far away from each other enough
		if check_dist:
			npts = pts_2d.shape[1]
			thres = 5
			for i in xrange(npts):
				for j in xrange(i+1, npts):
					if np.any(np.abs((pts_2d[:,i] - pts_2d[:,j]))< thres):
						return False
		return True

	def calc_homography(self, motion, board, im_size, img, scale, save_name):
		"""
		Calculates homography that transforms the image to board and render target as seen by the camera. Ignores distortion.
		Args:
			motion: list of Poses
			board: a Board
			im_size: (im_width, im_height)
			scale: scale up the image to avoid aliasing
		Returns:
			Hs: a list of 3x3 homography matrices
		"""
		Hs = []

		# Find image edge points
		im_edges = np.zeros((4,2), dtype=np.float32)
		im_edges[1,0] = im_size[0]-1
		im_edges[2,1] = im_size[1]-1
		im_edges[3,0] = im_size[0]-1
		im_edges[3,1] = im_size[1]-1
 
		# For each pose, project four corner locations onto image
		for i, pose in enumerate(motion):
			corners = board.get_four_corners()
			edges = self.project_point(pose.extrinsics(), corners, nodistortion=True).T.astype(np.float32)
			edges = edges * scale
			
			# Calculate homography
			H, mask = cv2.findHomography(im_edges, edges)

			warped = cv2.warpPerspective(img, H, (self.size[0]*scale, self.size[1]*scale), borderValue=np.array([127,127,127]))

			small = cv2.resize(warped, (0,0), fx=1./scale, fy=1./scale)
			if save_name is not None:
				cv2.imwrite(save_name+str(pose.time)+'.png', small)
			# Hd = self.intrinsics.intri_mat.dot(extrin.get_Rt_matrix().dot(board_to_world_T))[:, [0,1,3]]
			Hs.append(H)

		return Hs		
	# def calc_homography(self, extrins, board, board_dim, board_to_world_T):
	# 	"""
	# 	Calculates homography that transforms the image to board. Ignores distortion.
	# 	Args:
	# 		extrins: list of Extrinsics
	# 		board: a dictionary keyed by control point ID whose values are 3D points
	# 		board_dim: (board_width, board_height)
	# 	Returns:
	# 		Hs: a list of 3x3 homography matrices
	# 	"""
	# 	Hs = []

	# 	# Find image edge points
	# 	im_edges = np.zeros((4,2), dtype=np.float32)
	# 	im_edges[1,1] = self.size[0]-1
	# 	im_edges[2,0] = self.size[1]-1
	# 	im_edges[3,0] = self.size[1]-1
	# 	im_edges[3,1] = self.size[0]-1
 
	# 	# For each pose, project four corner locations onto image
	# 	for extrin in extrins:
	# 		edges = np.zeros((4,2), dtype=np.float32)
	# 		edges[0,:] = self.project_point(extrin, board[0], nodistortion=True).reshape(1,2)
	# 		edges[1,:] = self.project_point(extrin, board[board_dim[0]-1], nodistortion=True).reshape(1,2)
	# 		edges[2,:] = self.project_point(extrin, board[(board_dim[1]-1)*board_dim[0]], \
	# 														nodistortion=True).reshape(1,2)
	# 		edges[3,:] = self.project_point(extrin, board[board_dim[1]*board_dim[0]-1], \
	# 														nodistortion=True).reshape(1,2)

	# 		# Calculate homography
	# 		H, mask = cv2.findHomography(im_edges, edges)

	# 		# Hd = self.intrinsics.intri_mat.dot(extrin.get_Rt_matrix().dot(board_to_world_T))[:, [0,1,3]]

	# 		# import pdb; pdb.set_trace()
	# 		Hs.append(H)

	# 	return Hs

	def ray_from_pixel(self, pixel, cam_extrin):
		"""
		Returns the light ray pixel is capturing on camera.

		Args:
			pixel: (x,y), first col then row
			cam_extrin: Extrinsics
		Returns:
			pt3d: 3x1 numpy array, the 3D location of camear center
			ray_vec: 3x1 numpy array, a unit vector pointing in the direction of 
					 the ray
			(pt3d + a * ray_vec, where a is a scalar, gives a point on the ray)
		"""
		# Map pixel to undistorted pixel location
		dist_loc = np.zeros((1,1,2), dtype=np.float32)
		dist_loc[0,0,0] = pixel[0]
		dist_loc[0,0,1] = pixel[1]
		nodist_loc = cv2.undistortPoints(dist_loc, self.intrinsics.intri_mat, \
										self.get_opencv_dist_coeffs()) #1x1x2 np array
		nodist_loc[0,0,0] = nodist_loc[0,0,0] * self.intrinsics.fx() \
							 + self.intrinsics.cx()
		nodist_loc[0,0,1] = nodist_loc[0,0,1] * self.intrinsics.fy() \
							 + self.intrinsics.cy()

		# Camera center is at -Rt from extrinsics
		pt3d = cam_extrin.get_inv_location()

		# Calculate ray from pixel from intrinsics
		a = (pixel[0] - self.intrinsics.cx()) / self.intrinsics.fx()
		b = (pixel[1] - self.intrinsics.cy()) / self.intrinsics.fy()
		ray_pt = cam_extrin.rot_mat.T.dot(np.array([a,b,1]).reshape(3,1)-cam_extrin.trans_vec)
		ray_vec = ray_pt - pt3d
		return pt3d, ray_vec

	def write_offset_map_from_dist_coeffs(self):
		"""
		Args:
			camera: a Camera with intrinisics and distortion written
		Writes x and y offset mat to current directory
		"""
		mapx,mapy = cv2.initUndistortRectifyMap(self.intrinsics.intri_mat, \
										self.get_opencv_dist_coeffs(), \
										np.eye(3), \
										self.intrinsics.intri_mat, \
										self.size,\
										cv2.CV_32FC1)
		mapx = mapx - matlib.repmat(np.linspace(0, self.width(), num=self.width(), endpoint=False).reshape(1,self.width()), self.height(), 1)
		mapy = mapy - matlib.repmat(np.linspace(0, self.height(), num=self.height(), endpoint=False).reshape(self.height(),1), 1, self.width())

		sio.savemat('distort_map.mat', {'mapx':mapx, 'mapy':mapy})

	@staticmethod
	def calibrate_camera(img_pts, board, img_size, noDistortion=False):
		"""
		Given image coordinates of points and actual 3D points, return a list of
		intrinsics and extrinsics of camera estimated from the point coordinates.
		Args:
			img_pts: list of 2xN np array 
			board: a Board
			img_size: (img_width, img_height)
		"""
		# Save all seen images to file
		#vis.plot_all_chessboards_in_camera(img_pts, img_size, save_name='debug_calibrate_camera.pdf')

		board_list = []
		view_id = []
		b_pts = board.get_orig_points().astype(np.float32)
		for i in range(len(img_pts)):
			#pts_id = img_pts[i].keys()
			if img_pts[i].shape < board.num_points():
				print 'Cannot see the whole board in image', i
				continue
			view_id.append(i)
			board_list.append(b_pts.T.copy())
			img_pts[i] = img_pts[i].T.astype(np.float32).reshape((-1, 1, 2))
			# print str(board_list[-1].shape) + " == " + str(img_pts[-1].shape)

		# Inputs format:
		# board_list list of np(N, 3) float32
		# img_pts_list list of np(N, 1, 2) float32
		# (1260, 1080) (x, y)
		if noDistortion:
			retval, cameraMatrix, distCoeffs, rvecs, tvecs  = cv2.calibrateCamera( board_list, img_pts, (img_size[0], img_size[1]), None, np.zeros((8,1)), None, None, 8|32|64|128)
		else:
			retval, cameraMatrix, distCoeffs, rvecs, tvecs  = cv2.calibrateCamera( board_list, img_pts, (img_size[0], img_size[1]), None, None)
		print 'Calibration RMS re-projection error', retval
		
		# put img_pts[i] back to 2xN format
		for i in range(len(img_pts)):
			img_pts[i] = img_pts[i].reshape(-1, 2).T

		# package return vale
		intrinsics_ = Intrinsics(cameraMatrix, \
			np.concatenate( (distCoeffs[0][0:2], distCoeffs[0][4:5]), axis=0 ), \
			distCoeffs[0][2:4])
		extrinsics_ = dict()
		for i in range(len(rvecs)):
			extrinsics_[view_id[i]] = Extrinsics.init_with_rotation_vec(tvecs[i][:,0], rvecs[i][:,0])
		
		size = img_size
		aov = None
		name = "calibrated cam"
		return Camera(intrinsics_, extrinsics_, size, aov, name)
	
	@staticmethod
	def calibrate_camera_(img_pts, board, img_size):
		"""
		Given image coordinates of points and actual 3D points, return a list of
		intrinsics and extrinsics of camera estimated from the point coordinates.
		Args:
			img_pts: list of 2xN np array 
			board: a Board
			img_size: (img_width, img_height)
		Returns:
			camera:
			rvecs:
			tvecs:
		"""
		# Save all seen images to file
		#vis.plot_all_chessboards_in_camera(img_pts, img_size, save_name='debug_calibrate_camera.pdf')

		board_list = []
		view_id = []
		b_pts = board.get_orig_points().astype(np.float32)
		for i in range(len(img_pts)):
			#pts_id = img_pts[i].keys()
			if img_pts[i].shape < board.num_points():
				print 'Cannot see the whole board in image', i
				continue
			view_id.append(i)
			board_list.append(b_pts.T.copy())
			img_pts[i] = img_pts[i].T.astype(np.float32).reshape((-1, 1, 2))
			# print str(board_list[-1].shape) + " == " + str(img_pts[-1].shape)

		# Inputs format:
		# board_list list of np(N, 3) float32
		# img_pts_list list of np(N, 1, 2) float32
		# (1260, 1080) (x, y)
		retval, cameraMatrix, distCoeffs, rvecs, tvecs  = cv2.calibrateCamera( board_list, img_pts, (img_size[0], img_size[1]), None, np.zeros((8,1)), None, None, 8|32|64|128)
		print 'Calibration RMS re-projection error', retval
		
		# put img_pts[i] back to 2xN format
		for i in range(len(img_pts)):
			img_pts[i] = img_pts[i].reshape(-1, 2).T

		# package return vale
		intrinsics_ = Intrinsics(cameraMatrix, \
			np.concatenate( (distCoeffs[0][0:2], distCoeffs[0][4:5]), axis=0 ), \
			distCoeffs[0][2:4])
		extrinsics_ = dict()
		for i in range(len(rvecs)):
			extrinsics_[view_id[i]] = Extrinsics.init_with_rotation_vec(tvecs[i][:,0], rvecs[i][:,0])
		
		size = img_size
		aov = None
		name = "calibrated cam"
		return Camera(intrinsics_, extrinsics_, size, aov, name), rvecs, tvecs

	@staticmethod
	def make_pinhole_camera():
		"""
		Make a camera whose intrinsics and distortion coefficients are computed
		from a real experiment.
		Returns:
			a Camera.
		
		//may be useful:
		cv2.remap(src, map1, map2, interpolation[, dst[, borderMode[, borderValue]]]) dst
		"""
		# intri_mat = np.array([[6.1578617661522037e+02, 0., 6.1058715036731826e+02], \
		# 	[0., 6.1238350456764329e+02, 5.2959938173888008e+02],\
		# 	[0., 0., 1.]])
		# radial_dist = np.array([-1.8871087691827788e-02, 3.1035356528952687e-02, -1.2440956825625479e-02])
		# tang_dist = np.array([1.1627819998974306e-03, -1.8172149748173956e-04])
		#intri = Intrinsics(intri_mat, radial_dist, tang_dist)
		#return Camera(intri, None, (1016,1264),(53.8,84.1),"pinhole")

		# Xi camera calibrated on the week of April 11th pinhole-radtan model with Kalibr
		intri_mat = np.array([[619.9510108, 0., 630.46715704], \
			[0., 620.83866653, 530.26694171],\
			[0., 0., 1.]])
		radial_dist = np.array([-0.00187587, 0.00898923, 0.0])
		tang_dist = np.array([0.0018697, 0.00093728])
		intri = Intrinsics(intri_mat, radial_dist, tang_dist)
		cam_size = (1264, 1016)

		#@TODO: look up actual Ximea camera angle of view
		return Camera(intri, None, cam_size,(53.8, 84.1),"pinhole")

	@staticmethod
	def nikon_camera():
		"""
		Make a camera whose intrinsics and distortion coefficients are computed
		from a real experiment.
		Returns:
			a Camera.
		
		//may be useful:
		cv2.remap(src, map1, map2, interpolation[, dst[, borderMode[, borderValue]]]) dst
		"""
		# Nikon camera calibrated on Apr 27th
		# intri_mat = np.array([[663.128, 0., 401.775], \
		# 	[0., 882.618, 308.066],\
		# 	[0., 0., 1.]])
		# #TODO:zeros
		# #radial_dist = np.array([-0.00187587, 0.00898923, 0.0])
		# #tang_dist = np.array([0.0018697, 0.00093728])
		# radial_dist = np.zeros()
		# intri = Intrinsics(intri_mat, radial_dist, tang_dist)
		# cam_size = (800, 600)

		intri_mat = np.array([[664.935742, 0., 407.062943], \
			[0., 886.030013, 303.880099],\
			[0., 0., 1.]])
		#TODO:zeros
		radial_dist = np.array([-0.07287841, 0.37697890, -0.55815004])
		tang_dist = np.array([ -5.51172040e-05, 0.00222906])
		intri = Intrinsics(intri_mat, radial_dist, tang_dist)
		cam_size = (800, 600)

		return Camera(intri, None, cam_size, None,"pinhole")
		
