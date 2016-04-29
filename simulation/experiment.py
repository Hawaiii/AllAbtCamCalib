"""
Runs experiment on calibration.
"""

import exp_util as util
import camera as cam
import board as bd
import vis

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import copy


"""
Experiment and related 
"""
def calib_with_angle():
	angle_change = [0, 0.01, 0.1, 1]

	# parameters
	exp_repeat_times = 100
	noise3d_lvls = [0]
	noise2d_lvls = [0.3]
	board_height = [5]
	board_width = [8]
	board_sqsize = [0.23]
	depth_min = 3 #m
	depth_max = 5#m
	n = 3

	for noise3d in noise3d_lvls:
		for noise2d in noise2d_lvls:
			true_cam = cam.Camera.nikon_camera()

			# true_cam.intrinsics.radial_dist = 1*true_cam.intrinsics.radial_dist
			# true_cam.intrinsics.tang_dist = 1*true_cam.intrinsics.tang_dist
			cam_loc = np.zeros((3,1))
			cam_ori = np.zeros((3,1))
			cam_extrin = util.Pose(cam_loc, cam_ori).extrinsics()
			bh = board_height[0]
		 	bw = board_width[0]
			bs = board_sqsize[0]
			for ac in angle_change:
					estimations = []
					start_angle = util.random_rotation()

					#check start_angle works
					board = bd.Board.gen_calib_board((bw, bh), bs, \
							np.zeros((3,1)), np.zeros((3,1)), noise3d)
					m_board = board.move_board_in_camera(true_cam, cam_extrin, \
						(true_cam.width()/2, true_cam.height()/2), 4, start_angle)
					while m_board is None:
						start_angle = util.random_rotation()
						m_board = board.move_board_in_camera(true_cam, cam_extrin, \
						(true_cam.width()/2, true_cam.height()/2), 4, start_angle)
					start_angle = cv2.Rodrigues(start_angle)[0]

					for iexp in xrange(exp_repeat_times):
						# Generate n boards
						
						perfect_board = bd.Board.gen_calib_board((bw, bh), bs, \
							np.zeros((3,1)), np.zeros((3,1)), 0)
						obs_list = []
						# for i in xrange(n):
						while len(obs_list) < n:
							# choose a random pixel
							pxl_x = np.random.random_integers(0, true_cam.width()-1)
							pxl_y = np.random.random_integers(0, true_cam.height()-1)

							# choose a random depth on ray from pixel
							depth = np.random.rand() * (depth_max - depth_min) + depth_min
							# pt3d, ray_vec = true_cam.ray_from_pixel((pxl_x, pxl_y), cam_extrin)
							# bd_loc = pt3d + depth*ray_vec
							
							# choose a random orientation
							bd_ori = start_angle.dot(cv2.Rodrigues(util.random_rot_w_scale(ac))[0])

							# board.move_board(bd_loc, bd_ori)
							m_board = board.move_board_in_camera(true_cam, cam_extrin, (pxl_x, pxl_y), depth, bd_ori)
							if m_board is not None:
								# print 'board at (', pxl_x, ',', pxl_y,',', depth, '), rotation', bd_ori.flatten()
								obs_list.append(m_board.get_points()) #3xN np array

						img_pts = true_cam.capture_images(cam_extrin, obs_list, noise2d)
						esti_cam = cam.Camera.calibrate_camera(img_pts, perfect_board, true_cam.size, noDistortion=False)
						# esti_cam = cam.Camera.calibrate_camera(img_pts, perfect_board, true_cam.size, noDistortion=True)
						# esti_cam.intrinsics.radial_dist = np.zeros((3,))
						# esti_cam.intrinsics.tang_dist = np.zeros((2,))
						# vis.plot_camera_with_points(cam_extrin, obs_list)
						# vis.plot_all_chessboards_in_camera(img_pts, true_cam.size, seperate_plot=True, save_name='results/capture_rot_'+str(ac)+'_'+str(iexp)+'.pdf')

						estimations.append(esti_cam)

					# Analyze error
					vis.write_esti_results(estimations, true_cam, \
						save_name_pre='results/rot_'+str(ac)+'_board_'+str(n)+'_2dn_'+str(noise2d))
	print "depth {0} board experiment DONE".format(n)
# calib_with_angle()

def calib_with_points():
	"""
	Generate n boards at random angle and depth, experiment for calibration result 
	under different 3d noise, 2d noise, number of control points;
	distortion, focal length, and center point.
	"""
	# parameters
	exp_repeat_times = 100
	noise3d_lvls = [0]
	noise2d_lvls = [0.3]
	board_height = [5,50,500]
	board_width = [8,80,800]
	board_tot_size = 1.64
	# board_sqsize = []
	depth_min = 1 #m
	depth_max = 5#m
	n = 3

	for noise3d in noise3d_lvls:
		for noise2d in noise2d_lvls:
			true_cam = cam.Camera.make_pinhole_camera()
			cam_loc = np.zeros((3,1))
			cam_ori = np.zeros((3,1))
			cam_extrin = util.Pose(cam_loc, cam_ori).extrinsics()
			for bi in xrange(len(board_height)):
				bh = board_height[bi]
				bw = board_width[bi]
				bs = board_tot_size/bw

				estimations = []
				for iexp in xrange(exp_repeat_times):
					# Generate n boards
					board = bd.Board.gen_calib_board((bw, bh), bs, \
						np.zeros((3,1)), np.zeros((3,1)), noise3d)
					perfect_board = bd.Board.gen_calib_board((bw, bh), bs, \
						np.zeros((3,1)), np.zeros((3,1)), 0)
					obs_list = []
					# for i in xrange(n):
					while len(obs_list) < n:
						# choose a random pixel
						pxl_x = np.random.random_integers(0, true_cam.width()-1)
						pxl_y = np.random.random_integers(0, true_cam.height()-1)

						# choose a random depth on ray from pixel
						depth = np.random.rand() * (depth_max - depth_min) + depth_min
						# pt3d, ray_vec = true_cam.ray_from_pixel((pxl_x, pxl_y), cam_extrin)
						# bd_loc = pt3d + depth*ray_vec
						
						# choose a random orientation
						bd_ori = util.random_rotation()

						# board.move_board(bd_loc, bd_ori)
						m_board = board.move_board_in_camera(true_cam, cam_extrin, (pxl_x, pxl_y), depth, bd_ori)
						if m_board is not None:
							obs_list.append(m_board.get_points()) #3xN np array

					img_pts = true_cam.capture_images(cam_extrin, obs_list, noise2d)
					esti_cam = cam.Camera.calibrate_camera(img_pts, perfect_board, true_cam.size)
					# vis.plot_camera_with_points(cam_extrin, obs_list)
					# vis.plot_all_chessboards_in_camera(img_pts, true_cam.size, seperate_plot=True, save_name='results/capture_bn_'+str(bh*bw)+'_2dn_'+str(noise2d)+'_'+str(iexp)+'.pdf')

					estimations.append(esti_cam)

				# Analyze error
				vis.write_esti_results(estimations, true_cam, \
					save_name_pre='results/npts_'+str(bh*bw)+'_2dn_'+str(noise2d))
	print "random {0} board experiment DONE"
# calib_with_points()

def calib_with_depth():
	depth_start = [2, 4, 8]
	depth_range = [0.05]
	depth_end = 10

	# parameters
	exp_repeat_times = 50
	noise3d_lvls = [0]
	noise2d_lvls = [0.3]
	board_height = [5]
	board_width = [8]
	board_sqsize = [0.23]
	depth_min = 6 #m
	depth_max = 9#m
	n = 30

	for noise3d in noise3d_lvls:
		for noise2d in noise2d_lvls:
			true_cam = cam.Camera.nikon_camera()

			# true_cam.intrinsics.radial_dist = 1*true_cam.intrinsics.radial_dist
			# true_cam.intrinsics.tang_dist = 1*true_cam.intrinsics.tang_dist
			cam_loc = np.zeros((3,1))
			cam_ori = np.zeros((3,1))
			cam_extrin = util.Pose(cam_loc, cam_ori).extrinsics()
			bh = board_height[0]
		 	bw = board_width[0]
			bs = board_sqsize[0]
			for ds in depth_start:
				for dr in depth_range:
					if ds + dr > depth_end:
						continue
					depth_min = ds
					depth_max = ds + dr

					estimations = []
					for iexp in xrange(exp_repeat_times):
						# Generate n boards
						board = bd.Board.gen_calib_board((bw, bh), bs, \
							np.zeros((3,1)), np.zeros((3,1)), noise3d)
						perfect_board = bd.Board.gen_calib_board((bw, bh), bs, \
							np.zeros((3,1)), np.zeros((3,1)), 0)
						obs_list = []
						# for i in xrange(n):
						while len(obs_list) < n:
							# choose a random pixel
							pxl_x = np.random.random_integers(0, true_cam.width()-1)
							pxl_y = np.random.random_integers(0, true_cam.height()-1)

							# choose a random depth on ray from pixel
							depth = np.random.rand() * (depth_max - depth_min) + depth_min
							# pt3d, ray_vec = true_cam.ray_from_pixel((pxl_x, pxl_y), cam_extrin)
							# bd_loc = pt3d + depth*ray_vec
							
							# choose a random orientation
							bd_ori = util.random_rotation()

							# board.move_board(bd_loc, bd_ori)
							m_board = board.move_board_in_camera(true_cam, cam_extrin, (pxl_x, pxl_y), depth, bd_ori)
							if m_board is not None:
								# print 'board at (', pxl_x, ',', pxl_y,',', depth, '), rotation', bd_ori.flatten()
								obs_list.append(m_board.get_points()) #3xN np array

						img_pts = true_cam.capture_images(cam_extrin, obs_list, noise2d)
						esti_cam = cam.Camera.calibrate_camera(img_pts, perfect_board, true_cam.size, noDistortion=False)
						# esti_cam = cam.Camera.calibrate_camera(img_pts, perfect_board, true_cam.size, noDistortion=True)
						# esti_cam.intrinsics.radial_dist = np.zeros((3,))
						# esti_cam.intrinsics.tang_dist = np.zeros((2,))
						# vis.plot_camera_with_points(cam_extrin, obs_list)
						# vis.plot_all_chessboards_in_camera(img_pts, true_cam.size, seperate_plot=True, save_name='results/capture_depth_'+str(ds)+'_'+str(dr)+'_'+str(iexp)+'.pdf')

						estimations.append(esti_cam)

					# Analyze error
					vis.write_esti_results(estimations, true_cam, \
						save_name_pre='results/depth_s'+str(ds)+'_range_'+str(dr)+'_board_'+str(n)+'_2dn_'+str(noise2d))
	print "depth {0} board experiment DONE".format(n)

def calib_with_covarage():
	# parameters
	exp_repeat_times = 50
	noise3d_lvls = [0]
	noise2d_lvls = [0.3]
	board_height = [5]
	board_width = [8]
	board_sqsize = [0.23]
	depth_min = 6 #m
	depth_max = 9#m
	n = 50
	coverage = [1.0, 0.5, 0.1]

	for noise3d in noise3d_lvls:
		for noise2d in noise2d_lvls:
			true_cam = cam.Camera.nikon_camera()

			# true_cam.intrinsics.radial_dist = 1*true_cam.intrinsics.radial_dist
			# true_cam.intrinsics.tang_dist = 1*true_cam.intrinsics.tang_dist

			cam_loc = np.zeros((3,1))
			cam_ori = np.zeros((3,1))
			cam_extrin = util.Pose(cam_loc, cam_ori).extrinsics()
			bh = board_height[0]
		 	bw = board_width[0]
			bs = board_sqsize[0]
			for cov in coverage:
				estimations = []
				for iexp in xrange(exp_repeat_times):
					# Generate n boards
					board = bd.Board.gen_calib_board((bw, bh), bs, \
						np.zeros((3,1)), np.zeros((3,1)), noise3d)
					perfect_board = bd.Board.gen_calib_board((bw, bh), bs, \
						np.zeros((3,1)), np.zeros((3,1)), 0)
					obs_list = []
					# for i in xrange(n):
					while len(obs_list) < n:
						# choose a random pixel
						mid_x = true_cam.width()/2
						mid_y = true_cam.height()/2
						pxl_x = np.random.random_integers(int(mid_x-cov*mid_x), int(mid_x+cov*mid_x))
						pxl_y = np.random.random_integers(int(mid_y-cov*mid_y), int(mid_y+cov*mid_y))

						# choose a random depth on ray from pixel
						depth = np.random.rand() * (depth_max - depth_min) + depth_min
						# pt3d, ray_vec = true_cam.ray_from_pixel((pxl_x, pxl_y), cam_extrin)
						# bd_loc = pt3d + depth*ray_vec
						
						# choose a random orientation
						bd_ori = util.random_rotation()

						# board.move_board(bd_loc, bd_ori)
						m_board = board.move_board_in_camera(true_cam, cam_extrin, (pxl_x, pxl_y), depth, bd_ori)
						if m_board is not None:
							print 'board at (', pxl_x, ',', pxl_y,',', depth, '), rotation', bd_ori.flatten()
							obs_list.append(m_board.get_points()) #3xN np array

					img_pts = true_cam.capture_images(cam_extrin, obs_list, noise2d)
					esti_cam = cam.Camera.calibrate_camera(img_pts, perfect_board, true_cam.size, noDistortion=False)
					# esti_cam = cam.Camera.calibrate_camera(img_pts, perfect_board, true_cam.size, noDistortion=True)
					# esti_cam.intrinsics.radial_dist = np.zeros((3,))
					# esti_cam.intrinsics.tang_dist = np.zeros((2,))
					# vis.plot_camera_with_points(cam_extrin, obs_list)
					# vis.plot_all_chessboards_in_camera(img_pts, true_cam.size, seperate_plot=True, save_name='results/capture_cov_'+str(cov)+'_'+str(iexp)+'.pdf')

					estimations.append(esti_cam)

				# Analyze error
				vis.write_esti_results(estimations, true_cam, \
					save_name_pre='results/coverage_'+str(cov)+'_board_'+str(n)+'_2dn_'+str(noise2d))
	print "random {0} board experiment DONE".format(n)
# calib_with_covarage()

def calib_with_distortion_n_boards(n):
	"""
	Generate n boards at random angle and depth, experiment for calibration result 
	under different 3d noise, 2d noise, number of control points;
	distortion, focal length, and center point.
	"""
	# parameters
	exp_repeat_times = 100
	noise3d_lvls = [0]
	# noise3d_lvls = [0, 0.005, 0.01, 0.04]
	noise2d_lvls = [0]
	board_height = [5]
	board_width = [8]
	board_sqsize = [0.23]
	depth_min = 0.5 #m
	depth_max = 5#m

	for noise3d in noise3d_lvls:
		for noise2d in noise2d_lvls:
			# @TODO: experiment with different camera parameters?
			true_cam = cam.Camera.make_pinhole_camera()

			true_cam.intrinsics.radial_dist = 1*true_cam.intrinsics.radial_dist
			true_cam.intrinsics.tang_dist = 1*true_cam.intrinsics.tang_dist

			cam_loc = np.zeros((3,1))
			cam_ori = np.zeros((3,1))
			cam_extrin = util.Pose(cam_loc, cam_ori).extrinsics()
			for bh in board_height:
				for bw in board_width:
					for bs in board_sqsize:
						estimations = []
						for iexp in xrange(exp_repeat_times):
							# Generate n boards
							board = bd.Board.gen_calib_board((bw, bh), bs, \
								np.zeros((3,1)), np.zeros((3,1)), noise3d)
							perfect_board = bd.Board.gen_calib_board((bw, bh), bs, \
								np.zeros((3,1)), np.zeros((3,1)), 0)
							obs_list = []
							# for i in xrange(n):
							while len(obs_list) < n:
								# choose a random pixel
								pxl_x = np.random.random_integers(0, true_cam.width()-1)
								pxl_y = np.random.random_integers(0, true_cam.height()-1)

								# choose a random depth on ray from pixel
								depth = np.random.rand() * (depth_max - depth_min) + depth_min
								# pt3d, ray_vec = true_cam.ray_from_pixel((pxl_x, pxl_y), cam_extrin)
								# bd_loc = pt3d + depth*ray_vec
								
								# choose a random orientation
								bd_ori = util.random_rotation()

								# board.move_board(bd_loc, bd_ori)
								m_board = board.move_board_in_camera(true_cam, cam_extrin, (pxl_x, pxl_y), depth, bd_ori)
								if m_board is not None:
									print 'board at (', pxl_x, ',', pxl_y,',', depth, '), rotation', bd_ori.flatten()
									obs_list.append(m_board.get_points()) #3xN np array

							img_pts = true_cam.capture_images(cam_extrin, obs_list, noise2d)
							esti_cam = cam.Camera.calibrate_camera(img_pts, perfect_board, true_cam.size, noDistortion=False)
							# esti_cam = cam.Camera.calibrate_camera(img_pts, perfect_board, true_cam.size, noDistortion=True)
							# esti_cam.intrinsics.radial_dist = np.zeros((3,))
							# esti_cam.intrinsics.tang_dist = np.zeros((2,))
							# vis.plot_camera_with_points(cam_extrin, obs_list)
							# vis.plot_all_chessboards_in_camera(img_pts, true_cam.size, seperate_plot=True, save_name='results/capture_'+str(n)+'_3dn_'+str(noise3d)+'_2dn_'+str(noise2d)+'_bn_'+str(bh*bw)+'_bs_'+str(bs)+'_'+str(iexp)+'.pdf')

							estimations.append(esti_cam)

						# Analyze error
						vis.write_esti_results(estimations, true_cam, \
							save_name_pre='results/report_'+str(n)+'_3dn_'+str(noise3d)+'_2dn_'+str(noise2d)+'_bn_'+str(bh*bw)+'_bs_'+str(bs))
	print "random {0} board experiment DONE".format(n)

# calib_with_distortion_n_boards(2)

def calib_with_random_n_boards(n):
	"""
	Generate n boards at random angle and depth, experiment for calibration result 
	under different 3d noise, 2d noise, number of control points;
	distortion, focal length, and center point.
	"""
	# parameters
	exp_repeat_times = 50
	noise3d_lvls = [0]
	# noise3d_lvls = [0, 0.005, 0.01, 0.04]
	noise2d_lvls = [0]
	board_height = [5]
	board_width = [8]
	board_sqsize = [0.23]
	depth_min = 0.5 #m
	depth_max = 5#m

	for noise3d in noise3d_lvls:
		for noise2d in noise2d_lvls:
			# @TODO: experiment with different camera parameters?
			true_cam = cam.Camera.make_pinhole_camera()
			cam_loc = np.zeros((3,1))
			cam_ori = np.zeros((3,1))
			cam_extrin = util.Pose(cam_loc, cam_ori).extrinsics()
			for bh in board_height:
				for bw in board_width:
					for bs in board_sqsize:
						estimations = []
						for iexp in xrange(exp_repeat_times):
							# Generate n boards
							board = bd.Board.gen_calib_board((bw, bh), bs, \
								np.zeros((3,1)), np.zeros((3,1)), noise3d)
							perfect_board = bd.Board.gen_calib_board((bw, bh), bs, \
								np.zeros((3,1)), np.zeros((3,1)), 0)
							obs_list = []
							# for i in xrange(n):
							while len(obs_list) < n:
								# choose a random pixel
								pxl_x = np.random.random_integers(0, true_cam.width()-1)
								pxl_y = np.random.random_integers(0, true_cam.height()-1)

								# choose a random depth on ray from pixel
								depth = np.random.rand() * (depth_max - depth_min) + depth_min
								# pt3d, ray_vec = true_cam.ray_from_pixel((pxl_x, pxl_y), cam_extrin)
								# bd_loc = pt3d + depth*ray_vec
								
								# choose a random orientation
								bd_ori = util.random_rotation()

								# board.move_board(bd_loc, bd_ori)
								m_board = board.move_board_in_camera(true_cam, cam_extrin, (pxl_x, pxl_y), depth, bd_ori)
								if m_board is not None:
									obs_list.append(m_board.get_points()) #3xN np array

							img_pts = true_cam.capture_images(cam_extrin, obs_list, noise2d)
							esti_cam = cam.Camera.calibrate_camera(img_pts, perfect_board, true_cam.size)
							# vis.plot_camera_with_points(cam_extrin, obs_list)
							vis.plot_all_chessboards_in_camera(img_pts, true_cam.size, seperate_plot=True, save_name='results/capture_'+str(n)+'_3dn_'+str(noise3d)+'_2dn_'+str(noise2d)+'_bn_'+str(bh*bw)+'_bs_'+str(bs)+'_'+str(iexp)+'.pdf')

							estimations.append(esti_cam)

						# Analyze error
						vis.write_esti_results(estimations, true_cam, \
							save_name_pre='results/report_'+str(n)+'_3dn_'+str(noise3d)+'_2dn_'+str(noise2d)+'_bn_'+str(bh*bw)+'_bs_'+str(bs))
	print "random {0} board experiment DONE".format(n)

# calib_with_random_n_boards(2)

# def rotate_to_match_corners(board, true_cam, cam_loc, detection_noise):
# 	"""
# 	Rotate camera to capture images that matches the four corners of chessboard to
#     four corners of the camera
#     Return a list of points seen in different images captured.
# 	"""
# 	# @TODO
# 	return None, None

# def target_at_layered_grids(nlayer, grid_size, aov, board, board_size):
# 	"""
# 	Simulates the situation of a static camera, while the calibration target
# 	appears on several planes of different depth to the camera.
# 	TODO: Assumes camera position at (0,0,0;0,0,1) for now, could be changed to
# 	other positions later.
# 	TODO: Currently start from min_depth and move back in min_depth*2^n, could
# 	be changed to have other more flexible depth spacing

# 	Args:
# 		nlayer: positive integer, number of depth layered_grids
# 		grid_size: tuple of two positive integer, (grid_width, grid_height)
# 		aov: tuple of two positive number, (aov_vertical, aov_horizontal),
# 		     representing camera angle of view angle in degrees
# 		board: calibration target to be used with, generated by
# 			   util.gen_calib_board
# 		board_size: actual dimensions of the calibration target, in mm,
# 		            (board_width_mm, board_height_mm). 
# 	Returns:
# 		A list of dictionaries (boards), representing the 3D locations of the
# 		calibration target control points.
# 	"""
# 	targets = []

# 	# Compute minimum depth
# 	min_depth = max(board_size[0]/2. * math.tan(math.radians(aov[1]/2.)), \
# 		board_size[1]/2. * math.tan(math.radians(aov[0]/2.)) )

# 	# For each layer, evenly space the location of boards in a grid
# 	for layer in xrange(nlayer):
# 		cur_depth = min_depth * math.pow(2, layer)
# 		x_lim = cur_depth / math.tan(math.radians(aov[1]/2.)) #horizontal
# 		y_lim = cur_depth / math.tan(math.radians(aov[0]/2.)) #vertical
# 		x_step = (x_lim * 2. - board_size[0]) / grid_size[0]
# 		y_step = (y_lim * 2. - board_size[1]) / grid_size[1]
# 		for y_grid in xrange(grid_size[1]):
# 			for x_grid in xrange(grid_size[0]):
# 				cur_x = x_grid * x_step - x_lim
# 				cur_y = y_grid * y_step - y_lim
# 				cur_board = util.move_board(board, np.array([cur_x, cur_y, \
# 					cur_depth]).reshape(3,1), np.array([0.,0.,0.]).reshape(3,1))
# 				targets.append(cur_board)
# 	return targets

# def target_on_ray_at_depth_with_orientation(camera, cam_extrin, \
# 											pixel, depth, orientation, \
# 											board, board_size):
# 	"""
# 	Returns a board in 3D whose first control point projects to pixel on the camera,
# 	and is distance depth to the camera image plane. 

# 	Args:
# 		camera: Camera, containing intrinsics of camera
# 		cam_extrin: current location of camera
# 		pixel: (x,y), pixel location on image
# 		depth: distance of first control point to the image plane
# 		orientation: orientation of the board
# 		board: board to be moved to given location
# 		board_size: 

# 	Returns:
# 	    A dictionary keyed by point id, whose values are 3D points
# 	"""
# 	ray = camera.ray_from_pixel(pixel, cam_extrin)
# 	print 'target_on_ray_at_depth_with_orientation not yet implemented!'
# 	# @TODO

# 	target = {}
# 	return target

# """
# Running the experiment.
# """
# exp_repeat_times = 50
# noise3d_lvls = [0]
# # noise2d_lvls = [0]
# # noise3d_lvls = [0, 0.5, 1, 2]
# noise2d_lvls = [0, 0.5, 1, 2]
# board_height = 5
# board_width = 7
# board_sqsize = 23
# board_location = np.array([0,0,0]).reshape(3,1)
# board_orientation = np.array([0.,0.,0.]).reshape(3,1)


# true_cam = cam.Camera.make_pinhole_camera()
# print true_cam
# cam_loc = cam.Extrinsics.init_with_numbers(0.,0.,0.,0.,0.,0.) #TODO: input numbers
# # true_cam.extrinsics[0] = cam_loc
# for noise3d in noise3d_lvls:
# 	for noise2d in noise2d_lvls:

# 		print "Experiment with target noise:(mm)", noise3d, "detection noise:(pxl)", noise2d
# 		estimations = []
		
# 		for exp_iter in xrange(exp_repeat_times):
# 			board = util.gen_calib_board(board_width, board_height, board_sqsize, \
# 				board_location, board_orientation, noise3d)
			
# 			# Move the calibration target on different grid layers
# 			layered_grids = target_at_layered_grids(3, (7, 5), true_cam.aov, board,\
# 				(board_width*board_sqsize, board_height*board_sqsize))
# 			# vis.plot_calib_boards(layered_grids, (board_width, board_height))
# 			img_pts = true_cam.capture_images(cam_loc, layered_grids, noise2d)
# 			# vis.plot_all_chessboards_in_camera(img_pts,true_cam.size)

# 			# Estimate camera parameters from captured images
# 			esti_cam = cam.Camera.calibrate_camera(img_pts, board, true_cam.size)
# 			# bd.compare_board_estimations(esti_cam.extrinsics, board, (board_width, board_height), \
# 			# 	layered_grids, save_name='compare_board.pdf')

# 			estimations.append(esti_cam)

# 		# Analyze error
# 		vis.write_esti_results(estimations, true_cam, \
# 			save_name_pre='results/report_3dn_'+str(noise3d)+'_2dn_'+str(noise2d))
# print "experiment DONE"
