"""
Runs experiment on calibration.
"""

import exp_util as util
import camera as cam
import vis

import numpy as np
import math
import matplotlib.pyplot as plt

"""
Functions for running experiment and analyzing experiment results.
"""

# def rotate_to_match_corners(board, true_cam, cam_loc, detection_noise):
# 	"""
# 	Rotate camera to capture images that matches the four corners of chessboard to
#     four corners of the camera
#     Return a list of points seen in different images captured.
# 	"""
# 	# @TODO
# 	return None, None

def target_at_layered_grids(nlayer, grid_size, aov, board, board_size):
	"""
	Simulates the situation of a static camera, while the calibration target
	appears on several planes of different depth to the camera.
	TODO: Assumes camera position at (0,0,0;0,0,1) for now, could be changed to
	other positions later.
	TODO: Currently start from min_depth and move back in min_depth*2^n, could
	be changed to have other more flexible depth spacing

	Args:
		nlayer: positive integer, number of depth layered_grids
		grid_size: tuple of two positive integer, (grid_height, grid_width)
		aov: tuple of two positive number, (aov_vertical, aov_horizontal),
		     representing camera angle of view angle in degrees
		board: calibration target to be used with, generated by
			   util.gen_calib_board
		board_size: actual dimensions of the calibration target, in mm,
		            (board_height_mm, board_width_mm). @TODO: Create board class
		            to wrap this inside.
	Returns:
		A list of dictionaries (boards), representing the 3D locations of the
		calibration target control points.
	"""
	targets = []

	# Compute minimum depth
	min_depth = max(board_size[0]/2. * math.tan(math.radians(aov[0]/2.)), \
		board_size[1]/2. * math.tan(math.radians(aov[1]/2.)) )

	# For each layer, evenly space the location of boards in a grid
	for layer in xrange(nlayer):
		cur_depth = min_depth * math.pow(2, layer)
		x_lim = cur_depth / math.tan(math.radians(aov[0]/2.)) #vertical
		y_lim = cur_depth / math.tan(math.radians(aov[1]/2.)) #horizontal
		x_step = (x_lim * 2. - board_size[0]) / grid_size[0]
		y_step = (y_lim * 2. - board_size[1]) / grid_size[1]
		for x_grid in xrange(grid_size[0]):
			for y_grid in xrange(grid_size[1]):
				cur_x = x_grid * x_step - x_lim
				cur_y = y_grid * y_step - y_lim
				cur_board = util.move_board(board, np.array([cur_x, cur_y, \
					cur_depth]), np.asarray([0.,0.,0.]))
				targets.append(cur_board)
	return targets

def target_on_ray_at_depth_with_orientation(camera, cam_extrin, \
											pixel, depth, orientation, \
											board, board_size):
	"""
	Returns a board in 3D whose first control point projects to pixel on the camera,
	and is distance depth to the camera image plane. 

	Args:
		camera: Camera, containing intrinsics of camera
		cam_extrin: current location of camera
		pixel: (x,y), pixel location on image
		depth: distance of first control point to the image plane
		orientation: orientation of the board
		board: board to be moved to given location
		board_size: 

	Returns:
	    A dictionary keyed by point id, whose values are 3D points
	"""
	ray = camera.ray_from_pixel(pixel, cam_extrin)
	print 'target_on_ray_at_depth_with_orientation not yet implemented!'
	# @TODO

	target = {}
	return target

"""
Running the experiment.
"""
exp_repeat_times = 2
noise3d_lvls = [0.]
noise2d_lvls = [0]
# noise3d_lvls = [0, 0.5, 1, 2]
# noise2d_lvls = [0, 0.5, 1, 2]
board_height = 5
board_width = 7
board_sqsize = 23
board_location = [0,0,0]
board_orientation = np.asarray([0.,0.,0.])


true_cam = cam.Camera.make_pinhole_camera()
print true_cam
cam_loc = cam.Extrinsics.init_with_numbers(0.,0.,0.,0.,0.,0.) #TODO: input numbers
true_cam.extrinsics[0] = cam_loc
for noise3d in noise3d_lvls:
	for noise2d in noise2d_lvls:

		print "Experiment with target noise:(mm)", noise3d, "detection noise:(pxl)", noise2d
		estimations = []
		
		for exp_iter in xrange(exp_repeat_times):
			board = util.gen_calib_board(board_height, board_width, board_sqsize, \
				board_location, board_orientation, noise3d)

			# Move the calibration target on different grid layers
			layered_grids = target_at_layered_grids(3, (5, 7), true_cam.aov, board,\
				(board_height*board_sqsize, board_width*board_sqsize))
			# vis.plot_calib_boards(layered_grids, (board_height, board_width))
			img_pts = true_cam.capture_images(layered_grids)
			# vis.plot_all_chessboards_in_camera(img_pts,true_cam.size)

			# Estimate camera parameters from captured images
			esti_cam = cam.Camera.calibrate_camera(img_pts, board, true_cam.size)
			# vis.compare_board_estimations(esti_cam.extrinsics, board, (board_height, board_width), \
			# 	layered_grids, save_name='compare_board.pdf')

			estimations.append(esti_cam)

		# Analyze error
		vis.write_esti_results(estimations, true_cam, \
			save_name_pre='report_3dn_'+str(noise3d)+'_2dn_'+str(noise2d))
print "experiment DONE"
