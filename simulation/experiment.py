"""
Runs experiment on calibration.
"""

import exp_util as util
import camera as cam

"""
Functions for running experiment and analyzing experiment results.
"""

def rotate_to_match_corners(board, true_cam, cam_loc, detection_noise):
	"""
	Rotate camera to capture images that matches the four corners of chessboard to 
    four corners of the camera
    Return a list of points seen in different images captured.
	"""
	# @TODO
	return None, None

"""
Running the experiment.
"""
noise3d_lvls = [0, 0.5, 1, 2]
noise2d_lvls = [0, 0.5, 1, 2]
board_height = 5
board_width = 7
board_sqsize = 23
board_location = [0,0,0]
board_orientation = [0,0,0]


true_cam = cam.make_pinhole_camera()
cam_loc = cam.Extrinsics.init_with_numbers(0,0,0,0,0,1) #TODO: input numbers
for noise3d in noise3d_lvls:
	for noise2d in noise2d_lvls:
		
		print "target noise:(mm)", noise3d, "detection noise:(pxl)", noise2d
		board = util.gen_calib_board(board_height, board_width, board_sqsize, \
			board_location, board_orientation, noise3d)
		
		# Rotate camera around and capture images
		img_pts, true_extrin = rotate_to_match_corners(board, true_cam, \
														cam_loc, noise2d)

		# Estimate camera parameters from captured images
		esti_cam = cam.Camera.calibrate_camera(img_pts, board)

		# Analyze error
		diff = util.compute_estimation_diff(esti_cam, \
										true_cam, true_extrin)
print "experiment DONE"





