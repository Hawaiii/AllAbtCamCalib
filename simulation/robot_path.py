import camera as cam
import board as bd
import exp_util as util

def robot_pose_from_board_pixel_align(camera, camera_loc, board, robot2board_pose, pxl, align_point):
	pt3d, ray_vec = camera.ray_from_pixel(pxl, camera_loc.extrinsics())
	board_loc = pt3d + depth * ray_vec # board align point location
	board_ori = -ray_vec # board orientation
	align2base = util.Pose(board_loc, board_ori)
	T_align2base = align2base.T_pose2world()

	# Calculate board pose from align point pose
	tl2align_trans = -board.pts[align_point]
	tl2align = util.Pose(tl2align_trans, np.eye(3))
	T_tl2align = tl2align.T_pose2world()

	# Calculate robot pose from board pose
	T_robot2align = robot2board_pose.T_pose2world()
	T_robot2base = T.align2base.dot(T_tl2align.dot(T_robot2align))

	return util.Pose(T_robot2base[0:3, -1], T_robot2base[0:3, 0:3])

def robot_calibrate_cam_intrinsics(camera, camera_loc, board, robot2board_pose, samples, border_pxl=0):
	# Input: approximate camera intrinsics and extrinsics(in robot base frame)
	# Output: list of robot poses in 3D to calibrate the camera
	#
	# Args:
	# camera: a Camera with written intrinsics and distortion
	# camera_loc: Pose, camera pose in robot base frame
	# board: Board
	# robot2board_pose: Pose, robot tip pose in board frame
	# samples: (n_samples_width, n_samples_height), number of points to sample
	# border_pxl: int

	# Calculate appropriate depth to place camera: min depth s.t. board fits camera
	# depth = board.min_depth_fit_cam(camera) #@TODO
	depth = 1

	robot_poses = []
	# Sample in clock-wise order
	for x in xrange(0, samples[0]):
		pxl_x = x * camera.width() / samples[0]
		pxl_y = border_pxl

		if x < samples[0]/2: # align top-left point
			r_pose = robot_pose_from_board_pixel_align(camera, camera_loc, board, \
														robot2board_pose, \
														(pxl_x, pxl_y), \
														(0,0))
			robot_poses.append(r_pose)
		else: # align top-right point
			r_pose = robot_pose_from_board_pixel_align(camera, camera_loc, board, \
														robot2board_pose, \
														(pxl_x, pxl_y), \
														(board.width()-1,0))
			robot_poses.append(r_pose)
	for y in xrange(0, samples[1]):
		pxl_x = camera.width()-1-boarder_pxl
		pxl_y = y * camera.height() / samples[1]
		if y < samples[1]/2: # align top-right point
			r_pose = robot_pose_from_board_pixel_align(camera, camera_loc, board, \
														robot2board_pose, \
														(pxl_x, pxl_y), \
														(board.width()-1,0))
			robot_poses.append(r_pose)
		else: # align bottom-right point
			r_pose = robot_pose_from_board_pixel_align(camera, camera_loc, board, \
														robot2board_pose, \
														(pxl_x, pxl_y), \
														(board.width()-1,board.height()-1))
			robot_poses.append(r_pose)
	for x in xrange(0, samples[0]):
		pxl_x = camera.width()-1 - x*camera.width()/samples[0]
		pxl_y = camera.height()-1-boarder_pxl
		if x > samples[0]/2: # align bottom-right point
			r_pose = robot_pose_from_board_pixel_align(camera, camera_loc, board, \
														robot2board_pose, \
														(pxl_x, pxl_y), \
														(board.width()-1,board.height()-1))
			robot_poses.append(r_pose)
		else: # align bottom-left point
			r_pose = robot_pose_from_board_pixel_align(camera, camera_loc, board, \
														robot2board_pose, \
														(pxl_x, pxl_y), \
														(0,board.height()-1))
			robot_poses.append(r_pose)
	for y in xrange(0, samples[1]):
		pxl_x = boarder_pxl
		pxl_y = camera.height()-1 - y*camera.height()/samples[1]
		if y > samples[1]/2: # align bottom-left point
			r_pose = robot_pose_from_board_pixel_align(camera, camera_loc, board, \
														robot2board_pose, \
														(pxl_x, pxl_y), \
														(0,board.height()-1))
			robot_poses.append(r_pose)
		else:# align top-left point
			r_pose = robot_pose_from_board_pixel_align(camera, camera_loc, board, \
														robot2board_pose, \
														(pxl_x, pxl_y), \
														(0,0))
			robot_poses.append(r_pose)
	return robot_poses




