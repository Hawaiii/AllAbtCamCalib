import imu
import vis
import camera as cam
import exp_util as util

import unittest
import numpy as np
import matplotlib.pyplot as plt


class TestIMUMethods(unittest.TestCase):
	def test_spiral_motion(self):
		board_dim = (7,8)
		board = util.gen_calib_board(board_dim[0],board_dim[1],23, np.asarray([[0,0,600]]), np.zeros([1,3]), 0)

		extrins = imu.spiral_motion(board, board_dim)
		ax = vis.plot_poses(extrins, invert = True)
		ax = vis.plot_calib_boards( [ board ], (7,8), fax=ax)
		ax.set_aspect('equal')
		plt.show()

		camera = cam.Camera.make_pinhole_camera()
		camera.intrinsics.radial_dist = np.zeros((1,3))
		camera.intrinsics.tang_dist = np.zeros((1,2))
		
		all_pts = []
		for i in xrange(len(extrins)):
			img_pts = camera.capture_images(extrins[i], [board], 0)
			for ipts in img_pts:
				all_pts.append(ipts)
		vis.plot_all_chessboards_in_camera(all_pts, camera.size, 'results/test_imu_captures.pdf')

		