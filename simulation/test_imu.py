import imu
import vis
import camera as cam

import unittest
import matplotlib.pyplot as plt

class TestIMUMethods(unittest.TestCase):
	def test_spiral_motion(self):
		extrins, board = imu.spiral_motion()
		ax = vis.plot_poses(extrins, invert = True)
		ax = vis.plot_calib_boards( [ board ], (7,8), fax=ax)
		ax.set_aspect('equal')
		plt.show()

		camera = cam.Camera.make_pinhole_camera()
		for i in xrange(len(extrins)):
			img_pts = camera.capture_images(extrins[i], [board], 0)
			vis.plot_all_chessboards_in_camera(img_pts, camera.size, 'test_imu_captures'+str(i)+'.pdf')

		