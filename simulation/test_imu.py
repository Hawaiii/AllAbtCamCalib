import imu
import vis
import camera as cam

import unittest
import matplotlib.pyplot as plt

class TestIMUMethods(unittest.TestCase):
	def test_spiral_motion(self):
		extrins, board = imu.spiral_motion()
		ax = vis.plot_poses(extrins)
		ax = vis.plot_calib_boards( [ board ], (7,8), fax=ax)
		ax.set_aspect('equal')
		plt.show()
