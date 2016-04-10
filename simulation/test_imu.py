import unittest
import imu
import vis
import camera as cam

class TestIMUMethods(unittest.TestCase):
	def test_spiral_motion(self):
		extrins, board = imu.spiral_motion()
		vis.plot_poses(extrins)
		vis.plot_calib_boards( [ board ], (7,8))
