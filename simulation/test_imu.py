import unittest
import imu
import vis
import camera as cam

class TestIMUMethods(unittest.TestCase):
	def test_spiral_motion(self):
		extrins = imu.spiral_motion()
		vis.plot_poses(extrins)