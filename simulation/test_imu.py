import unittest
import imu
import vis
import camera as cam

class TestIMUMethods(unittest.TestCase):
	def test_spiral_motion(self):
		extrins = imu.spiral_motion()
		locs = [ext.trans_vec for ext in extrins]
		vis.plot_locations(locs)