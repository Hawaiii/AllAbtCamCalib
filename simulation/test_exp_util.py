# For testing individual functions developed for the simulation
import unittest
import exp_util as util
import numpy as np
import cv2
import math
import vis

class TestExpUtilMethods(unittest.TestCase):
	def test_random_rotation(self):
		z_vec = np.asarray([0,0,1])
		vecs = []
		for i in range(1000):
			rot = util.random_rotation()
			rot_mat, _ = cv2.Rodrigues(rot)
			vecs.append(rot_mat.dot(z_vec))
		#vis.plot_directions(vecs)

	# def test_move_board(self):
	# 	bx30 = util.gen_calib_board(4,3,1,np.array([0,0,0]).reshape(3,1), np.array([0,-1./2, math.sqrt(3)/2]),0)
	# 	moved = util.move_board(bx30, np.array([0.,0.,0.]).reshape(3,1), np.array([0, math.pi/3, 0]))
	# 	vis.plot_calib_boards([bx30,moved], (4,3))

	def test_pose_init(self):
		p0 = util.Pose(np.array([3,4,5]).reshape(3,1), np.zeros((3,1)), 0)
		p1 = util.Pose(np.array([3,4,5]), np.zeros((3,1)), 0)
		p2 = util.Pose(np.array([3,4,5]), np.eye(3), 0)
		assert( np.allclose(p0.loc, p1.loc))
		assert( np.allclose(p0.loc, p2.loc))
		assert( np.allclose(p0.ori, p1.ori))
		assert( np.allclose(p0.ori, p2.ori))

	def test_pose_transform(self):
		p0 = util.Pose(np.zeros((3,1)), np.zeros((3,1)), 0)
		p1 = util.Pose(np.array([1,0,0]), np.zeros((3,1)), 0) #translation

		p1_0 = p0.transform_p2w(p1)
		assert( np.allclose(p1_0.loc, p1.loc) )
		assert( np.allclose(p1_0.ori, p1.ori) )
		assert( np.allclose(p1_0.time, p1.time) )
		p0_1 = p1.transform_p2w(p0)
		assert( np.allclose(p0_1.loc, p1.loc) )
		assert( np.allclose(p0_1.ori, p1.ori) )
		assert( np.allclose(p0_1.time, p1.time) )

		p2 = util.Pose(np.zeros((3,1)), np.array([math.pi/6, 0, 0])) #rotate 30 degrees in x axis
		p0_2 = p2.transform_p2w(p0)
		assert( np.allclose(p0_2.loc, p2.loc) )
		assert( np.allclose(p0_2.ori, p2.ori) )
		assert( np.allclose(p0_2.time, p1.time) )
		p1_2 = p2.transform_p2w(p1)
		assert( np.allclose(p1_2.loc, p1.loc) )
		assert( np.allclose(p1_2.ori, p2.ori) )
		assert( np.allclose(p1_2.time, p1.time) )
		p2_1 = p1.transform_p2w(p2)
		assert( np.allclose(p2_1.loc, p1.loc) )
		assert( np.allclose(p2_1.ori, p2.ori) )
		assert( np.allclose(p2_1.time, p1.time) )

		p3 = util.Pose(np.array([0,2,0]), np.zeros((3,1)), time=0.3) #translation and time offset
		p3_0 = p0.transform_p2w(p3)
		assert( np.allclose(p3_0.loc, p3.loc) )
		assert( np.allclose(p3_0.ori, p3.ori) )
		assert( np.allclose(p3_0.time, p3.time) )
		p3_1 = p1.transform_p2w(p3)
		assert( np.allclose(p3_1.loc, np.array([1,2,0]).reshape(3,1)) )
		assert( np.allclose(p3_1.ori, p3.ori) )
		assert( np.allclose(p3_1.time, p3.time) )
		p3_2 = p2.transform_p2w(p3)
		assert( np.allclose(p3_2.loc, np.array([0, math.sqrt(3), 1]).reshape(3,1)) )
		assert( np.allclose(p3_2.ori, p2.ori) )
		assert( np.allclose(p3_2.time, p3.time) )


