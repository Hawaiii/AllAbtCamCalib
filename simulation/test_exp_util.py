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
			vecs.append(rot_mat*z_vec)
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

		p1_0 = p0.transform(p1)
		assert( np.allclose(p1_0.loc, p1.loc) )
		assert( np.allclose(p1_0.ori, p1.ori) )
		assert( np.allclose(p1_0.time, p1.time) )
		p0_1 = p1.transform(p0)
		assert( np.allclose(p0_1.loc, np.array([-1,0,0]).reshape(3,1)) )
		assert( np.allclose(p0_1.ori, p0.ori) )
		assert( np.allclose(p0_1.time, p0.time) )

		p2 = util.Pose(np.zeros((3,1)), np.array([math.pi/6, 0, 0])) #rotate 30 degrees in x axis
		p0_2 = p2.transform(p0)
		assert( np.allclose(p0_2.loc, p0.loc) )
		print p0_2.ori
		print cv2.Rodrigues(np.array([-math.pi/6, 0, 0]))[0]
		assert( np.allclose(p0_2.ori, cv2.Rodrigues(np.array([-math.pi/6, 0, 0]))[0]) )
		assert( np.allclose(p0_2.time, p0.time) )
		p1_2 = p2.transform(p1)
		assert( np.allclose(p1_2.loc, p1.loc) )
		assert( np.allclose(p1_2.ori, cv2.Rodrigues(np.array([-math.pi/6, 0, 0]))[0]) )
		assert( np.allclose(p1_2.time, p1.time) )


