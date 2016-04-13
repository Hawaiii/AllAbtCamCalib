"""
Makes synthetic data for Kalibr.
"""
import sys
sys.path.append('../calib/')

import imu
import camera as cam
import targets
import vis
import exp_util as util


import numpy as np
import matplotlib.pyplot as plt

board_dim = (5, 8) #DONT CHANGE as this is the dimension of data/chess.png
board = util.gen_calib_board(board_dim[0],board_dim[1],23, np.asarray([[0,0,600]]), np.zeros([1,3]), 0)

extrins = imu.spiral_motion(board, board_dim)
# ax = vis.plot_poses(extrins, invert = True)
# ax = vis.plot_calib_boards( [ board ], (7,8), fax=ax)
# ax.set_aspect('equal')
# plt.show()

camera = cam.Camera.make_pinhole_camera()
camera.intrinsics.radial_dist = np.zeros((1,3))
camera.intrinsics.tang_dist = np.zeros((1,2))

Hs = camera.calc_homography(extrins, board, board_dim)

