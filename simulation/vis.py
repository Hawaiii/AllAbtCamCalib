"""
For visualizing calibration targets, cameras, etc.
"""
from mpl_toolkits.mplot3d.axes3d import Axes3D
from mpl_toolkits.mplot3d import axes3d

import matplotlib.pyplot as plt
import matplotlib.colors as colors

import numpy as np
import exp_util as util

def plot_calib_boards(boards, board_dim):
	"""
	Plots a board in 3D

	Args:
		boards: a list of dictionaries, where each dictionary is a board
		board_dim: (board_height, board_width)
	"""
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	clist = colors.cnames.keys()

	for i in xrange(len(boards)):
		board = boards[i]
		X, Y, Z = util.board_dict2array(board, board_dim)
		ax.plot_wireframe(X, Y, Z, color=clist[i])
		print X[0,0], Y[0,0], Z[0,0]
	plt.show()

def plot_all_chessboards_in_camera(img_pts, true_cam):
	for i in range(len(img_pts)):
		viewed_pts = np.asarray(img_pts[i].values())
		plt.axis([0, true_cam.size[1], 0, true_cam.size[0]])
		plt.grid(True)
		if viewed_pts.size == 0:
			print "chessboard " + str(i) + " is not viewable\n"
			continue
		plt.plot(viewed_pts[:,1,:], viewed_pts[:,0,:], 'ro')
		plt.ylabel('chessboard' + str(i))
		plt.show()
	plt.close('all')
