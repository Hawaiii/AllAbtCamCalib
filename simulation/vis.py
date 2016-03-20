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

