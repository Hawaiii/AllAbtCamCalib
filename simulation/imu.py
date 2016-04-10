import camera as cam
import numpy as np
import numpy.matlib as matlib
import exp_util as util
import cv2
import math

def spiral_motion():
	"""
	Generating location with sin in each axis.
	TODO: generate orientation
	TODO: generate time stamp

	Returns:
		a list of (TODO: time-stamped) extrinsics
	"""
	extrins = []
	theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
	r = 150
	x = r * np.sin(theta)
	y = r * np.sin(theta*1.5)
	z = r * np.sin(theta*1.2)

	board = util.gen_calib_board(7,8,23, np.asarray([[0,0,1000]]), np.zeros([1,3]), 0)
	fov = (120.0, 80.0) # (x,y)degrees x->width
	sensor_ratio = 0.8 # height/width
	increment_ratio = 0.02
	fake_fx = 1 / ( 2 * math.tan((fov[0]/180)*3.14 /2) )
	fake_fy = sensor_ratio / (2 * math.tan( (fov[1]/180)*3.14 /2  ) )
	fake_k = np.asarray([[fake_fx, 0, 1], [0, fake_fy, sensor_ratio], [0,0,1]])
	R_init = np.eye(3)
	R_last = R_init
	boundary = np.asarray( [ board[0], board[6], board[49], board[55] ] ).reshape(4,3).T


	for i in range(len(theta)):
		trans_vec = np.asarray([x[i], y[i], z[i]])
		flag = True
		while flag:
			flag = False
			R_increment = cv2.Rodrigues(np.random.randn(3,1) * increment_ratio)[0]
			R_last = R_increment.dot(R_last)

			projection = R_last.dot( boundary)  + matlib.repmat(trans_vec, 4, 1).T
			projection = projection / matlib.repmat( projection[-1,:],3,1)
			for i in range(4):
				if projection[0,i] > 1 or projection[1,i] > sensor_ratio or projection[0,i] < 0 or projection[1,i] < 0:
					flag = True
				else:
					# print projection
				pass
			pass
		# import pdb; pdb.set_trace()

		ext = cam.Extrinsics(trans_vec, None, R_last)
		extrins.append(ext)


	return extrins, board
