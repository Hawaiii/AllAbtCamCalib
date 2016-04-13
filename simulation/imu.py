import camera as cam
import numpy as np
import numpy.matlib as matlib
import exp_util as util
import cv2
import math

def spiral_motion(board, board_dim):
	"""
	A hard coded motion.
	Generates location with sine in each axis.
	Generates orientation while keeping target in view.
	TODO: generate time stamp

	Args:
		board: dictionary keyed by point id whose values are 3D position of 
				control points
		board_dim: (board_height, board_width)

	Returns:
		extrins: a list of (TODO: time-stamped) extrinsics
	"""
	extrins = []
	
	# Generate location
	theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
	r = 150
	x = r * np.sin(theta)
	y = r * np.sin(theta*1.5)
	z = r * np.sin(theta*1.2)

	# Generate timestamp: trying even division first?
	tot_length = 10 * 10**9 #10s converted to ns
	ts_s = 1
	ts = np.round(np.linspace(ts_s, tot_length+ts_s, 100)).astype(int)

	# Generate orientation
	fov = (120.0, 80.0) # (x,y)degrees x->width
	sensor_ratio = 0.8 # height/width
	increment_ratio = 0.09
	fake_fx = 1 / ( 2 * math.tan((fov[0]/180)*3.14 /2) )
	fake_fy = sensor_ratio / (2 * math.tan( (fov[1]/180)*3.14 /2  ) )
	fake_k = np.asarray([[fake_fx, 0, 1.0/2], [0, fake_fy, sensor_ratio/2], [0,0,1]])
	R_init = np.eye(3)
	R_last = R_init
	boundary = np.asarray( [ board[0], board[board_dim[1]-1], \
		board[(board_dim[0]-1)*board_dim[1]], board[board_dim[0]*board_dim[1]-1] ] ).reshape(4,3).T

	for i in range(len(theta)):
		trans_vec = np.asarray([x[i], y[i], z[i]])
		flag = True
		while flag:
			flag = False
			R_increment = cv2.Rodrigues(np.random.randn(3,1) * increment_ratio)[0]
			R_last = R_increment.dot(R_last)
			
			projection = R_last.dot( boundary)  + matlib.repmat(-R_last.dot(trans_vec), 4, 1).T
			projection = fake_k.dot(projection)
			projection = projection / matlib.repmat( projection[-1,:],3,1)

			#import pdb; pdb.set_trace()
			for j in range(4):
				if projection[0,j] > 1 or projection[1,j] > sensor_ratio or projection[0,j] < 0 or projection[1,j] < 0:
					flag = True
					#print projection
					#print projection[0,i] > 1, projection[1,i] > sensor_ratio, projection[0,i] < 0, projection[1,i] < 0
		ext = cam.Extrinsics.init_with_rotation_matrix(np.array([-R_last.dot(trans_vec)]), R_last, ts[i])
		
		extrins.append(ext)

	return extrins

def transform_motion(orig_motion, rel_extrin):
	"""
	If orig_motion is camera motion and rel_extrin specifies 
	"""