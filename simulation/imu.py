import camera as cam
import exp_util as util

import numpy as np
import numpy.matlib as matlib
import cv2
import math
import csv

def spiral_motion(board, board_dim, camera):
	"""
	A hard coded motion.
	Generates location with sine in each axis.
	Generates orientation while keeping target in view.
	Generates even time stamp.

	Args:
		board: dictionary keyed by point id whose values are 3D position of 
				control points
		board_dim: (board_width, board_height)
		camera: a Camera

	Returns:
		extrins: a list of time-stamped extrinsics
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
	ts = np.round(np.linspace(ts_s, tot_length+ts_s, len(theta))).astype(int)

	# Generate orientation
	increment_ratio = 0.09
	R_init = np.eye(3)
	R_last = R_init
	boundary = np.concatenate( ( board[0], board[board_dim[0]-1], \
		board[(board_dim[1]-1)*board_dim[0]], board[board_dim[1]*board_dim[0]-1] ), axis=1)
	# fov = (120.0, 80.0) # (x,y)degrees x->width
	# sensor_ratio = 0.8 # height/width
	# fake_fx = 1 / ( 2 * math.tan((fov[0]/180)*3.14 /2) )
	# fake_fy = sensor_ratio / (2 * math.tan( (fov[1]/180)*3.14 /2  ) )
	# fake_k = np.asarray([[fake_fx, 0, 1.0/2], [0, fake_fy, sensor_ratio/2], [0,0,1]])
	
	for i in range(len(theta)):
		trans_vec = np.asarray([x[i], y[i], z[i]]).reshape(3,1)
		flag = True
		while flag:
			flag = False
			R_increment = cv2.Rodrigues(np.random.randn(3,1) * increment_ratio)[0]
			R_last_test = R_increment.dot(R_last)
			
			for j in range(4):
				projection = camera.project_point(\
					cam.Extrinsics.init_with_rotation_matrix((-R_last_test.dot(trans_vec)), \
															R_last_test), \
												boundary[:,j])
				if projection[0,0] >= camera.size[0] or projection[0,0] < 0 or \
					projection[1,0] >= camera.size[1] or projection[1,0] < 0:
					flag = True
				else:
					R_last = R_last_test

			# projection = R_last_test.dot( boundary)  + matlib.repmat(-R_last_test.dot(trans_vec), 4, 1).T
			# projection = fake_k.dot(projection)
			# projection = projection / matlib.repmat( projection[-1,:],3,1)
			# for j in range(4):
			# 	if projection[0,j] > 1 or projection[1,j] > sensor_ratio or projection[0,j] < 0 or projection[1,j] < 0:
			# 		flag = True
			# 	else:
			# 		R_last = R_last_test
			# 		#print projection
			# 		#print projection[0,i] > 1, projection[1,i] > sensor_ratio, projection[0,i] < 0, projection[1,i] < 0
		ext = cam.Extrinsics.init_with_rotation_matrix(\
			-R_last.dot(trans_vec).reshape(1,3), R_last, ts[i])		
		extrins.append(ext)

	return extrins

def transform_motion(orig_motion, rel_extrin):
	"""
	If orig_motion is camera motion and rel_extrin specifies transformation from 
	camera to imu then this function returns the imu motion.
	@TODO: add time stamp offset from rel_extrin

	Args:
		orig_motion: a list of time-stamped Extrinsics
		rel_extrin: Extrinsics, relative pose ([R|t] transforms from camera to imu)
	Returns:
		new_motion: a list of time-stamped Extrinsics
	"""
	new_motion = []
	imu_loc_in_camera = -rel_extrin.rot_mat.T.dot(rel_extrin.trans_vec)
	imu_loc_in_camera = np.concatenate((imu_loc_in_camera, np.ones((1,1))), axis=0)
	imu_orien_to_camera = rel_extrin.rot_mat.T
	# print 'imu_loc_in_cam', imu_loc_in_camera
	# print 'imu_orien_to_cam', imu_orien_to_camera

	for orig_e in orig_motion:
		imu_loc_in_world = orig_e.get_homo_trans_matrix_inv().dot(imu_loc_in_camera)
		imu_loc_in_world = imu_loc_in_world / matlib.repmat( imu_loc_in_world[-1,:], 4, 1)
		imu_loc_in_world = imu_loc_in_world[0:3,:]

		imu_orien_to_world = orig_e.rot_mat.dot(imu_orien_to_camera)

		# print orig_e.trans_vec, (-imu_orien_to_world.dot(imu_loc_in_world)).T
		# print
		
		new_e = cam.Extrinsics.init_with_rotation_matrix(\
			(-imu_orien_to_world.dot(imu_loc_in_world)), imu_orien_to_world, orig_e.time_stamp)
		new_motion.append(new_e)
	return new_motion

def gen_imu_readings(imu_motion, gravity, save_name='results/imu0.csv'):
	"""
	Generate imu readings and write to csv file.
	Args:
		imu_motion: a list of time-stamped Extrinsics
		gravity: 1x3 numpy array, gravity in target coordinate
	Returns: 
		reading: N x 7 numpy array, where columns are: timestamp, gyroscope x, 
				gyroscope y, gyroscope z, accelerometer x, accelerometer y, 
				accelerometer z
	Also writes reading to csv file of given name.
	"""
	reading = np.zeros((len(imu_motion), 7))
	if save_name:
		writer=csv.writer(open(save_name,'wb'))
		header=['timestamp','omega_x','omega_y','omega_z','alpha_x','alpha_y','alpha_z']
		writer.writerow(header)

	for i in xrange(1,len(imu_motion)-1):
		reading[i,0] = imu_motion[i].time_stamp

		# time in seconds
		dt = (imu_motion[i+1].time_stamp - imu_motion[i-1].time_stamp) * (10**-9)
		
		# Gyroscope measurements
		drdt = (imu_motion[i+1].rot_vec - imu_motion[i-1].rot_vec)/dt
		reading[i,1:4] = imu_motion[i].rot_mat.dot(drdt.reshape(3,1)).reshape(1,3)
		
		# Acceleration - gravity measurements 
		acc = (imu_motion[i+1].get_inv_location() - imu_motion[i-1].get_inv_location())/(dt*dt)
		reading[i,4:7] = imu_motion[i].rot_mat.dot(acc - gravity.reshape(3,1)).reshape(1,3)

		if save_name:
			to_write = [str(reading[i,0])]
			for j in xrange(1, 7):
				to_write.append(reading[i,j])
			writer.writerow(to_write)
	# if save_name:
		# np.savetxt(save_name, reading, delimiter=',')

	return reading