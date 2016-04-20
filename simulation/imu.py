import camera as cam
import exp_util as util

import numpy as np
import numpy.matlib as matlib
import cv2
import math
import csv

# def spiral_motion(board, board_dim, camera):
# 	"""
# 	A hard coded motion.
# 	Generates location with sine in each axis.
# 	Generates orientation while keeping target in view.
# 	Generates even time stamp.

# 	Args:
# 		board: dictionary keyed by point id whose values are 3D position of 
# 				control points
# 		board_dim: (board_width, board_height)
# 		camera: a Camera


# 	Returns:
# 		extrins: a list of time-stamped extrinsics
# 	"""
# 	extrins = []
	
# 	# Generate location
# 	theta = np.linspace(-4 * np.pi, 4 * np.pi, 600)
# 	r = 150
# 	x = r * np.sin(theta)
# 	y = r * np.sin(theta*1.5)
# 	z = r * np.sin(theta*1.2)

# 	# Generate timestamp: trying even division first?
# 	tot_length = 3 * 10**9 #10s converted to ns
# 	ts_s = 1
# 	ts = np.round(np.linspace(ts_s, tot_length+ts_s, len(theta))).astype(int)

# 	# Generate orientation
# 	increment_ratio = 0.001
# 	R_init = np.eye(3)
# 	R_last = R_init
# 	boundary = np.concatenate( ( board[0], board[board_dim[0]-1], \
# 		board[(board_dim[1]-1)*board_dim[0]], board[board_dim[1]*board_dim[0]-1] ), axis=1)
# 	# fov = (120.0, 80.0) # (x,y)degrees x->width
# 	# sensor_ratio = 0.8 # height/width
# 	# fake_fx = 1 / ( 2 * math.tan((fov[0]/180)*3.14 /2) )
# 	# fake_fy = sensor_ratio / (2 * math.tan( (fov[1]/180)*3.14 /2  ) )
# 	# fake_k = np.asarray([[fake_fx, 0, 1.0/2], [0, fake_fy, sensor_ratio/2], [0,0,1]])
	
# 	for i in range(len(theta)):
# 		trans_vec = np.asarray([x[i], y[i], z[i]]).reshape(3,1)
# 		flag = True
# 		while flag:
# 			flag = False
# 			R_increment = cv2.Rodrigues(np.random.randn(3,1) * increment_ratio)[0]
# 			R_last_test = R_increment.dot(R_last)
			
# 			for j in range(4):
# 				projection = camera.project_point(\
# 					cam.Extrinsics.init_with_rotation_matrix((-R_last_test.dot(trans_vec)), \
# 															R_last_test), \
# 												boundary[:,j])
# 				if projection[0,0] >= camera.size[0] or projection[0,0] < 0 or \
# 					projection[1,0] >= camera.size[1] or projection[1,0] < 0:
# 					flag = True
# 				else:
# 					R_last = R_last_test

# 			# projection = R_last_test.dot( boundary)  + matlib.repmat(-R_last_test.dot(trans_vec), 4, 1).T
# 			# projection = fake_k.dot(projection)
# 			# projection = projection / matlib.repmat( projection[-1,:],3,1)
# 			# for j in range(4):
# 			# 	if projection[0,j] > 1 or projection[1,j] > sensor_ratio or projection[0,j] < 0 or projection[1,j] < 0:
# 			# 		flag = True
# 			# 	else:
# 			# 		R_last = R_last_test
# 			# 		#print projection
# 			# 		#print projection[0,i] > 1, projection[1,i] > sensor_ratio, projection[0,i] < 0, projection[1,i] < 0
# 		ext = cam.Extrinsics.init_with_rotation_matrix(\
# 			-R_last.dot(trans_vec), R_last, ts[i])		
# 		extrins.append(ext)

# 	return extrins


def read_motion(filename, sample_ratio=1):
	"""
	Reads headset motion captured with mocap. All lengths in m.
	Args:
		filename: csv file to read from, column in order: 
					time stamp; position x, y, z; orientation x,y,z,w; 
					linear velocity x, y, z; angular velocity x, y, z;
					linear acceleration x, y, z; angular acceleration x, y, z
		sample_ratio: read one sample out of sample_ratio rows
	Returns:
		motion: a list of Poses
	"""
	motion = []

	csvfile = open(filename,'rb')
	reader = csv.reader(csvfile, delimiter=',')
	cnt = 0
	tcnt = 0
	for row in reader: 
		if cnt == 0: # skip header
			cnt += 1
			continue

		# fast debugging
		# if cnt == 3000:
		# 	break

		if cnt % sample_ratio == 0:
			ts = int(row[0])
			loc = np.array([float(row[1]), float(row[2]), float(row[3])]).reshape(3,1)
			rot_mat = util.quaternion2mat(float(row[4]), float(row[5]), float(row[6]), float(row[7]))

			lvel = np.array([float(row[8]), float(row[9]), float(row[10])])
			avel = np.array([float(row[11]), float(row[12]), float(row[13]) ])
			lacc = np.array([float(row[14]), float(row[15]), float(row[16])])
			aacc = np.array([float(row[17]), float(row[18]), float(row[19]) ])

			pose = util.Pose(loc, rot_mat, ts, lvel, avel, lacc, aacc)
			motion.append(pose)
			tcnt += 1

		cnt += 1
	print 'read', cnt, 'poses, recorded', tcnt,'poses'
	return motion
	
# def read_motion(filename, sample_ratio=1):
# 	"""
# 	All lengths in m.
# 	"""
# 	extrins = []

# 	csvfile = open(filename,'rb')
# 	reader = csv.reader(csvfile, delimiter=',')
# 	cnt = 0
# 	tcnt = 0
# 	for row in reader: #time stamp; position x, y, z; orientation x,y,z,w
# 		if cnt == 0:
# 			cnt += 1
# 			continue

# 		# fast debugging
# 		# if cnt == 300:
# 		# 	break
# 		if cnt % sample_ratio == 0:
# 			ts = int(row[0])
# 			trans_vec = np.array([float(row[1]), float(row[2]), float(row[3])]).reshape(3,1)
# 			rot_mat = util.quaternion2mat(float(row[4]), float(row[5]), float(row[6]), float(row[7]))

# 			lvel = np.array([float(row[8]), float(row[9]), float(row[10])])
# 			avel = np.array([float(row[11]), float(row[12]), float(row[13]) ])
# 			lacc = np.array([float(row[14]), float(row[15]), float(row[16])])
# 			aacc = np.array([float(row[17]), float(row[18]), float(row[19]) ])

# 			ext = cam.Extrinsics.init_with_rotation_matrix(-rot_mat.dot(trans_vec), rot_mat, ts, \
# 				lvel, lacc, avel, aacc)
# 			extrins.append(ext)
# 			tcnt += 1

# 		cnt += 1
# 	print 'read', cnt, 'poses, recorded', tcnt,'poses'
# 	return extrins

def transform_motion(orig_motion, rel_pose, sample_ratio, transform_deriviatives=False):
	"""
	If orig_motion is camera motion and rel_extrin specifies transformation from 
	camera to imu then this function returns the imu motion.

	Args:
		orig_motion: a list of time-stamped Extrinsics
		rel_pose: Pose, relative pose from orig_motion object frame
		sample_ratio: 
	Returns:
		new_motion: a list of time-stamped Extrinsics
	"""
	new_motion = []
	cnt = 0
	tcnt = 0
	for orig_p in orig_motion:
		if cnt % sample_ratio == 0:
			new_p = orig_p.transform_p2w(rel_pose, transform_deriviatives)
			new_motion.append(new_p)
			tcnt += 1
		cnt += 1
	print 'transform motion read', cnt,' poses and transformed', tcnt, 'poses.'
	return new_motion
# def transform_motion(orig_motion, rel_extrin, sample_ratio):
# 	"""
# 	If orig_motion is camera motion and rel_extrin specifies transformation from 
# 	camera to imu then this function returns the imu motion.
# 	@TODO: add time stamp offset from rel_extrin

# 	Args:
# 		orig_motion: a list of time-stamped Extrinsics
# 		rel_extrin: Extrinsics, relative pose ([R|t] transforms from camera to imu)
# 		sample_ratio: 
# 	Returns:
# 		new_motion: a list of time-stamped Extrinsics
# 	"""
# 	new_motion = []
# 	imu_loc_in_camera = -rel_extrin.rot_mat.T.dot(rel_extrin.trans_vec)
# 	imu_loc_in_camera = np.concatenate((imu_loc_in_camera, np.ones((1,1))), axis=0)
# 	imu_orien_to_camera = rel_extrin.rot_mat.T
# 	# print 'imu_loc_in_cam', imu_loc_in_camera
# 	# print 'imu_orien_to_cam', imu_orien_to_camera

# 	cnt = 0
# 	pcnt = 0
# 	for orig_e in orig_motion:
# 		if cnt % sample_ratio == 0:
# 			imu_loc_in_world = orig_e.get_homo_trans_matrix_inv().dot(imu_loc_in_camera)
# 			imu_loc_in_world = imu_loc_in_world / matlib.repmat( imu_loc_in_world[-1,:], 4, 1)
# 			imu_loc_in_world = imu_loc_in_world[0:3,:]

# 			imu_orien_to_world = orig_e.rot_mat.dot(imu_orien_to_camera)

# 			# print orig_e.trans_vec, (-imu_orien_to_world.dot(imu_loc_in_world)).T
# 			# print
			
# 			new_e = cam.Extrinsics.init_with_rotation_matrix(\
# 				(-imu_orien_to_world.dot(imu_loc_in_world)), imu_orien_to_world, orig_e.time_stamp)
# 			new_motion.append(new_e)
# 			pcnt += 1

# 		cnt += 1
# 	print 'transform motion has', pcnt, 'poses.'
# 	return new_motion

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
		# d2t = (imu_motion[i+1].time_stamp - imu_motion[i-1].time_stamp) * (10**-9)
		dt1 = (imu_motion[i+1].time_stamp - imu_motion[i].time_stamp) * (10**-9)
		dt2 = (imu_motion[i].time_stamp - imu_motion[i-1].time_stamp) * (10**-9)
		d2t = dt1 + dt2
		
		# Gyroscope measurements
		drdt = (imu_motion[i+1].rot_vec - imu_motion[i-1].rot_vec)/d2t
		drdt = drdt.reshape(3,1)
                reading[i,1:4] = imu_motion[i].rot_mat.dot(drdt).reshape(1,3)
		
		# Acceleration - gravity measurements 
		v1 = (imu_motion[i+1].get_inv_location() - imu_motion[i].get_inv_location())/dt1
		v2 = (imu_motion[i].get_inv_location() - imu_motion[i-1].get_inv_location())/dt2
		acc = (v1-v2)*2/d2t - np.cross(drdt.flatten(), (v1+v2).flatten()).reshape(3,1)
		acc = acc / (10**3) # convert back to m/s^2
		reading[i,4:7] = imu_motion[i].rot_mat.dot(acc).reshape(1,3)
		#reading[i,4:7] = imu_motion[i].rot_mat.dot(acc - gravity.reshape(3,1)).reshape(1,3)

		if save_name:
			to_write = [str(reading[i,0])]
			for j in xrange(1, 7):
				to_write.append(reading[i,j])
			writer.writerow(to_write)
	# if save_name:
		# np.savetxt(save_name, reading, delimiter=',')

	return reading

def get_imu_readings(imu_motion, gravity_in_world, save_name):
	"""
	Writes imu readings to csv file.
	Args:
		imu_motion: a list of Poses
		gravity_in_world: gravity vector in target
		save_name: name of csv file to save to
	"""
	writer=csv.writer(open(save_name,'wb'))
	header=['timestamp','omega_x','omega_y','omega_z','alpha_x','alpha_y','alpha_z']
	writer.writerow(header)

	for i in xrange(len(imu_motion)):
		to_write = []

		to_write.append(imu_motion[i].time)

		# Gyroscope measurements: angular velocity in imu frame
		gyro = imu_motion[i].ori.T.dot(imu_motion[i].ang_vel) #world to pose
		for j in xrange(3):
			to_write.append(gyro[j,0])

		# Acceleration - gravity measurements 
		acc = imu_motion[i].lin_acc
		acc = imu_motion[i].ori.T.dot(acc-gravity_in_world.reshape(3,1)) #world to pose
		for j in xrange(3):
			to_write.append(acc[j,0])

		writer.writerow(to_write)
# def get_imu_readings(imu_motion, gravity_in_target, save_name):
# 	"""
# 	Writes imu readings to csv file.
# 	"""
# 	writer=csv.writer(open(save_name,'wb'))
# 	header=['timestamp','omega_x','omega_y','omega_z','alpha_x','alpha_y','alpha_z']
# 	writer.writerow(header)

# 	for i in xrange(len(imu_motion)):
# 		to_write = []

# 		to_write.append(imu_motion[i].time_stamp)

# 		# Gyroscope measurements: angular velocity in imu frame
# 		gyro = imu_motion[i].rot_mat.dot(imu_motion[i].ang_vel)
# 		for j in xrange(3):
# 			to_write.append(gyro[j,0])

# 		# Acceleration - gravity measurements 
# 		acc = imu_motion[i].linear_acc # convert back to m/s^2
# 		# acc = imu_motion[i].rot_mat.dot(acc)
# 		acc = imu_motion[i].rot_mat.dot(acc-gravity_in_target.reshape(3,1))
# 		for j in xrange(3):
# 			to_write.append(acc[j,0])

# 		writer.writerow(to_write)
