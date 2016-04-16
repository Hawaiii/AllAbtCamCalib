"""
For visualizing calibration targets, cameras, etc.
"""
import exp_util as util
import board

import numpy as np

from mpl_toolkits.mplot3d.axes3d import Axes3D
from mpl_toolkits.mplot3d import axes3d

import matplotlib.pyplot as plt
import matplotlib.colors as colors
from matplotlib.backends.backend_pdf import PdfPages


# def plot_calib_boards(boards, board_dim, fax=None):
# 	"""
# 	Plots a board in 3D

# 	Args:
# 		boards: a list of dictionaries, where each dictionary is a board
# 		board_dim: (board_width, board_height)
# 	"""
# 	if fax:
# 		ax = fax
# 	else:
# 		fig = plt.figure()
# 		ax = fig.add_subplot(111, projection='3d')

# 	clist = colors.cnames.keys()
# 	for i in xrange(len(boards)):
# 		board = boards[i]
# 		X, Y, Z = util.board_dict2array(board, board_dim)

# 		ax.plot_wireframe(X, Y, Z, color=clist[i])
	
# 	if not fax:
# 		plt.show()
# 	return ax

def compare_board_estimations(esti_extrinsics, board, board_dim, \
								actual_boards, save_name=None):
	"""
	Plots true and estimated boards on the same figure
	Args:
		esti_extrinsics: dictionary, keyed by image number, values are Extrinsics
		board:
		board_dim: (board_width, board_height)
		actual_boards: list of dictionaries
		save_name: filename, string
	"""
	if save_name:
		pp = PdfPages(save_name)
	plt.clf()

	for i in xrange(len(actual_boards)):
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')

		act_board = actual_boards[i]
		aX, aY, aZ = util.board_dict2array(act_board, board_dim)
		ax.plot_wireframe(aX, aY, aZ, color='b')

		if i in esti_extrinsics:
			esti_loc = esti_extrinsics[i].trans_vec
			esti_board = util.move_board(board, esti_loc)
			eX, eY, eZ = util.board_dict2array(esti_board, board_dim)
			ax.plot_wireframe(eX, eY, eZ, color='r')

		if pp:
			pp.savefig()
		else:
			plt.show()
	if pp:
		pp.close()

def plot_all_chessboards_in_camera(img_pts, img_size, save_name=None):
	"""
	Args:
		img_pts: list of dictionaries, each representing a captured board
		img_size: (img_width, img_height)
		save_name: string, filename to save plot pdf to.
	"""
	if save_name:
		pp = PdfPages(save_name)
	plt.clf()

	for i in range(len(img_pts)):
		viewed_pts = np.asarray(img_pts[i].values())
		plt.axis([0, img_size[0], 0, img_size[1]])
		plt.grid(True)
		if viewed_pts.size == 0:
			print "chessboard " + str(i) + " is not seen in camera\n"
		else:
			plt.plot(viewed_pts[:,0,:], viewed_pts[:,1,:], 'ro')
		plt.gca().invert_yaxis()
		plt.ylabel('chessboard' + str(i))
		if pp:
			pp.savefig()
		else:
			plt.show()
		plt.clf()

	# Plot all points on images of whole board on the same page
	# Assuming at least one saw all points on board
	tot_pts_num = 0
	for i in range(len(img_pts)):
		if len(img_pts[i]) > tot_pts_num:
			tot_pts_num = len(img_pts[i])
	plt.axis([0, img_size[0], 0, img_size[1]])
	plt.grid(True)
	for i in range(len(img_pts)):
		if len(img_pts[i]) == tot_pts_num:
			viewed_pts = np.asarray(img_pts[i].values())
			plt.plot(viewed_pts[:,0,:], viewed_pts[:,1,:], 'ro')
	plt.ylabel('all points used')
	if pp:
		pp.savefig()
	else:
		plt.show()

	if pp:
		pp.close()
	else:
		plt.close('all')

def write_esti_results(estimations, true_cam, save_name_pre):
	"""
	TODO: compare extrinsics
	Args:
		estimations: list of Cameras from calibration results
		true_cam: actual camera parameters
		save_name_pre: filename without .txt or .pdf extensions
	"""
	ftxt = open(save_name_pre+'.txt', 'w')
	fpdf = PdfPages(save_name_pre+'.pdf')

	# focal length x
	fx_arr = np.asarray([est_cam.intrinsics.intri_mat[0,0] for est_cam in estimations])
	print >> ftxt, 'focal length x\tground truth:{0}\testimation mean:{1}\testimation std:{2}'.format(\
		true_cam.intrinsics.intri_mat[0,0], np.mean(fx_arr), np.std(fx_arr))
	fig, ax = plt.subplots()
	bars = plt.bar(range(len(fx_arr)), fx_arr)
	plt.ylabel('focal length x')
	fpdf.savefig()

	# focal length y
	fy_arr = np.asarray([est_cam.intrinsics.intri_mat[1,1] for est_cam in estimations])
	print >> ftxt, 'focal length y\tground truth:{0}\testimation mean:{1}\testimation std:{2}'.format(\
		true_cam.intrinsics.intri_mat[1,1], np.mean(fy_arr), np.std(fy_arr))
	fig, ax = plt.subplots()
	bars = plt.bar(range(len(fy_arr)), fy_arr)
	plt.ylabel('focal length y')
	fpdf.savefig()

	# principal point x
	px_arr = np.asarray([est_cam.intrinsics.intri_mat[0,2] for est_cam in estimations])
	print >> ftxt, 'principal point x\tground truth:{0}\testimation mean:{1}\testimation std:{2}'.format(\
		true_cam.intrinsics.intri_mat[0,2], np.mean(px_arr), np.std(px_arr))
	fig, ax = plt.subplots()
	bars = plt.bar(range(len(px_arr)), px_arr)
	plt.ylabel('principal point x')
	fpdf.savefig()

	# principal point y
	py_arr = np.asarray([est_cam.intrinsics.intri_mat[1,2] for est_cam in estimations])
	print >> ftxt, 'principal point y\tground truth:{0}\testimation mean:{1}\testimation std:{2}'.format(\
		true_cam.intrinsics.intri_mat[1,2], np.mean(py_arr), np.std(py_arr))
	fig, ax = plt.subplots()
	bars = plt.bar(range(len(py_arr)), py_arr)
	plt.ylabel('principal point y')
	fpdf.savefig()

	# extrinsics diff r1
	# extrinsics diff r2
	# extrinsics diff r3
	# extrinsics diff t1
	# extrinsics diff t2
	# extrinsics diff t3

	ftxt.close()
	fpdf.close()

	print 'write_esti_results not FULLY implemented yet!'

# def plot_directions(orientations, location=np.asarray([0,0,0])):
# 	fig = plt.figure()
# 	ax = fig.add_subplot(111, projection='3d')
# 	for orient in orientations:
# 		ax.quiver(location[0], location[1], location[2], \
# 			orient[0], orient[1], orient[2], pivot='tail')
# 	ax.set_xlim(-1,1)
# 	ax.set_ylim(-1,1)
# 	ax.set_zlim(-1,1)
# 	plt.show()

# def plot_locations(locations):
# 	fig = plt.figure()
# 	ax = fig.gca(projection='3d')
# 	x = [loc[0] for loc in locations]
# 	y = [loc[1] for loc in locations]
# 	z = [loc[2] for loc in locations]
# 	ax.plot(x, y, z, label='locations')
# 	ax.legend()
# 	plt.show()

def plot_poses(extrinsics, invert=False, connectpath=True, fax=None, clr=None):
	"""
	Args:
		extrinsics: a list of Extrinsics
		invert: plots location of -Rt and orientation of R' when true; 
		        plots location of t and orientations of R when false
		connectpath: draws a path that connects the locations when true
	"""
	if fax:
		ax = fax
	else:
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')

	if invert:		
		xyz = [ext.get_inv_location() for ext in extrinsics]
		x = [loc[0,0] for loc in xyz]
		y = [loc[1,0] for loc in xyz]
		z = [loc[2,0] for loc in xyz]
	else:
		x = [ext.trans_vec[0] for ext in extrinsics]
		y = [ext.trans_vec[1] for ext in extrinsics]
		z = [ext.trans_vec[2] for ext in extrinsics]

	z_vec = np.asarray([0,0,1])
	if invert:
		u = [np.dot(ext.rot_mat.T[0,:],z_vec) for ext in extrinsics]
		v = [np.dot(ext.rot_mat.T[1,:],z_vec) for ext in extrinsics]
		w = [np.dot(ext.rot_mat.T[2,:],z_vec) for ext in extrinsics]
	else:
		u = [np.dot(ext.rot_mat[0,:],z_vec) for ext in extrinsics]
		v = [np.dot(ext.rot_mat[1,:],z_vec) for ext in extrinsics]
		w = [np.dot(ext.rot_mat[2,:],z_vec) for ext in extrinsics]

	if connectpath:
		ax.plot(x,y,z,label='path',color=clr)
	ax.quiver(x,y,z,u,v,w,pivot='tail',length=0.05, color=clr)

	return ax

def plot_camera_pose(extrin, save_name=None):
	"""
	Plots the location of the camera given extrinsics of board
	@TODO: Currently labels image number text on the location of camera, could
	       add in the orientation and a 3D camera figure
	Args:
		extrin: a dictionary keyed by image number, whose values are Extrinsics
		save_name: if save_name is provided, figure will be saved to that name;
		           otherwise, the figure will be shown on screen
	"""
	print 'plot_camera_pose not implemented yet!'
	pass

def plot_camera_with_rays(cam_extrin, rays, invert=True):
	"""
	Args:
		cam_extrin: Extrinsics
		rays: list of tuples, (pt3d, ray_vec) as returned by ray_from_pixel
		invert: plots camera center at -Rt when true
	"""
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	# plot camera
	if invert:
		# x = [-np.dot(cam_extrin.rot_mat[0,:], cam_extrin.trans_vec) ]
		# y = [-np.dot(cam_extrin.rot_mat[1,:], cam_extrin.trans_vec) ]
		# z = [-np.dot(cam_extrin.rot_mat[2,:], cam_extrin.trans_vec) ]
		xyz = cam_ext.get_inv_location()
		x = xyz[0,0]
		y = xyz[1,0]
		z = xyz[2,0]
	else:
		x = [cam_extrin.trans_vec[0,0]]
		y = [cam_extrin.trans_vec[1,0]]
		z = [cam_extrin.trans_vec[2,0]]
		
	z_vec = np.asarray([0,0,1])
	u = [np.dot(cam_extrin.rot_mat[0,:],z_vec)]
	v = [np.dot(cam_extrin.rot_mat[1,:],z_vec)]
	w = [np.dot(cam_extrin.rot_mat[2,:],z_vec)]
	ax.quiver(x,y,z,u,v,w,pivot='tail',length=0.5)
	ax.text(x[0]+u[0],y[0]+v[0],z[0]+w[0],'camera',None)

	# plot rays
	for ray in rays:
		ax.quiver(ray[0][0], ray[0][1], ray[0][2], ray[1][0], ray[1][1], ray[1][2], \
			pivot='tail',color='m')

	ax.set_xlim(ray[0][0]-1,ray[0][0]+1)
	ax.set_ylim(ray[0][1]-1,ray[0][1]+1)
	ax.set_zlim(ray[0][2]-1,ray[0][2]+1)

	plt.show()
