# board at ( 1254 , 505 , 3.76316885531 ), rotation [-1.4262966   3.22082211  2.47569224]
# board at ( 56 , 839 , 1.36072283546 ), rotation [ 1.7867019   0.33523003  1.31729307]

# esti_cam:
# array([[  1.15279881e+03,   0.00000000e+00,   6.36783076e+02],
#        [  0.00000000e+00,   5.66591406e+02,   5.11186150e+02],
#        [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
# distCoeffs
# array([[ 0.01177302, -0.03552487, -0.00204971,  0.00083257,  0.03556318]])
# rvecs
# array([ 0.75128125, -1.42284365, -0.99762584])
# array([ 1.7465889 ,  0.55356987,  1.50258164])
# tvecs
# array([ 1.88412259, -0.0357347 ,  3.52297502])
# array([-0.69019904,  0.79330586,  1.36723636])


import exp_util as util
import camera as cam
import board as bd
import vis

import cv2
import numpy as np
import numpy.matlib as matlib
import math
import matplotlib.pyplot as plt
import copy

noise3d = 0
noise2d = 0.5
bh = 50
bw = 80
bs = 0.023
depth_min = 0.5 #m
depth_max = 5#m

true_cam = cam.Camera.make_pinhole_camera()

true_cam.intrinsics.radial_dist = 0*true_cam.intrinsics.radial_dist
true_cam.intrinsics.tang_dist = 0*true_cam.intrinsics.tang_dist

cam_loc = np.zeros((3,1))
cam_ori = np.zeros((3,1))
cam_extrin = util.Pose(cam_loc, cam_ori).extrinsics()

# Generate n boards
board = bd.Board.gen_calib_board((bw, bh), bs, \
	np.zeros((3,1)), np.zeros((3,1)), noise3d)
perfect_board = bd.Board.gen_calib_board((bw, bh), bs, \
	np.zeros((3,1)), np.zeros((3,1)), 0)
obs_list = []
# while len(obs_list) < n:
# FIRST BOARD
# choose a random pixel
pxl_x0 = 1254
pxl_y0 = 505
depth0 = 3.76316885531
# choose a random orientation
bd_ori0 = np.array([-1.4262966,   3.22082211,  2.47569224])

m_board = board.move_board_in_camera(true_cam, cam_extrin, (pxl_x0, pxl_y0), depth0, bd_ori0)
obs_list.append(m_board.get_points()) #3xN np array

# SECOND BOARD
# choose a random pixel
pxl_x1 = 56
pxl_y1 = 839
depth1 = 1.36072283546
# choose a random orientation
bd_ori1 = np.array([ 1.7867019,   0.33523003,  1.31729307])

m_board = board.move_board_in_camera(true_cam, cam_extrin, (pxl_x1, pxl_y1), depth1, bd_ori1)
obs_list.append(m_board.get_points()) #3xN np array

img_pts = true_cam.capture_images(cam_extrin, obs_list, noise2d)
esti_cam = cam.Camera.calibrate_camera(img_pts, perfect_board, true_cam.size)

# plot actual 3d points
# ax = vis.plot_camera_with_points(cam_extrin, obs_list)
# # vis.plot_all_chessboards_in_camera(img_pts, true_cam.size, seperate_plot=False)

# # plot recovered 3d points
# recons_list = []
# rmat0 = cv2.Rodrigues(np.array([ 0.75128125, -1.42284365, -0.99762584]))[0]
# tvec0 = np.array([ 1.88412259, -0.0357347 ,  3.52297502]).reshape(3,1)
# recons_list.append(rmat0.dot(board.get_points()) + matlib.repmat(tvec0, 1, board.num_points()))
# # ax.scatter(recons_list[0][0,:], recons_list[0][1,:], recons_list[0][2,:], color='r')

# rmat1 = cv2.Rodrigues(np.array([ 1.7465889 ,  0.55356987,  1.50258164]))[0]
# tvec1 = np.array([-0.69019904,  0.79330586,  1.36723636]).reshape(3,1)
# recons_list.append(rmat1.dot(board.get_points()) + matlib.repmat(tvec1, 1, board.num_points()))
# # ax.scatter(recons_list[1][0,:], recons_list[1][1,:], recons_list[1][2,:], color='r')
# vis.plot_camera_with_points(cam_extrin, obs_list+recons_list)
# plt.show()

# img_pts_n = esti_cam.capture_images(cam_extrin, recons_list, noise2d)
# # vis.plot_all_chessboards_in_camera(img_pts_n, esti_cam.size, seperate_plot=False)


# tot_pts_num = board.num_points()
# img_size = true_cam.size
# plt.axis([0, img_size[0], 0, img_size[1]])
# plt.grid(True)
# for i in range(len(img_pts)):
# 	plt.plot(img_pts[i][0,:], img_pts[i][1,:], 'ro')
# for i in range(len(img_pts_n)):
# 	plt.plot(img_pts_n[i][0,:], img_pts[i][1,:], 'bx')
# plt.gca().invert_yaxis()
# plt.ylabel('all points used')
# plt.show()

# Analyze error
# vis.write_esti_results([esti_cam], true_cam, \
	# save_name_pre='results/report_'+str(n)+'_3dn_'+str(noise3d)+'_2dn_'+str(noise2d)+'_bn_'+str(bh*bw)+'_bs_'+str(bs))
print "DEBUG DONE"