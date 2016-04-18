import imu
import camera as cam
import exp_util as util
import vis
import board as bd

import numpy as np
import cv2
import math
import matplotlib.pyplot as plt

imu_motion = imu.read_motion('results/test_.csv', 100)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# ax = vis.plot_poses(imu_motion, fax=ax, invert=True, clr='g')
cm = plt.get_cmap('plasma')
for i, p in enumerate(imu_motion):
	ax = p.plot(ax, clr=cm(1.*i/(len(imu_motion)-1)), length=0.05)
x = [p.loc[0,0] for p in imu_motion]
y = [p.loc[1,0] for p in imu_motion]
z = [p.loc[2,0] for p in imu_motion]
ax.plot(x,y,z)


board_dim = (5, 8)
board_orient = np.array([[-8.7036998831043544e-01, -9.0912931784559947e-02, 4.8393276628352877e-01],\
 [-1.1424780991646996e-03, -9.8243204946945251e-01, -1.8661715601477852e-01],\
 [4.9239697215994882e-01, -1.6297885448605659e-01, 8.5497550537902922e-01]])
assert(abs(np.linalg.det(board_orient) - 1) < 0.01 )
board_trans_vec = np.array([7.6537016714681638e-01, -3.0420191600265117e-01, -4.5732500607382032e-01])
board_ext = cam.Extrinsics.init_with_rotation_matrix(board_trans_vec, board_orient)
board_pose = board_ext.pose()
board = bd.Board.gen_calib_board(board_dim, 0.068, board_pose.loc, board_pose.ori, 0)
ax = board.plot(ax)

# ax.set_aspect('equal')
ax.set_xlabel('x')
ax.set_ylabel('y')
# ax.set_ylim(-0.9, -0.2)
ax.set_zlabel('z')

plt.show()