"""
Utility functions for dealing with 3D objects.
"""
import cv2
import numpy as np
import math
import random

class Pose:
    loc = None # location, 3x1 numpy array
    ori = None # orientation, 3x3 rotation matrix
    lin_vel = None # linear velocity, 3x1 numpy array
    ang_vel = None # angular velocity, 3x1 numpy array
    lin_acc = None # linear acceleration, 3x1 numpy array
    ang_acc = None # angular acceleration, 3x1 numpy array
    time = None # time stampe in ns

    def __init__(self, loc, ori, time=None, lvel=None, avel=None, lacc=None, aacc=None):
        # assert(loc.size == 3)
        self.loc = 1.0 * loc.reshape(3,1)
        
        assert(ori.size == 3 or ori.size == 9)
        if ori.size == 3:
            self.ori = cv2.Rodrigues(ori)[0]
        else:
            self.ori = 1.0 * ori

        self.time = time

        if lvel is not None:
            # assert(lvel.size == 3)
            self.lin_vel = 1.0 * lvel.reshape(3,1)

        if avel is not None:
            # assert(avel.size == 3)
            self.ang_vel = 1.0 * avel.reshape(3,1)

        if lacc is not None:
            # assert(lacc.size == 3)
            self.lin_acc = 1.0 * lacc.reshape(3,1)

        if aacc is not None:
            # assert(aacc.size == 3)
            self.ang_acc = 1.0 * aacc.reshape(3,1)

    def __repr__(self):
        selfstr = 'Pose at location {0} orientation {1} time {2} '.format(\
            self.loc.flatten(), self.ori_vec().flatten(), self.time)
        if self.lin_vel is not None:
            selfstr += 'linear velocity {0}'.format(self.lin_vel.flatten())
        if self.ang_vel is not None:
            selfstr += 'angular velocity {0}'.format(self.ang_vel.flatten())
        if self.lin_acc is not None:
            selfstr += 'linear acceleration {0}'.format(self.lin_acc.flatten())
        if self.ang_acc is not None:
            selfstr += 'angular acceleration {0}'.format(self.ang_acc.flatten())
        return selfstr

    def ori_vec(self):
        """
        Returns the Euler angles of orientation.
        """
        return cv2.Rodrigues(self.ori)[0]

    def loc_homo(self):
        return np.concatenate((self.loc, np.ones(1,1)), axis=0)

    def extrinsics(self):
        """
        Return the corresponding Extrinsics.
        """
        return cam.Extrinsics.init_with_rotation_matrix(-self.ori.dot(loc), \
            self.ori, self.time)

    def transformation(self):
        """
        Returns the 4x4 transformation from pose coordinate to world coordinate.
        """
        return self.extrinsics.get_homo_trans_matrix()

    @staticmethod
    def motion_regress_vel_acc(motion, window):
        """
        TODO: change the window from number of samples to a time-window
        Args:
            motion: a list of time-stamped poses
            window: number of samples to use for each window, an odd number 
                    greater than 1
        Returns:
            motion: with linear velocity and acceleration fields written
        """
        n = len(motion)
        for i in xrange(n):
            if motion[i].lin_vel is not None and motion[i].lin_acc is not None:
                print 'skipping pose',i,'for computing linear velocity and acceleration.'
                continue

            if i - window/2 < 0 or i + window/2 > n-1:
                continue

            s_x = np.zeros(3,1)
            s_tx = np.zeros(3,1)
            s_t2x = np.zeros(3,1)
            s_t = 0
            s_t2 = 0
            s_t3 = 0
            s_t4 = 0

            for j in xrange(i-window/2, i+window/2+1):
                t_j = motion[j].time - motion[i].time
                s_x += motion[j].loc
                s_tx += motion[j].loc * t_j
                s_t2x += motion[j].loc * t_j * t_j
                s_t += t_j
                s_t2 += t_j * t_j
                s_t3 += t_j ** 3
                s_t4 += t_j ** 4

            A = window * (s_t3*s_t3 - s_t2*s_t4) + \
                    s_t * (s_t*s_t4 - s_t2 * s_t3) + \
                    s_t2 * (s_t2*s_t2 - s_t*s_t3)

            motion[i].lin_vel = 1/A * (s_x * (s_t*s_t4 - s_t2*s_t3) + \
                                        s_tx * (s_t2*s_t2 - n*s_t4) + \
                                        s_t2x * (n*s_t3 - s_t*s_t2))
            motion[i].lin_acc = 2/A * (s_x * (s_t2*s_t2 - s_t*s_t3) + \
                                        s_tx * (n*s_t3 - s_t*s_t2) + \
                                        s_t2x * (s_t*s_t - n*s_t2))

        return motion


    def plot(self, fax=None, clr=None, length=1.0):
        """
        Plots a arrow of z-axis indicating the pose
        """
        z_vec = self.ori.dot(np.array([0,0,1]).reshape(3,1))
        if fax is None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
        else:
            ax = fax
        if clr is None:
            ax.quiver(self.loc[0,0], self.loc[1,0], self.loc[2,0], \
                z_vec[0,0], z_vec[1,0], z_vec[2,0], pivot='tail', \
                length=length)
        else:
            ax.quiver(self.loc[0,0], self.loc[1,0], self.loc[2,0], \
                z_vec[0,0], z_vec[1,0], z_vec[2,0], pivot='tail', \
                color=clr, length=length)

        if fax is None:
            plt.show()
        return ax

    @staticmethod
    def plot_pose_seq(motion, fax=None, length=1.0, cmap='plasma'):
        """
        Plots a list of poses in gradient color.
        Args:
            motion: list of Pose
        """
        #TODO

def unit_vector(vector):
    """
    Returns the unit vector of the vector.
    """
    norm = np.linalg.norm(vector)
    if norm == 0:
        return vector
    return vector / norm

def random_rotation():
    """
    Returns a random roll-pitch-yaw rotation

    Returns:
        1x3 numpy array, representing a rotation in Roll-Pitch-Yaw Euler angles
    """
    theta = 2*math.pi*random.random() - math.pi
    phi = math.acos(1 - 2*random.random()) + math.pi/2.0
    if random.random() < 0.5:
        if phi < math.pi:
            phi = phi + math.pi
        else:
            phi = phi - math.pi
    eta = 2 * math.pi * random.random() - math.pi
    return np.asarray([theta, phi, eta])

def quaternion2mat(x,y,z,w):
    """
    https://github.com/bistromath/gr-air-modes/blob/master/python/Quaternion.py
    """
    xx2 = 2 * x * x
    yy2 = 2 * y * y
    zz2 = 2 * z * z
    xy2 = 2 * x * y
    wz2 = 2 * w * z
    zx2 = 2 * z * x
    wy2 = 2 * w * y
    yz2 = 2 * y * z
    wx2 = 2 * w * x

    rmat = np.empty((3, 3), float)
    rmat[0,0] = 1. - yy2 - zz2
    rmat[0,1] = xy2 - wz2
    rmat[0,2] = zx2 + wy2
    rmat[1,0] = xy2 + wz2
    rmat[1,1] = 1. - xx2 - zz2
    rmat[1,2] = yz2 - wx2
    rmat[2,0] = zx2 - wy2
    rmat[2,1] = yz2 + wx2
    rmat[2,2] = 1. - xx2 - yy2

    return rmat
    
# def gen_calib_board(board_width, board_height, sqsize, \
#                     location, orientation, noise3d):
#     """
#     Generate a calibration board placed at give location with given direction,
#     with given number of points on a plane.
#     Args:
#         board_width: positive integer, number of control points in horizontal direction
#         board_height: positive integer, number of control points in vertical direction
#         sqsize: positive number, size of each square, in millimeters
#         location: 3x1 numpy array, 3D location of the top-left corner of board
#         orientation: 3x1 numpy array, 3D orientation of the board, (rx,ry,rz)
#         noise3d: positive number, standard deviation of Gaussian noise added to board

#     Returns:
#         A dictionary keyed by point id, whose values are 3D points (1x3 numpy arrays)
#     """
#     if location.shape != (3,1) or orientation.shape != (3,1):
#         print 'location shape',location.shape,' orientation shape',orientation.shape,'incorrect!'
#         return None
#     board = {}

#     # Make board whose top-left point is at (0,0,0) and lies on x-y plane
#     pt_id = 0
#     for x in xrange(board_height):
#         for y in xrange(board_width):
#             board[pt_id] = np.array([[x * sqsize, y * sqsize, 0]], np.float32).T
#             pt_id += 1

#     # Rotate board to given orientation and move to given location
#     rot_mat, _ = cv2.Rodrigues(orientation)
#     for pt in board.keys():
#         board[pt] = np.dot(rot_mat, board[pt]) + location

#     # Add noise3d
#     if noise3d > 0:
#         noises = np.random.normal(0, noise3d, (board_height*board_width,3))
#         for ipt in xrange(len(board)):
#             board[board.keys()[ipt]] += noises[ipt, :]
# 
#     return board
# 
# def move_board(board, location, orientation):
#     """
#     Move a generated board to a given location and orientation.
#     @TODO: Assumes board is given at 0 rotation.

#     Args:
#         board: a dictionary keyed by point id, whose values are 3D points (1x3
#                numpy arrays); this is a board generated by gen_calib_board, and
#                the top-left corner point is keyed by 0
#         location: 3x1 numpy array, desired 3D location of the top-left corner
#                   of board
#         orientation: 1x3 numpy array, desired 3D orientation of the board,
#                      (rx,ry,rz)

#     Returns:
#         A dictionary keyed by point id, whose values are 3D points
#     """
#     if len(board) == 0:
#         return {}

#     rot_mat, _ = cv2.Rodrigues(orientation)
#     newboard = {}
#     for pt in board.keys():
#         newboard[pt] = rot_mat.dot(board[pt])
#     offset = location - newboard[0]
#     for pt in newboard.keys():
#         newboard[pt] = newboard[pt] + offset
#     return newboard
# 
# def board_dict2array(board, board_dim):
#     """
#     Converts the dictionary representation of board into X,Y,Z array format

#     Args:
#         board: a dictionary keyed by point id, whose values are 3D points, keyed
#                in the following order: starting from top-left point, move right
#                alone each row, and down for all rows, ending at bottom-right
#         board_dim: (board_width, board_height)

#     Returns:
#         X,Y,Z: each a 2D numpy array, specifying the location in the
#                corresponding dimension of each point
#     """
#     X = np.empty((board_dim[1], board_dim[0]))
#     Y = np.empty((board_dim[1], board_dim[0]))
#     Z = np.empty((board_dim[1], board_dim[0]))
#     for pt in board.keys():
#         x = pt / board_dim[0]
#         y = pt % board_dim[0]
#         X[x,y] = board[pt][0,0]
#         Y[x,y] = board[pt][1,0]
#         Z[x,y] = board[pt][2,0]
#     return X, Y, Z


"""
Utility functions for running experiments.
"""

# def euler2mat(e_angles):
#     """
#     Returns the rotation matrix for given Euler angle (rx, ry, rz)
#     Args:
#         e_angles: 1x3 numpy array

#     Returns:
#         rot_mat: rotation matrix of given euler angles
#     """
#     rot_mat = np.eye(3)
#     cosz = math.cos(e_angles[2])
#     sinz = math.sin(e_angles[2])
#     zmat = np.array([[cosz, -sinz, 0], [sinz, cosz, 0],[0, 0, 1]])

#     cosy = math.cos(e_angles[1])
#     siny = math.sin(e_angles[1])
#     ymat = np.array([[cosy, 0, siny], [0, 1, 0], [-siny, 0, cosy]])

#     cosx = math.cos(e_angles[0])
#     sinx = math.sin(e_angles[0])
#     xmat = np.array([[1, 0, 0], [0, cosx, -sinx], [0, sinx, cosx]])

#     return np.dot(xmat, np.dot(ymat, zmat))

# def mat2euler(M, cy_thresh=None):
#     ''' Discover Euler angle vector from 3x3 matrix

#     Uses the conventions above.

#     Parameters
#     ----------
#     M : array-like, shape (3,3)
#     cy_thresh : None or scalar, optional
#        threshold below which to give up on straightforward arctan for
#        estimating x rotation.  If None (default), estimate from
#        precision of input.

#     Returns
#     -------
#     z : scalar
#     y : scalar
#     x : scalar
#        Rotations in radians around z, y, x axes, respectively

#     Notes
#     -----
#     If there was no numerical error, the routine could be derived using
#     Sympy expression for z then y then x rotation matrix, which is::

#       [                       cos(y)*cos(z),                       -cos(y)*sin(z),         sin(y)],
#       [cos(x)*sin(z) + cos(z)*sin(x)*sin(y), cos(x)*cos(z) - sin(x)*sin(y)*sin(z), -cos(y)*sin(x)],
#       [sin(x)*sin(z) - cos(x)*cos(z)*sin(y), cos(z)*sin(x) + cos(x)*sin(y)*sin(z),  cos(x)*cos(y)]

#     with the obvious derivations for z, y, and x

#        z = atan2(-r12, r11)
#        y = asin(r13)
#        x = atan2(-r23, r33)

#     Problems arise when cos(y) is close to zero, because both of::

#        z = atan2(cos(y)*sin(z), cos(y)*cos(z))
#        x = atan2(cos(y)*sin(x), cos(x)*cos(y))

#     will be close to atan2(0, 0), and highly unstable.

#     The ``cy`` fix for numerical instability below is from: *Graphics
#     Gems IV*, Paul Heckbert (editor), Academic Press, 1994, ISBN:
#     0123361559.  Specifically it comes from EulerAngles.c by Ken
#     Shoemake, and deals with the case where cos(y) is close to zero:

#     See: http://www.graphicsgems.org/

#     The code appears to be licensed (from the website) as "can be used
#     without restrictions".
#     '''
#     M = np.asarray(M)
#     if cy_thresh is None:
#         try:
#             cy_thresh = np.finfo(M.dtype).eps * 4
#         except ValueError:
#             cy_thresh = _FLOAT_EPS_4
#     r11, r12, r13, r21, r22, r23, r31, r32, r33 = M.flat
#     # cy: sqrt((cos(y)*cos(z))**2 + (cos(x)*cos(y))**2)
#     cy = math.sqrt(r33*r33 + r23*r23)
#     if cy > cy_thresh: # cos(y) not close to zero, standard form
#         z = math.atan2(-r12,  r11) # atan2(cos(y)*sin(z), cos(y)*cos(z))
#         y = math.atan2(r13,  cy) # atan2(sin(y), cy)
#         x = math.atan2(-r23, r33) # atan2(cos(y)*sin(x), cos(x)*cos(y))
#     else: # cos(y) (close to) zero, so x -> 0.0 (see above)
#         # so r21 -> sin(z), r22 -> cos(z) and
#         z = math.atan2(r21,  r22)
#         y = math.atan2(r13,  cy) # atan2(sin(y), cy)
#         x = 0.0
#     return z, y, x

# def angle_between(v1, v2):
#     """
#     http://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional
#     -vectors-in-python
#     Returns the angle in radians between vectors 'v1' and 'v2'::

#             >>> angle_between((1, 0, 0), (0, 1, 0))
#             1.5707963267948966
#             >>> angle_between((1, 0, 0), (1, 0, 0))
#             0.0
#             >>> angle_between((1, 0, 0), (-1, 0, 0))
#             3.141592653589793
#     """
#     v1_u = unit_vector(v1)
#     v2_u = unit_vector(v2)
#     angle = np.arccos(np.dot(v1_u, v2_u))
#     if np.isnan(angle):
#         if (v1_u == v2_u).all():
#             return 0.0
#         else:
#             return np.pi
#     return angle

# def rotation_matrix(axis, theta):
#     """
#     Return the rotation matrix associated with counterclockwise rotation about
#     the given axis by theta radians.

#     From: http://stackoverflow.com/questions/6802577/python-rotation-of-3d-
#     vector
#     """
#     axis = axis/math.sqrt(np.dot(axis, axis))
#     a = math.cos(theta/2.0)
#     b, c, d = -axis*math.sin(theta/2.0)
#     aa, bb, cc, dd = a*a, b*b, c*c, d*d
#     bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
#     return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
#                      [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
#                      [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])

# def rotate_and_translate(pts, r_mat, t_vec):
#     """
#     Returns the new location of point after rotation and translation
#     """
#     # @TODO
#     pass
