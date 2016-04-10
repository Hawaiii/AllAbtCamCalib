"""
Utility functions for dealing with 3D objects.
"""
import cv2
import numpy as np
import math
import random

# # A Control Point in 3D space (in world frame), containing an id for matching
# #the same points
# class ControlPoint
#   loc = np.zeros(1,3)
#   id_num = -1
#   def __init__(self, location, idnum):
#         self.loc = location
#         self.id_num = id_num

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

def unit_vector(vector):
    """
    Returns the unit vector of the vector.
    """
    norm = np.linalg.norm(vector)
    if norm == 0:
        return vector
    return vector / norm

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

def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.

    From: http://stackoverflow.com/questions/6802577/python-rotation-of-3d-
    vector
    """
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2.0)
    b, c, d = -axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])

def rotate_and_translate(pts, r_mat, t_vec):
    """
    Returns the new location of point after rotation and translation
    """
    # @TODO
    pass

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

def gen_calib_board(board_height, board_width, sqsize, \
                    location, orientation, noise3d):
    """
    Generate a calibration board placed at give location with given direction,
    with given number of points on a plane.
    Args:
        board_height: positive integer, number of control points in vertical direction
        board_width: positive integer, number of control points in horizontal direction
        sqsize: positive number, size of each square, in millimeters
        location: 1x3 numpy array, 3D location of the top-left corner of board
        orientation: 1x3 numpy array, 3D orientation of the board, (rx,ry,rz)
        noise3d: positive number, standard deviation of Gaussian noise added to board

    Returns:
        A dictionary keyed by point id, whose values are 3D points (1x3 numpy arrays)
    """
    board = {}

    # Make board whose top-left point is at (0,0,0) and lies on x-y plane
    pt_id = 0
    for x in xrange(board_height):
        for y in xrange(board_width):
            board[pt_id] = np.array([x * sqsize, y * sqsize, 0], np.float32)
            pt_id += 1

    # Rotate board to given orientation and move to given location
    # z_vec = np.array([0,0,1])
    # rot_axis = np.cross(z_vec, orientation)
    # r_angle = np.arccos( np.dot(z_vec,orientation)/np.linalg.norm(orientation) ) #signed?
    # # print "rot_axis", rot_axis, "r_angle", r_angle
    # if not np.isnan(r_angle):
    #     rot_mat = rotation_matrix(rot_axis, r_angle)
    #     # print "rot_mat:", rot_mat
    #     for pt in board.keys():
    #         board[pt] = rot_mat.dot(board[pt]) + location
    rot_mat, _ = cv2.Rodrigues(orientation)
    for pt in board.keys():
        board[pt] = np.dot(rot_mat, board[pt]) + location

    # Add noise3d
    if noise3d > 0:
        noises = np.random.normal(0, noise3d, (board_height*board_width,3))
        for ipt in xrange(len(board)):
            board[board.keys()[ipt]] += noises[ipt, :]

    return board

def move_board(board, location, orientation):
    """
    Move a generated board to a given location.
    @TODO: rotate a board to a given orientation

    Args:
        board: a dictionary keyed by point id, whose values are 3D points (1x3
               numpy arrays); this is a board generated by gen_calib_board, and 
               the top-left corner point is keyed by 0
        location: 1x3 numpy array, desired 3D location of the top-left corner
                  of board
        orientation: 1x3 numpy array, desired 3D orientation of the board,
                     (rx,ry,rz)

    Returns:
        A dictionary keyed by point id, whose values are 3D points
    """
    if len(board) == 0:
        return {}
    
    rot_mat, _ = cv2.Rodrigues(orientation)
    newboard = {}
    for pt in board.keys():
        newboard[pt] = np.dot(rot_mat, board[pt])
    offset = location - newboard[0]
    for pt in newboard.keys():
        newboard[pt] = newboard[pt] + offset
    return newboard

def board_dict2array(board, board_dim):
    """
    Converts the dictionary representation of board into X,Y,Z array format

    Args:
        board: a dictionary keyed by point id, whose values are 3D points, keyed
               in the following order: starting from top-left point, move right
               alone each row, and down for all rows, ending at bottom-right
        board_dim: (board_height, board_width)

    Returns:
        X,Y,Z: each a 2D numpy array, specifying the location in the
               corresponding dimension of each point
    """
    X = np.empty(board_dim)
    Y = np.empty(board_dim)
    Z = np.empty(board_dim)
    for pt in board.keys():
        x = pt / board_dim[1]
        y = pt % board_dim[1]
        X[x,y] = board[pt][0]
        Y[x,y] = board[pt][1]
        Z[x,y] = board[pt][2]
    return X, Y, Z


"""
Utility functions for running experiments.
"""
def compute_estimation_diff(est_cam, true_cam):
    """
    Compute the difference between estimated result and true value
    """
    # compute difference between intrinsics

    # compute difference between extrinsics

    # plot distribution?

    pass
