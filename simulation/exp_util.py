"""
Utility functions for dealing with 3D objects.
"""

import numpy as np

# # A Control Point in 3D space (in world frame), containing an id for matching 
# #the same points
# class ControlPoint
#   loc = np.zeros(1,3)
#   id_num = -1
#   def __init__(self, location, idnum):
#         self.loc = location
#         self.id_num = id_num

def rotate_and_translate(pts, r_mat, t_vec):
    """
    Returns the new location of point after rotation and translation
    """
    # @TODO
    pass

# 
# I
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

    pt_id = 0
    for x in xrange(board_height):
        for y in xrange(board_width):
            board[pt_id] = np.array([x * sqsize, y * sqsize, 0])
            pt_id += 1

    # @TODO location, orientation, add noise3d
    print "@TODO: haven't implemented location, orientation, and noise3d of board!"
    return board


"""
Utility functions for running experiments.
"""

def compute_estimation_diff(est_cam, true_cam, true_extrin):
    """
    Compute the difference between estimated result and true value
    """
    # @TODO
    pass
