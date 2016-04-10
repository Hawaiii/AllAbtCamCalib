import cv2
import numpy as np

def make_chessboard(screen_res, chess_dim, orig_name='chess.png', inv_name='chess_inv.png'):
	"""
	Returns a chessboard image of given size and dimension.

	Args:
		screen_res: (screen_width, screen_height)
		chess_dim: chessboard dimention, (chessboard_width, chessboard_height)
		orig_name: name to save original board with
		inv_name: name to save inverted board with
	"""
	board = 255*np.ones((screen_res[1], screen_res[0]),dtype=np.uint8)
	dim_pxl = min(screen_res[0]/chess_dim[0], screen_res[1]/chess_dim[1])
	for vblock in xrange(chess_dim[1]):
		for hblock in xrange(chess_dim[0]):
			if (vblock % 2) != (hblock % 2):
				# color the block black
				for i in xrange(vblock*dim_pxl, (vblock+1)*dim_pxl):
					for j in xrange(hblock*dim_pxl, (hblock+1)*dim_pxl):
						board[i][j] = 0
	inv_board = np.invert(board, dtype=np.uint8)
	cv2.imwrite(orig_name, board)
	cv2.imwrite(inv_name, inv_board)
	
	return board, inv_board


def flash_invert_chessboard(screen_res, chess_dim, fps=30):
	"""
	Flashs both original and inverted chessboard on screen.

	Args:
		screen_res: (screen_width, screen_height)
		chess_dim: chessboard dimention, (chessboard_width, chessboard_height)
		fps: flashing images at this frame rate, default to 30

	"""
	# Make original and inverted chessboard
	board, inv_b = make_chessboard((1366, 768), (8,5),'chess.png','chess_inv.png')
	
	# Flash between two boards indefintely
	cv2.namedWindow('calibration', 0)
	# OpenCV has a fullscreen bug
	#cv2.setWindowProperty('calibration', cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
	flip = 0
	while True:
		if flip:
			cv2.imshow('calibration',board)
		else:
			cv2.imshow('calibration',inv_b)
		flip = not flip
		cv2.waitKey(1000/fps)

flash_invert_chessboard((1366, 768), (5,8))
