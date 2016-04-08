import camera as cam
import numpy as np

def spiral_motion():
	"""
	Generating location with sin in each axis.
	TODO: generate orientation
	TODO: generate time stamp

	Returns:
		a list of (TODO: time-stamped) extrinsics
	"""
	extrins = []
	theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
	r = 5
	x = r * np.sin(theta)
	y = r * np.sin(theta*1.5)
	z = r * np.sin(theta*1.2)

	for i in range(len(theta)):
		trans_vec = np.asarray([x[i], y[i], z[i]])
		print 'trans_vec', trans_vec
		ext = cam.Extrinsics(trans_vec, None, None)
		extrins.append(ext)

	return extrins