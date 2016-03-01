import unittest
import camera as cam
import exp_util as util
import numpy as np

class TestCameraMethods(unittest.TestCase):

    
    # def test_works(self):
    #     cam = camera.Camera()
    #     self.assertEqual(cam.testTODO,0)

    def helper_gen_img_pts_straight(self):
        """
        Returns a list of dictionaries of img_pts, keyed by point id and values are 2d 
        pixel location
        """
        board_height = 5
        board_width = 7
        board_sqsize = 23
        board_location = [0,0,0]
        board_orientation = [0,0,0]
        noise3d = 0
        board_pts = util.gen_calib_board(board_height, board_width, board_sqsize, \
            board_location, board_orientation, noise3d)

        d0 = {} #board 0 is just the x,y location of the board
        d1 = {} #board 1 shifts 1 pixel in both x and y direction
        for pt_id, pt_loc in board_pts.iteritems():
            d0[pt_id] = pt_loc[0:1]
            d1[pt_id] = pt_loc[0:1] + np.ones((1,2))
        img_pts = [d0, d1]
        return img_pts, board_pts


    def test_calibrate_camera_format(self):
        # set up img points: a list of dictionaries keyed by point ID, whose 
        # values are (x,y) tuples of pixel location
        # set up board: a dictionary keyed by point ID, whose values are (x,y,0)
        # tuples of 3D points
        img_pts, board_pts = self.helper_gen_img_pts_straight()

        img_size = (600, 800)

        # check result format
        esti_cam = cam.Camera.calibrate_camera(img_pts, board_pts, img_size)

        self.assertEqual(esti_cam.intrinsics.intri_mat.shape, (3,3))
        self.assertEqual(esti_cam.intrinsics.radial_dist.shape, (1,3))
        self.assertEqual(esti_cam.intrinsics.tang_dist.shape, (1,3))

        self.assertEqual(len(esti_cam.extrinsics), 2)
        self.assertEqual(esti_cam.extrinsics[0].trans_vec.shape, (1,3))
        self.assertEqual(esti_cam.extrinsics[0].rot_vec.shape, (1,3))
        self.assertEqual(esti_cam.extrinsics[0].rot_mat.shape, (3,3))
        self.assertEqual(esti_cam.extrinsics[1].trans_vec.shape, (1,3))
        self.assertEqual(esti_cam.extrinsics[1].rot_vec.shape, (1,3))
        self.assertEqual(esti_cam.extrinsics[1].rot_mat.shape, (3,3))

    
    # if __name__ == '__main__':
    #     unittest.main() 

  #     self.assertEqual('foo'.upper(), 'FOO')
  #     self.assertTrue('FOO'.isupper())
  #     self.assertFalse('Foo'.isupper())
  #     self.assertRaises(TypeError):
