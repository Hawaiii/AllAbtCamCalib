import camera as cam
import exp_util as util
import vis
import matplotlib.pyplot as plt


import unittest
import numpy as np
import pdb

class TestCameraMethods(unittest.TestCase):

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
            d0[pt_id] = pt_loc
            d1[pt_id] = pt_loc + np.ones((1,3))
        img_pts = [d0, d1]
        return img_pts, board_pts


    # def test_calibrate_camera_format(self):
        # # set up img points: a list of dictionaries keyed by point ID, whose
        # # values are (x,y) tuples of pixel location
        # # set up board: a dictionary keyed by point ID, whose values are (x,y,0)
        # # tuples of 3D points
        # img_pts, board_pts = self.helper_gen_img_pts_straight()

        # img_size = (600, 800)

        # # check result format
        # esti_cam = cam.Camera.calibrate_camera(img_pts, board_pts, img_size)

        # self.assertEqual(esti_cam.intrinsics.intri_mat.shape, (3,3))
        # self.assertEqual(esti_cam.intrinsics.radial_dist.shape, (1,3))
        # self.assertEqual(esti_cam.intrinsics.tang_dist.shape, (1,3))

        # self.assertEqual(len(esti_cam.extrinsics), 2)
        # self.assertEqual(esti_cam.extrinsics[0].trans_vec.shape, (1,3))
        # self.assertEqual(esti_cam.extrinsics[0].rot_vec.shape, (1,3))
        # self.assertEqual(esti_cam.extrinsics[0].rot_mat.shape, (3,3))
        # self.assertEqual(esti_cam.extrinsics[1].trans_vec.shape, (1,3))
        # self.assertEqual(esti_cam.extrinsics[1].rot_vec.shape, (1,3))
        # self.assertEqual(esti_cam.extrinsics[1].rot_mat.shape, (3,3))

    # def test_ray_from_pixel(self):
    #     camera = cam.Camera.make_pinhole_camera()
    #     cam_loc = cam.Extrinsics.init_with_numbers(0.,0.,0.,0.,0.,0.)
    #     ray0 = camera.ray_from_pixel((0,0), cam_loc)
    #     ray1 = camera.ray_from_pixel((10,0), cam_loc)
    #     ray2 = camera.ray_from_pixel((0,10), cam_loc)
    #     ray3 = camera.ray_from_pixel((10,10), cam_loc)
    #     ray4 = camera.ray_from_pixel((camera.width()-1,0), cam_loc)
    #     ray5 = camera.ray_from_pixel((0, camera.height()-1), cam_loc)
    #     ray6 = camera.ray_from_pixel((camera.width()-1, camera.height()-1), cam_loc)
    #     vis.plot_camera_with_rays(cam_loc, [ray0, ray1, ray2, ray3, ray4, ray5, ray6], invert=False)

    # def test_ray_from_pixel_depth(self):
    #     camera = cam.Camera.make_pinhole_camera()
    #     cam_loc = cam.Extrinsics.init_with_numbers(0.,0.,0.,0.,0.,0.)
    #     ray0 = camera.ray_from_pixel((0,0), cam_loc)
    #     ray1 = camera.ray_from_pixel((camera.width()-1,0), cam_loc)
    #     ray2 = camera.ray_from_pixel((0, camera.height()-1), cam_loc)
    #     ray3 = camera.ray_from_pixel((camera.width()-1, camera.height()-1), cam_loc)
    #     rays = [ray0, ray1, ray2, ray3]

    #     pts_at_depth = {}
    #     for i in [0,1,2,4]:
    #         for r in rays:
    #             if i in pts_at_depth:
    #                 pts_at_depth[i].append(r[0]+r[1]*i)
    #             else:
    #                 pts_at_depth[i] = [r[0]+r[1]*i]
    #     vis.plot_camera_with_points(cam_loc, pts_at_depth)
    #     plt.show()

    def test_ray_from_pixel_moved(self):
        camera = cam.Camera.make_pinhole_camera()
        camera_loc = np.array([0, 2, 0.5]).reshape(3,1)
        camera_ori = np.array([np.pi/2, 0, 0]).reshape(3,1)
        cam_loc = util.Pose(camera_loc, camera_ori).extrinsics()

        ray0 = camera.ray_from_pixel((0,0), cam_loc)
        ray1 = camera.ray_from_pixel((camera.width()-1,0), cam_loc)
        ray2 = camera.ray_from_pixel((0, camera.height()-1), cam_loc)
        ray3 = camera.ray_from_pixel((camera.width()-1, camera.height()-1), cam_loc)

        ray4 = camera.ray_from_pixel((camera.width()/2,0), cam_loc)
        ray5 = camera.ray_from_pixel((0, camera.height()/2), cam_loc)
        ray6 = camera.ray_from_pixel((camera.width()-1, camera.height()/2), cam_loc)
        ray7 = camera.ray_from_pixel((camera.width()/2, camera.height()-1), cam_loc)

        rays = [ray0, ray1, ray2, ray3, ray4, ray5, ray6, ray7]
        vis.plot_camera_with_rays(cam_loc, rays)

        # pts_at_depth = {}
        # for i in [0,1]:
        #     for r in rays:
        #         if i in pts_at_depth:
        #             pts_at_depth[i].append(r[0]+r[1]*i)
        #         else:
        #             pts_at_depth[i] = [r[0]+r[1]*i]
        # vis.plot_camera_with_points(cam_loc, pts_at_depth)
        # plt.show()
    # if __name__ == '__main__':
    #     unittest.main()

  #     self.assertEqual('foo'.upper(), 'FOO')
  #     self.assertTrue('FOO'.isupper())
  #     self.assertFalse('Foo'.isupper())
  #     self.assertRaises(TypeError):
