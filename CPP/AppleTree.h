#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <sys/types.h>
#include <dirent.h>
#include <string>
#include <stdarg.h>
#include <cv.h>
#include <highgui.h>
#include <fstream>
#include <map>
#include <string.h>
#include <glog/logging.h>
#include <armadillo>

#define DEBUG 1

struct TreeNode{
	TreeNode * left_node; //0
	TreeNode * right_node; //1
	int min;
	int max;
};

struct Chessboard {
	TreeNode * lookup_table_x;
	TreeNode * lookup_table_y;
	double width; // physical width
	double height;	 //phyiscal height
	double rows; // # of rows
	double cols; // # of cols
	Chessboard(){}
	Chessboard(double r, double c, double x, double y):rows(r = 5),cols(c = 8),width(x = 0), height(y = 0){}
};

struct Intrinsic {
	//Intrinsics.. focus_X, focus_Y, pptx, ppty
	double focus_x, focus_y, pptx, ppty;
	Intrinsic(){}
	Intrinsic(double f_x, double f_y, double p_x, double p_y):focus_x(f_x), focus_y(f_y),
	pptx(p_x), ppty(p_y){}
};

struct Extrinsic {
	// all in CV rouguious form
	cv::Mat  Rot, Trans;
	Extrinsic(cv::Mat r, cv::Mat t): Rot(r), Trans(t){}
};

struct opt_ {

	std::string Lookup_table_dir_x, Lookup_table_dir_y, Image_list_dir, output_dir, pose_prefix, image_prefix, image_type;
	unsigned int N_shoots, N_poses, image_width, image_height, split_pos, x_encoding_len, y_encoding_len;
	double suppression_ratio, board_physical_width, board_physical_height;
	opt_(){}
	opt_(unsigned int N_shoots, unsigned int N_poses, unsigned int image_width, unsigned int image_height, double suppression_ratio,
		unsigned int split_pos,unsigned int x_encoding_len, unsigned int y_encoding_len, double board_physical_width, double board_physical_height;
		std::string a, std::string b, std::string c, std::string d, std::string e, std::string f, std::string g):
	 N_shoots(N_shoots ),
	 N_poses(N_poses ),
	 image_width(image_width),
	 image_height(image_height),
	 suppression_ratio(suppression_ratio),
	 board_physical_width(board_physical_width),
	 board_physical_height(board_physical_height),

	 split_pos(split_pos),
	 x_encoding_len(x_encoding_len),
	 y_encoding_len(y_encoding_len),
	 Lookup_table_dir_x(a ),
	 Lookup_table_dir_y(b ),
	 Image_list_dir(c ),
	 output_dir(d ),
	 image_prefix(e ),
	 pose_prefix(f ),
	 image_type(g )
		{}
};

class AppleJuice{
	private:

		Chessboard chessboard;

		std::vector<std::vector<cv::Mat>> ImageLists;
		std::vector<std::vector<cv::Mat>> BinaryImages;
		std::vector<std::vector<cv::Point2f>> FeaturePool;
		std::vector<std::vector<cv::Point3f>> PatternPtsPool;
		std::pair<cv::Point3f, cv::Point3f> SearchPoints(std::string xs, std::string ys);
		Intrinsic intrinsic;
		std::vector<arma::cube> ImageBlob;
		std::vector<arma::ucube> BinaryBlob;
		std::vector<arma::umat> Masks;

	public:
		opt_ options;
		//read lookup table from txt
		// 	>> Chessboard struct
		void ReadLookup_table(const opt_ options);
		//read image from DIR
		//	>> ImageLists
		void ReadImageLists(const opt_ options, unsigned int display = 0);
		//Biinarize all images
		//	>> BinaryImages
		void BinarizeAllImages();
		//Extract all control pts
		//	>> FeaturePools & PatternPtsPool
		void ExtractControlPts();
		//Get Init. Guess from standart OpenCV Camera Calibration
		//	>> Intrisics >> Extrinsic
		void InitCameraCalibration();
		// Full BundleAdjustment
		//  >> ???
		void AppleBundleAdjustment();

		void ComputeReprojectionError();

		void ExportResults();






};
