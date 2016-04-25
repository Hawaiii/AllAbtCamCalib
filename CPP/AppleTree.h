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


struct TreeNode{
	TreeNode * left_node;
	TreeNode * right_node;
	int min;
	int max;
};

struct Chessboard {
	TreeNode * lookup_table;
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

struct opt {
	std::string Lookup_table_dir, Image_list_dir, output_dir, pose_prefix, image_prefix, image_type;
	unsigned int N_shoots, N_poses;
	opt(unsigned int N_shoots, unsigned int N_poses, std::string a, std::string b, std::string c, std::string d, std::string e, std::string f):
	 N_shoots(N_shoots ),
	 N_poses(N_poses ),
	 Lookup_table_dir(a ),
	 Image_list_dir(b ),
	 output_dir(c ),
	 image_prefix(d ),
	 pose_prefix(e ),
	 image_type(f )
		{}
};

class AppleJuice{
	private:

		Chessboard chessboard;

		std::vector<std::vector<cv::Mat>> ImageLists;
		std::vector<std::vector<cv::Mat>> BinaryImages;
		std::vector<std::vector<cv::Point2f>> FeaturePool;
		std::vector<std::vector<cv::Point3f>> PatternPtsPool;
		cv::Point3f SearchPoints(std::string s);
		Intrinsic intrinsic;

	public:
		static opt options;
		//read lookup table from txt
		// 	>> Chessboard struct
		void ReadLookup_table(std::string filename);
		//read image from DIR
		//	>> ImageLists
		void ReadImageLists(const opt options);
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
