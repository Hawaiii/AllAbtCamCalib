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
struct TreeNode{
	TreeNode * left_node;
	TreeNode * right_node;
	int val; //swtich to * Data if you want
};

struct Chessboard {
	TreeNode * lookup_table;
	double height;	 //phyiscal height
	double rows; // # of rows
	double cols; // # of cols
        double width;

	Chessboard(double r, double c, double x, double y){
		rows = r;
		cols = c;
	        width = x;
		height = y;
	return;	
	}
};

class AppleJuice{
	private:
		Chessboard chessboard;

	public:
		void ReadLookup_table();

			
		


};
