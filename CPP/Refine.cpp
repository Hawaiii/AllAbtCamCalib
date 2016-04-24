#include "VisualPath.h"

using namespace std;
using namespace cv;

// Triangulates points repeatedly seen by multiple cameras, return their 3D location
// Input:
//  pts: pts[nview][npoint]: 2d observation of npoint-th point in nview-th view
//  K: K[nview]: intrinsic matrix of camera,  3x3
//  r: r[nview]: rotation of (1+nview)-th camera view, 3x3
//  t: t[nview]: translation of (1+nview)-th camera view, 3x1
void VisualPath::multi_view_triangulate(){

	vector<vector<Point2f>> pts;
	std::vector<Mat> K;
	for (size_t i = 0; i < numOfPose; i++) {
		pts.push_back(observedControlPts[i][0]);
		K.push_back(Camera_Matrix);
	}
	vector<Mat> r = R_list;
	vector<Mat> t = T_list;

	std::cout << "/* message */" <<pts.size() <<" "<< r.size()<<" " << t.size()<< std::endl;

	assert(pts.size() == r.size() && r.size() == t.size());
	if (pts.size() == 0) return ;

	const unsigned int nview = pts.size();
	const unsigned int npoint = pts[0].size();

	vector<Mat> pmat;

	for (unsigned int iv = 0; iv < nview; ++iv){
		Mat P = calculatePMatrix(K[iv], r[iv], t[iv]);
		pmat.push_back(P);
	}
	for (unsigned int ip = 0; ip < npoint; ++ip){
		Mat A;// =  Mat(2*nview, 4, CV_64FC1);
		Mat w, u, vt;
		for (unsigned int iv = 0; iv < nview; ++iv){
			double rowx[4] = {pmat[iv].at<double>(2,0)*pts[iv][ip].x - pmat[iv].at<double>(0,0),
				pmat[iv].at<double>(2,1)*pts[iv][ip].x - pmat[iv].at<double>(0,1),
				pmat[iv].at<double>(2,2)*pts[iv][ip].x - pmat[iv].at<double>(0,2),
				pmat[iv].at<double>(2,3)*pts[iv][ip].x - pmat[iv].at<double>(0,3)};
			double rowy[4] = {pmat[iv].at<double>(2,0)*pts[iv][ip].y - pmat[iv].at<double>(1,0),
				pmat[iv].at<double>(2,1)*pts[iv][ip].y - pmat[iv].at<double>(1,1),
				pmat[iv].at<double>(2,2)*pts[iv][ip].y - pmat[iv].at<double>(1,2),
				pmat[iv].at<double>(2,3)*pts[iv][ip].y - pmat[iv].at<double>(1,3)};
			A.push_back(Mat(1,4,CV_64FC1,rowx));
			A.push_back(Mat(1,4,CV_64FC1,rowy));
		}
		SVD::compute(A, w, u, vt);
		Mat ans = vt.row(vt.rows-1);
		Point3f a = Point3f(ans.at<double>(0)/ans.at<double>(3),ans.at<double>(1)/ans.at<double>(3),ans.at<double>(2)/ans.at<double>(3));
		p3d.push_back(a);
	}
	std::cout << "Triangulates done!" << std::endl;
	return;
}

// Returns P = K[R|t]
Mat VisualPath::calculatePMatrix(Mat& K, Mat& r, Mat& t){
	Mat P;

	hconcat(r,t, P);
	P = K*P;

	return P;
}

void VisualPath::readRobotArmRT(string filename){
	// Open file
	ifstream infile(filename, ifstream::in);
    if (!infile.is_open())
        throw std::runtime_error("readRobotArmRT: unable to open script: " + filename);

    // Clear R_robot_list and T_robot_list
    R_robot_list.clear();
    T_robot_list.clear();

	// Read line by line, and push to R_robot_list and T_robot_list
	string line;
	while (getline(infile, line)){
		if (line.empty() || line.find_first_not_of(' ') == std::string::npos ||
            line[0] == '#')
            continue;

        std::vector<std::string> sub_strings;
    	std::string sub;
    	std::stringstream ss(line);
    	while (std::getline(ss, sub, ':')) {
        	sub_strings.push_back(sub);
    	}

    	if(sub_strings.size() != 8)
    		throw std::runtime_error("invalid argument: " + sub_strings[0]);

        if (sub_strings[0] != "c" || sub_strings[1] != "a")
            throw std::runtime_error("unknown cartesian specification: " + sub_strings[0] + ":" +sub_strings[1]);

		Mat t = (Mat_<double>(3,1) << stod(sub_strings[2]), stod(sub_strings[3]), stod(sub_strings[4]) );
		// std::cerr<<"r:\n" << stod(sub_strings[5]) << " " << stod(sub_strings[6]) << " " << stod(sub_strings[7])<< std::endl;

		Mat rvec = (Mat_<double>(3,1) << stod(sub_strings[5]), stod(sub_strings[6]), stod(sub_strings[7]) );
		Mat r(3,3,CV_64FC1);
		Rodrigues(rvec, r);
		R_robot_list.push_back(r);
		T_robot_list.push_back(t);
	}

	infile.close();
	std::cout << "read log path done!" << std::endl;
}
