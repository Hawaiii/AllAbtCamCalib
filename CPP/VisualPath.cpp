#include "VisualPath.h"

using namespace cv;

std::vector<Mat> VisualPath::readPositionImg(int pos, std::string revpath){
	std::vector<Mat> imgs;

	if (pos < 0 || pos >= numOfPose){
		std::cout << "pos number " << pos << "is out of range [0, 24]" << std::endl;
		return imgs;
	}
	const std::string partpath = "/capture_";
	const std::string endpath = ".png";
	// const std::string endpath = ".jpg";
	// 1 4 7 10 13 16 19 22 25 28
	int start = 1;
	for (int i = start; i < numOfRuns+start; ++i){
		std::string filename = revpath + partpath + std::to_string(pos);
		filename = filename + endpath;
		// if(i > 1) filename = filename + "_" + std::to_string(i-1) + endpath;
		// else filename = filename + endpath;
		//@Final test
		filename = std::to_string(pos*numOfRuns+i-1) + ".png";
		std::cout << "reading file " <<revpath + filename << std::endl;
		Mat image = imread(revpath  +filename, CV_LOAD_IMAGE_GRAYSCALE);

    if( !image.data )
          throw std::runtime_error("cannot load image !!!!!!!!!");
    // undistortImage(image);
    // FishEyeUndistortImage(image);
		// /**/
		// imwrite("./undistort"+partpath+"_" +std::to_string(pos) +"_"+ std::to_string(i-1)+"_undistorted"+endpath,image);
		imgs.push_back(image);
	}
	return imgs;
}

void VisualPath::readCalibrationResult(std::string file){
  FileStorage fs(file, FileStorage::READ); // Read the settings
  if (!fs.isOpened())
  {
   throw std::runtime_error("Could not open the configuration file: \""+file);
   return ;
  }
  // To-DO :
  // read data using XML;
  // fs["nrOfFrames"] >> numOfPose;
  // fs["board_Width"] >> chessboardHight;
  // fs["board_Height"] >> chessboardWidth;
  // fs["square_Size"] >> chessboardSize;
  fs["Camera_Matrix"] >> Camera_Matrix;
  fs["Distortion_Coefficients"] >> Distortion_Coefficients;
  fs.release();
  std::cout<<"Intrinsics:\n" << getCamera_Matrix() << std::endl;
  std::cout<<"\nistortion_Coefficients:\n" << getDistortion_Coefficients() << std::endl;

}

void VisualPath::createChessboard(){
		if(useChessboard){
			      for (size_t b = 0; b < numOfRuns; b++) {
			        std::vector<Point3f> corners;
							// pre run
			        for( int i = 0; i < chessboardWidth; i++ )
			        {
			           for( int j = 0; j < chessboardHight; j++ )
			           {
			             corners.push_back(Point3f(float(j*chessboardSize),
			                                       float(i*chessboardSize), 0));
			           }
			        }
			        actualControlPts.push_back(corners);
			      }
		}else{
					for (size_t b = 0; b < numOfRuns; b++) {
		        std::vector<Point3f> corners;
						//per run
						for (int i = 0; i < chessboardWidth; i++) {
								 for (int j = 0; j < chessboardHight; j++){
										 corners.push_back(cv::Point3f(float((2 * j + i % 2)*chessboardSize),
																 float(i*chessboardSize), 0));
								 }
						 }
						 actualControlPts.push_back(corners);
					 }
		 }
      std::cout << "createChessboard done!" << std::endl;
}

void VisualPath::extractControlPoints(){

    SimpleBlobDetector::Params params;

        // Range of values at which to perform thresholding
        params.minThreshold = 10.0;
        params.maxThreshold = 400.0;

        // Filter by the area of the blob
        params.minArea = 40.0;
        params.maxArea = 10e6;

        // Filter by the color of the blob, we are looking explicity for dark blobs
         params.filterByColor = 1;
         params.blobColor = 0;

        // Filter by the convexity of the blob
        // Useful, but EXPENSIVE
        params.filterByConvexity = true;
        params.minConvexity = 0.80;
        params.maxConvexity = 1.0;

        // DON'T filter by inertia or circulartiy, as these will hurt us when finding targets
        // which are significantly out of plane (I think).
        // cv::Ptr<cv::FeatureDetector> blobDetector =  new cv::SimpleBlobDetector(params);
        // Ptr<FeatureDetector> blobDetector = new SimpleBlobDetector(params);
         // cv::SimpleBlobDetector blobDetector(params);

     //OpenCV implementation for now
     for (size_t i = 0; i < numOfPose; i++) {
       /* extract frames at same positions */
       std::vector<std::vector<Point2f>> list;
        for (size_t j = 0; j < numOfRuns; j++) {
          /* extract one frame */
          std::vector<Point2f> corners;
          // imshow("hehe", ImageLists[i][j]);
          // waitKey(0);
					bool found;
					if(useChessboard) found = findChessboardCorners(ImageLists[i][j], cvSize(chessboardHight,chessboardWidth),corners);
          else found = findCirclesGrid(ImageLists[i][j], cvSize(chessboardHight,chessboardWidth),corners, CALIB_CB_ASYMMETRIC_GRID);

          if(!found)
            {
							std::cerr << "/* error message: Checkboard corners not found!*/" << std::endl;
              // throw std::runtime_error("Checkboard corners not found!");
            }else{
						if(useChessboard){
              Mat grey_img = ImageLists[i][j];
              // cv::cvtColor(ImageLists[i][j], grey_img, CV_BGR2GRAY);
            	cornerSubPix(grey_img, corners, Size(5,5), Size(-1,-1),
                        TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                    50, 0.001));
							}
						Mat tmp_show; ImageLists[i][j].copyTo(tmp_show);
						cvtColor(tmp_show, tmp_show, CV_GRAY2RGB);
            drawChessboardCorners(tmp_show, cvSize(chessboardHight,chessboardWidth), corners, found);
            imshow("extracting chessboard ",tmp_show );
            char k = waitKey(1000);
            if(k == 'u')
                std::reverse(corners.begin(),corners.end());
            std::cout << "extracting chessboard @ pose: "<< i << " run: "<< j << std::endl;
            list.push_back(corners);
          }
        }
        observedControlPts.push_back(list);
     }
     std::cout << "extract control pts done!" << std::endl;

}

void VisualPath::createImageLists(std::string revpath){

    for (int i = 0; i < numOfPose; i++) {
      /* code */
      std::vector<Mat> tmp = readPositionImg(i, revpath);
      ImageLists.push_back(tmp);
      std::cout << "reading img: "<< i <<"..."<< std::endl;
    }
		// Camera_Matrix.at<double>(0,0) = 308.8941007431293;
		// Camera_Matrix.at<double>(0,2) = 874.0915549387335;
		// Camera_Matrix.at<double>(1,1) = 308.8941007431293;
		// Camera_Matrix.at<double>(1,2) = 601.5506135924284;
		if(!ImageLists.empty())
		imageSize = ImageLists[0][0].size();

}

void VisualPath::undistortImage(cv::Mat& img){
    Mat output;
    undistort(img, output, Camera_Matrix, Distortion_Coefficients);
    output.copyTo(img);
    imshow("undistort",img);
    waitKey(10);
}
void VisualPath::FishEyeUndistortImage(cv::Mat& img){
	Mat output;
 Matx33d newK = Camera_Matrix;
 // newK(0, 0) = 308.8941007431293;
 // newK(0, 2) = 874.0915549387335;
 // newK(1, 1) = 308.8941007431293;
 // newK(1, 2) = 601.5506135924284;
	 // cv::fisheye::estimateNewCameraMatrixForUndistortRectify(Camera_Matrix, Distortion_Coefficients, img.size(), cv::noArray(), newK, 0.0);
	fisheye::undistortImage(img, output, Camera_Matrix, Distortion_Coefficients, newK);
	output.copyTo(img);
	imshow("undistort",img);
	waitKey(10);
}

void VisualPath::computeExtrinsic(){
      // Options for stereo calibratiosn
      // Zero Lens Distortation after undistortion
      int flags = CV_CALIB_FIX_INTRINSIC;
      TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6);
      R_list.push_back(Mat::eye(3,3,CV_64FC1));
      T_list.push_back(Mat::zeros(3,1,CV_64FC1));
    for (size_t i = 1; i < numOfPose; i++) {
      /* compute R & T for each pose*/
      cv::Mat relT;// (3, 1, CV_64F);
      cv::Mat relR;//(3, 3, CV_64F);
      cv::Mat essentialMat;//(3, 3, CV_64F);
      cv::Mat fundamentalMat;//(3, 4, CV_64F);

      Mat ZeroDistortion; Distortion_Coefficients.copyTo(ZeroDistortion);
      ZeroDistortion.at<double>(0) = ZeroDistortion.at<double>(1) = ZeroDistortion.at<double>(2) = ZeroDistortion.at<double>(3) = ZeroDistortion.at<double>(4)= 0;
      // std::cout << "ZeroDistortion: "<< ZeroDistortion << std::endl;
      // std::cout << "Camera_Matrix: "<< Camera_Matrix << std::endl;

      // Perform the actual calibration
			std::vector<std::vector<Point3f>> ControlPts(actualControlPts.begin(),actualControlPts.begin()+observedControlPts[i].size());
			std::vector<std::vector<Point2f>> BaseControlPts(observedControlPts[0].begin(),observedControlPts[0].begin()+observedControlPts[i].size());

			double reprojError = stereoCalibrate(
          ControlPts, BaseControlPts, observedControlPts[i],
          Camera_Matrix, ZeroDistortion,
          Camera_Matrix, ZeroDistortion,
          ImageLists[i][0].size(), // shouldn't be used since intrinsics are fixed
          relR, relT, essentialMat, fundamentalMat,
         flags,criteria);
      // Mat r_tmp = relR;
      // relT = r_tmp*T_list[i-1]+ relT;
      // relR = r_tmp*R_list[i-1];
      std::cout << "camera ID:" << i <<"    --> stereo reprojection error was " << reprojError << std::endl;
      R_list.push_back(relR);
      T_list.push_back(relT);
    }
}

void VisualPath::saveVisualPath(FileStorage& fs) const{
  for (size_t i = 0; i < R_list.size(); i++) {
    /* code */
    std::string name = "T_list_" + std::to_string(i);
    fs << name <<  T_list[i];
  }
  for (size_t i = 0; i < T_list.size(); i++) {
    /* code */
    std::string name = "R_list_" + std::to_string(i);
    fs << name <<  R_list[i];
  }
  Mat pts_ = Mat(p3d);
  fs << "Point3f" <<pts_;

	fs <<"local_R" << local_R;
	fs <<"local_T" << local_T;

	fs <<"chessboard_T" << chessboard_T;
	fs <<"chessboard_R" << chessboard_R;

	std::cerr << " local_R" << local_R << std::endl;
	std::cerr << " local_T" << local_T << std::endl;


  std::cout << "saved!!!" << std::endl;
}

void VisualPath::getCameraRTfromRobotArm(bool flag){
	std::cout << "/* local_R size: */" << local_R.size() << std::endl;
	std::cout << "/* local_T size: */" << local_T.size() << std::endl;

	for (size_t i = 0; i < R_robot_list.size(); i++) {
		//  transformation matrix from world to robot
		R_TCP_list.push_back(R_robot_list[i].t());
		// R_TCP_list.push_back(R_robot_list[i]);
		T_TCP_list.push_back(-R_TCP_list[i]*T_robot_list[i]);
		// T_TCP_list.push_back(T_robot_list[i]);

		/* First extend the TCP to camera center */
		/////????????????
		//local R & T, from robot cord to cam cord
		Mat R_tmp = local_R * R_TCP_list[i];
		Mat T_tmp = local_R * T_TCP_list[i] + local_T;

		// std::cout << "/* R_TCP_list size: */" << R_TCP_list[i].size() << std::endl;
		// std::cout << "/* T_TCP_list size: */" << T_TCP_list[i].size() << std::endl;
		/* Then transfer the camera center to Camera projection matrix */
		//************ for Triangulations
		if(flag){
			R_list.push_back(R_tmp);
			T_list.push_back(T_tmp);
		}
		// std::cout << "/* R_list size: */" << R_list[i].size() << std::endl;
		// std::cout << "/* T_list size: */" << T_list[i].size() << std::endl;
	}
	std::cout << "getCameraRTfromRobotArm done!" << std::endl;
}

void VisualPath::computeReporjectionError(bool flag){
	// Accumulators
	 double sumError = 0;
	 double sumSqError = 0;
	 double maxError = 0;
	 int nTargetPts = 0;
	 for (size_t i = 0; i < numOfPose; i++) {
	 	/* per view */
		  std::vector<Point2f> reprojPts;
			Mat R_Rodrigues;  Rodrigues(R_list[i],R_Rodrigues);
			//ZeroDistortion ????? because I undistort???
			Mat ZeroDistortion; Distortion_Coefficients.copyTo(ZeroDistortion);
      ZeroDistortion.at<double>(0) = ZeroDistortion.at<double>(1) = ZeroDistortion.at<double>(2) = ZeroDistortion.at<double>(3) = ZeroDistortion.at<double>(4)= 0;
			std::cerr<<"T: "<<T_list[i].total() *  T_list[i].channels() <<"   " << T_list[i].depth() << std::endl;
			projectPoints(p3d, R_Rodrigues, T_list[i], Camera_Matrix, ZeroDistortion, reprojPts);
			// fisheye::projectPoints(p3d, reprojPts, R_Rodrigues, T_list[i], Camera_Matrix, ZeroDistortion);
					double perPoseError = 0;
					for (size_t j = 0; j < chessboardWidth*chessboardHight; j++) {
						/* per feature of per view */
						double errX = reprojPts[j].x - observedControlPts[i][0][j].x;
		        double errY = reprojPts[j].y - observedControlPts[i][0][j].y;
		        double distSq = errX * errX + errY * errY;
		        double dist = sqrt(distSq);
						// std::cout << "/* errX */" << errX << std::endl;
		        sumError += dist;
						perPoseError += dist;
		        sumSqError += distSq;
		        nTargetPts++;
		        maxError = max(maxError, dist);
					}
					static int ccc = 0;
					if( flag ){
						 Mat tmp; ImageLists[i][0].copyTo(tmp);
						 drawChessboardCorners(tmp, cv::Size(chessboardHight,chessboardWidth), reprojPts, true);
						//  imwrite("reprojection"+std::to_string(ccc)+".jpg",tmp); waitKey(10);ccc++;
						 imshow("reprojection",tmp); waitKey(1000);ccc++;
					 }

			std::cout<<"Pose ID: " << i<<"  perPoseError: " << perPoseError/ (chessboardWidth*chessboardHight)<<"\n";
	 }
	 double meanError = sumError / nTargetPts;
	 double rmsError = sqrt(sumSqError / nTargetPts);

	 std::cout << std::endl <<"=== Reprojection error report ===" << std::endl;
	 std::cout << "    RMS error = " << rmsError << std::endl;
	 std::cout << "   Mean error = " << meanError << std::endl;
	 std::cout << "    Max error = " << maxError << std::endl << std::endl;
}

void VisualPath::getCameraRTfromBA(){

	for (size_t i = 0; i < R_robot_list.size(); i++) {

				Mat R_tmp = local_R * R_TCP_list[i];
				Mat T_tmp = local_R * T_TCP_list[i] + local_T;

				R_tmp.copyTo(R_list[i]);
				T_tmp.copyTo(T_list[i]);
			}

}
// disgard
void VisualPath::saveTCPPath(FileStorage& fs) const{
  for (size_t i = 0; i < T_TCP_list.size(); i++) {
    /* code */
    std::string name = "T_list_" + std::to_string(i);
    fs << name <<  T_TCP_list[i];
  }
  for (size_t i = 0; i < R_TCP_list.size(); i++) {
    /* code */
    std::string name = "R_list_" + std::to_string(i);
    fs << name <<  R_TCP_list[i];
  }
  Mat pts_ = Mat(p3d);
  fs << "Point3f" <<pts_;

	fs <<"local_R" << local_R;
	fs <<"local_T" << local_T;


	std::cerr << " local_R" << local_R << std::endl;
	std::cerr << " local_T" << local_T << std::endl;
	fs <<"chessboard_R" << chessboard_R;
	fs <<"chessboard_T" << chessboard_T;

  std::cout << "saved!!!" << std::endl;
}


void VisualPath::estimateChessboardPose(int camID){
		// currently use cam[0] to locate the chessboard,
		//however, cam[0] is also set as world frame
		//which is trival in this case, but it is for future complicated case
		Mat rvec, tvec;
		solvePnP(actualControlPts[0], observedControlPts[camID][0],Camera_Matrix, Mat::zeros(4, 1, CV_64F),
        rvec, tvec);
		Mat targetCameraRot;
		Rodrigues(rvec, targetCameraRot);

		Mat cameraWorldRot = R_list[camID].t();
	  Mat cameraWorldTrans = -cameraWorldRot*T_list[camID];

	   chessboard_R = Mat::eye(3, 3, CV_64F);
	   chessboard_T = Mat::zeros(3, 1, CV_64F);

	    // First rotate and translate to transform in to the camera frame

	    // Now rotate and translate to transform the camera to the world frame
	    chessboard_R = cameraWorldRot * targetCameraRot ;
	    chessboard_T = cameraWorldRot * tvec + cameraWorldTrans;

			std::cout << "/* estimate Chessboard Pose Done !*/" << std::endl;
			std::cout << chessboard_R << std::endl;
			std::cout << chessboard_T << std::endl;
			//transfer chessboard from camID frame to world frame
}

void VisualPath::subsampleControlPts(){
			// take the average/RANSAC of all the runs and comress them into one run[0]
			for (size_t i = 0; i < numOfPose; i++) {
				/* handle per pose */
				for (size_t j = 1; j < observedControlPts[i].size(); j++) {
					/* handle per run */
					for (size_t k = 0; k < chessboardWidth*chessboardHight; k++) {
						/* handle pre pts */
						observedControlPts[i][0][k] += observedControlPts[i][j][k];
					}
				}
				for (size_t x = 0; x < chessboardWidth*chessboardHight; x++) {
					/* average */
					observedControlPts[i][0][x].x = observedControlPts[i][0][x].x/observedControlPts[i].size();
					observedControlPts[i][0][x].y = observedControlPts[i][0][x].y/observedControlPts[i].size();
				}
			}

			// for (size_t i = 0; i < numOfPose; i++) {
			// 	/* handle per pose */
			// 	for (size_t j = 1; j < observedControlPts[i].size(); j++) {
			// 		/* handle per run */
			// 		for (size_t k = 0; k < chessboardWidth*chessboardHight; k++) {
			// 			/* handle pre pts */
			// 			std::cout << "diff: " <<
			// 			observedControlPts[i][0][k] - observedControlPts[i][j][k] <<  std::endl;
			// 		}
			// 	}
			// }
			std::cout << "subsampleControlPts done!" << std::endl;
}
Mat cameraPoseFromHomography(const Mat& H)
{
	  Mat pose = Mat::eye(3, 4, CV_64FC1);     // 3x4 matrix, the camera pose
    double norm1 = (double)norm(H.col(0));
    double norm2 = (double)norm(H.col(1));
    double tnorm = (norm1 + norm2) / 2.0f; // Normalization value

    cv::normalize(H.col(0), pose.col(0));   // Normalize the rotation, and copies the column to pose

    cv::normalize(H.col(1), pose.col(1));   // Normalize the rotation and copies the column to pose

    Mat p1 = pose.col(0);
    Mat p2 = pose.col(1);

    Mat p3 = p1.cross(p2);   // Computes the cross-product of p1 and p2
    p3.copyTo(pose.col(2));       // Third column is the crossproduct of columns one and two

		Mat tmp = H.col(2) / tnorm;
    tmp.copyTo(pose.col(3));   //vector t [R|t] is the last column of pose
		return pose;
}

void VisualPath::estimateRTfromHomography(){
	std::cout << "/* message */" << std::endl;
	R_list.clear();
	T_list.clear();
	std::vector<Point2f> points2D;
	for(auto& i : actualControlPts[0]){
		points2D.push_back(Point2f(i.x,i.y));
	}
	for (size_t i = 0; i < numOfPose; i++) {
		/* estimate R T from cameraCaliberation function */
		Mat rvec, tvec;
		Mat H = findHomography(points2D,observedControlPts[i][0]);
		Mat H_ = Camera_Matrix.inv()*H;
		std::cout << "/* H_ */" << H_ << std::endl;
		Mat pose = cameraPoseFromHomography(H_);
			Mat revc_tmp;
		// Rodrigues(rvec,revc_tmp);
		 R_list.push_back(pose.colRange(0,3));
		 T_list.push_back(pose.col(3));
		 std::cout << "/* R_list */" << R_list[i]<< std::endl;
		 std::cout << "/* T_list */" << T_list[i]<< std::endl;
	}
	std::cout << " estimate RT from single Homography done!" << std::endl;

}
