#include "AppleTree.h"

using namespace std;
using namespace cv;


arma::mat CV2Arma_double(Mat opencv_mat){
  assert(opencv_mat.type() == CV_64FC1);
  DLOG(INFO)<<"CV2Arma_double...input size: " << opencv_mat.rows <<" | " << opencv_mat.cols  <<"\n\n";
  arma::mat ret( reinterpret_cast<double*>(opencv_mat.data), opencv_mat.cols, opencv_mat.rows );
  ret = ret.t();
  assert(ret.n_rows == opencv_mat.rows && ret.n_cols == opencv_mat.cols );
  return ret;
}

Mat Arma2CV_double(arma::mat B){
  cv::Mat ret;
  cv::Mat opencv_mat( B.n_cols, B.n_rows, CV_64FC1, B.memptr() );
  transpose(opencv_mat, ret);
  assert(B.n_rows == ret.rows && B.n_cols == ret.cols);
  return ret;
}


void AppleJuice::ReadImageLists(const opt_ options, unsigned int display){
    DLOG(INFO) << "start AppleJuice::ReadImageLists ....\n";
    string base_name = options.Image_list_dir + options.pose_prefix;

    for (size_t i = 0; i < options.N_poses; i++) {
        vector<Mat> tmp_vector;
        string file_name = base_name + (to_string(i) + "/");
        arma::cube tmp_blob(options.image_height, options.image_width, options.N_shoots );
        DLOG(INFO) << "Arma Blob size:" << tmp_blob.n_rows <<" | " << tmp_blob.n_cols <<" | " <<
tmp_blob.n_slices <<"\n";

        for (size_t j = 0; j < options.N_shoots; j++) {

             string reading_name = file_name + options.image_prefix + to_string(j) + options.image_type;
             DLOG(INFO) << "/* message */ reading: " << reading_name << std::endl;
             Mat image = imread(reading_name , CV_LOAD_IMAGE_GRAYSCALE);
             if(! image.data ){
              string ErrMessage = "Cannot read image:" + reading_name ;
              throw std::runtime_error( ErrMessage );
            }
            if(display){
              imshow(file_name, image);
              waitKey(1000);
            }
            // Using INTER_AREA Here @Eric
            DLOG(INFO) << "image size:" << image.size() <<" | channels: "<< image.channels() << " | type: "<< image.depth();

            cv::resize(image, image, Size(options.image_width,options.image_height),INTER_AREA);
            image.convertTo(image, CV_64FC1);
            tmp_blob.slice(j) = CV2Arma_double(image);
            tmp_vector.push_back(image);
        }
        this->ImageBlob.push_back(tmp_blob);
        // this->ImageLists.push_back(tmp_vector);
    }
    // assert(ImageLists.size() == options.N_poses && ImageLists[0].size() == options.N_shoots);
    assert(ImageBlob.size() == options.N_poses && ImageBlob[0].n_slices == options.N_shoots);
    std::cout << "/* ImageLists loading is done... */" << std::endl;
}

void AppleJuice::BinarizeAllImages(){
        assert(ImageBlob.size() != 0);
        DLOG(INFO) << "start AppleJuice::BinarizeAllImages() ....\n";

        for (size_t i = 0; i < ImageBlob.size(); i++) {
            arma::ucube tmp_blob(ImageBlob[i].n_rows, ImageBlob[i].n_cols, ImageBlob[i].n_slices);
            tmp_blob.fill(2);

            // Find the  & min map of each blob
            DLOG(INFO) << "Blob size: "<<  ImageBlob[0].n_rows <<" | "<< ImageBlob[0].n_cols << " | " << ImageBlob[0].n_slices <<"\n";
            arma::mat min_map = arma::min(ImageBlob[i],2);
            arma::mat max_map = arma::max(ImageBlob[i],2);
            arma::mat high_threshold_map = max_map * (1 - this->options.suppression_ratio )   + min_map * this->options.suppression_ratio;
            arma::mat low_threshold_map = min_map * (1 - this->options.suppression_ratio )   + max_map * this->options.suppression_ratio;

            arma::umat mask = arma::zeros<arma::umat>(options.image_height, options.image_width);
            mask.elem( find( (max_map - min_map)  >= options.suppression_ratio*255 ) ).ones();
            this->Masks.push_back(mask);
            // mask.save("mask"+to_string(i)+".mat", arma::raw_ascii);


            for (size_t x = 0; x < ImageBlob[i].n_slices; x++) {
              // the ambigulty == 2 !!!!!!!!
              arma::mat tmp_mat = ImageBlob[i].slice(x) - high_threshold_map;
              // if it is greater than high_threshold, assign 1
              tmp_blob.slice(x).elem( find(tmp_mat  >= 0 ) ).ones();
              tmp_mat = ImageBlob[i].slice(x) - low_threshold_map;
              // if it is smaller than low_threshold, assign 0
              tmp_blob.slice(x).elem( find(tmp_mat  <= 0 ) ).zeros();

              // if(DEBUG) tmp_blob.slice(x).save("blob" + to_string(i) + " slice " + to_string(x) + ".mat", arma::raw_ascii);
            }
            this->BinaryBlob.push_back(tmp_blob);
            DLOG(INFO) << "Save Blob: " << i << " done!\n";
        }
      std::cout << "/* message */ AppleJuice::BinarizeAllImages Done..." << std::endl;
    return;
}



void AppleJuice::ExtractControlPts(){

    for (size_t i = 0; i < BinaryBlob.size(); i++) {
      /* loop every pose */
        arma::uvec cords = find(Masks[i] == 1);
        DLOG(INFO) << "Search Region Size: " << cords.size() << endl;
        arma::umat check_map = arma::zeros<arma::umat>(options.image_height, options.image_width);

        unsigned int outlier_cnt = 0;
        map<Point3f, pair<int, Point2f>, Point3fComp> checker;

        for (size_t j = 0; j < cords.size(); j++) {
          /* code */
          unsigned int row_ = cords(j) % options.image_height;
          unsigned int col_ = cords(j) / options.image_height;
          arma::umat tmp_string = BinaryBlob[i].subcube(arma::span(row_,row_),arma::span(col_,col_),arma::span::all);
          stringstream ss;
          tmp_string.print(ss);
          string target = ss.str();
          string::iterator end_pos = std::remove(target.begin(), target.end(), ' ');
          target.erase(end_pos, target.end());
          // if(DEBUG)
            // DLOG(INFO) << "target string: " <<"[" << row_ <<" , " << col_ << "]\n"<< target <<"\n";

          if(target.substr(0,3) != "010" && target.substr(target.size()-3,3) != "101")
              DLOG(INFO) << "Outlier....!!!"<< ++outlier_cnt << endl;
          // if(DEBUG)
          //     DLOG(INFO) << endl << "X encoding: " << target.substr(3,options.x_encoding_len) << endl
          //                << "Y encoding: " << target.substr(3+options.x_encoding_len,options.y_encoding_len) << endl;
          pair<Point3f,Point3f> pts = SearchPoints(target.substr(3,options.x_encoding_len), target.substr(3+options.x_encoding_len,options.y_encoding_len));
          DLOG(INFO) <<pts.first <<" | "<< pts.second<<endl;
          if(pts.first.x >= 0 && pts.first.y >= 0){
              auto finder = checker.find(pts.first);
              if ( finder == checker.end()){
                    checker.insert(make_pair(pts.first, make_pair(1,Point2f(col_,row_))));
              }else{
                    finder->second.first++;
                  }
          }else{
            ++outlier_cnt;
          }


        }
        DLOG(INFO) << "checker map size: " << checker.size() << " | outlier: " << outlier_cnt << endl;
        //Insert into pool....
        vector<Point2f> single_featurePool;
        vector<Point3f>  single_PatternPool;
        unsigned int conflicts_cnt = 0;
        for(auto itr = checker.begin(); itr != checker.end(); itr++) {
              if (itr->second.first == 1) {
                /* no conflicts */
                check_map(round(itr->first.y), round(itr->first.x)) = 1;
                single_featurePool.push_back(itr->second.second);
                single_PatternPool.push_back(itr->first);
              }else
                  DLOG(INFO) << "Reject one conflicts!!!  " << itr->first << "#: " << itr->second.first  << " | " << ++conflicts_cnt <<"/" << cords.size()<< endl;
        }
        this->FeaturePool.push_back(single_featurePool);
        this->PatternPtsPool.push_back(single_PatternPool);
        // if(DEBUG)
        //   check_map.save("check_map" + to_string(i) + ".mat",arma::raw_ascii  );
      }

      assert(FeaturePool.size() == PatternPtsPool.size() && FeaturePool[0].size() == PatternPtsPool[0].size());
      std::cerr << "/* message */ AppleJuice::ExtractControlPts done..." << std::endl;
}

void AppleJuice::InitCameraCalibration(){
  //subsample FeaturePool && PatternPtsPool
  unsigned int subsample_N = 100;
  std::vector<std::vector<Point3f>> subPatternPool;
  std::vector<std::vector<Point2f>> subFeaturePool;
  assert(FeaturePool.size() == PatternPtsPool.size() && FeaturePool[0].size() == PatternPtsPool[0].size());

  for (size_t i = 0; i < FeaturePool.size(); i++) {
    /* loop every pose */

    std::srand ( unsigned ( std::time(0) ) );
    std::vector<unsigned int> index_(FeaturePool[i].size());
    std::iota(index_.begin(), index_.end(), 0);
    std::random_shuffle ( index_.begin(), index_.end() );

    subPatternPool.push_back(std::vector<Point3f>());
    subFeaturePool.push_back(std::vector<Point2f>());
    for (size_t j = 0; j < subsample_N; j++) {
      /* code */
      subFeaturePool[i].push_back( FeaturePool[i][index_[j]] );
      subPatternPool[i].push_back( PatternPtsPool[i][index_[j]] );
    }

    DLOG(WARNING) << "subFeaturePool " << i <<" size: "<< subFeaturePool[i].size() << endl;
    DLOG(WARNING) << "subPatternPool " << i <<" size: "<< subPatternPool[i].size() << endl;

  }
  //Start init Calibration....
  Mat init_cameraMatrix = Mat::eye(3, 3, CV_64F);
  Mat fix_distCoeffs = Mat::zeros(8, 1, CV_64F);
  vector<Mat> rvecs, tvecs;

  // if(DEBUG)
  //     for (size_t i = 0; i < subPatternPool.size(); i++) {
  //       /* code */
  //       for (size_t j = 0; j < subPatternPool[i].size(); j++) {
  //         /* code */
  //         cerr<< "pattern: " << subPatternPool[i][j] << " | feature: " << subFeaturePool[i][j] << endl;
  //       }
  //     }
  //Caliberate with assuming no distortation
  double rms = calibrateCamera(PatternPtsPool, FeaturePool, cv::Size(options.image_width, options.image_height), init_cameraMatrix,
                            fix_distCoeffs, rvecs, tvecs, CV_CALIB_FIX_K1|CV_CALIB_FIX_K2
                                                         |CV_CALIB_FIX_K3|CV_CALIB_FIX_K4
                                                         |CV_CALIB_ZERO_TANGENT_DIST);

  DLOG(WARNING) << "init.. calibration finish......" << endl
  <<   "init_cameraMatrix:\n " << init_cameraMatrix << endl
  <<   "fix_distCoeffs:\n " << fix_distCoeffs.t() << endl
  <<   "Rot size: & Trans size: " << rvecs.size() <<" |  " <<  tvecs.size() << endl;

  auto tmp_intrinsic = Mat_<double>(init_cameraMatrix);
  // std::cerr << "tmp_intrinsic" << tmp_intrinsic << std::endl;
  // waitKey(0);
  this->intrinsic = Intrinsic(tmp_intrinsic(0,0), tmp_intrinsic(1,1), tmp_intrinsic(0,2), tmp_intrinsic(1,2));
  for (size_t x = 0; x < options.N_poses; x++) {
    /* insert Extrinsics */
    this->extrinsic_list.push_back(Extrinsic(rvecs[x], tvecs[x]));
  }

  std::cerr << "/* message */ AppleJuice::InitCameraCalibration done..." << std::endl;
  return;
}

void AppleJuice::CreatingOffsetMap(){
  assert(FeaturePool.size() == PatternPtsPool.size() && FeaturePool[0].size() == PatternPtsPool[0].size());
  DLOG(INFO) << "AppleJuice::CreatingOffsetMap starts...." <<endl;
  map<Point2f, Point3f, Point2fComp> checker;
  // Second Point3f: x-> num of observations, y-> pose index, z->point index
  for (size_t i = 0; i < FeaturePool.size(); i++) {
    /* loop each pose */
      for (size_t j = 0; j < FeaturePool[i].size(); j++) {
        /* checker checks all observations */
          auto finder = checker.find(FeaturePool[i][j]);
          if( finder == checker.end() ){
              checker.insert(make_pair(FeaturePool[i][j],Point3f(1,i,j)));
          }else{
              finder->second.x++; //observations ++
          }
      }
  }
  arma::umat Optimizion_pts_check_map = arma::zeros<arma::umat>(options.image_width, options.image_height);

  for(auto itr = checker.begin(); itr != checker.end(); itr++) {
        if(itr->second.x == 1){
            unsigned int i_ = itr->second.y;
            unsigned int j_ = itr->second.z;
            //delete only one observation pts
            FeaturePool[i_][j_] = Point2f(-1,-1);
            PatternPtsPool[i_][j_] = Point3f(-1,-1,-1);
        }else if(itr->second.x > 1){
            //index keeps the same as purned FeaturePool && PatternPtsPool
            double * tmp = new double[2]{0};
            // tmp[0] = intrinsic.pptx; tmp[1] = intrinsic.ppty;
            if ( this->offset_map.find(itr->first) == offset_map.end() ) {
              this->offset_map.insert(make_pair(itr->first,tmp));
            }
            Optimizion_pts_check_map(itr->first.y, itr->first.x) = 1;
        }
  }
  Optimizion_pts_check_map.save("final_optimziation_pts_map.mat", arma::raw_ascii);
  DLOG(WARNING) << "actuall control pts that can be optimized: " << offset_map.size() << "/" << checker.size() << endl;
  return;
}


void AppleJuice::computeReporjectionError(bool flag){
	// Accumulators
	 double sumError = 0;
	 double sumSqError = 0;
	 double maxError = 0;
	 int nTargetPts = 0;
	 for (size_t i = 0; i < options.N_poses; i++) {
	 	/* per view */
		  std::vector<Point2f> reprojPts;
			Mat R_Rodrigues;  Rodrigues(this->extrinsic_list[i].Rot,R_Rodrigues);

			Mat ZeroDistortion = Mat::zeros(4,1,CV_64FC1);

      ZeroDistortion.at<double>(0) = ZeroDistortion.at<double>(1) = ZeroDistortion.at<double>(2) = ZeroDistortion.at<double>(3) = ZeroDistortion.at<double>(4)= 0;
      Mat Camera_Matrix = Mat::zeros(3,3,CV_64FC1);
      Camera_Matrix.at<double>(0,0) = this->intrinsic.focus_x;
      Camera_Matrix.at<double>(1,1) = this->intrinsic.focus_y;
      Camera_Matrix.at<double>(0,2) = this->intrinsic.pptx;
      Camera_Matrix.at<double>(1,2) = this->intrinsic.ppty;

			projectPoints(this->PatternPtsPool[i], R_Rodrigues, this->extrinsic_list[i].Trans,
         Camera_Matrix, ZeroDistortion, reprojPts);
         unsigned int outlier_cnt = 0;
					double perPoseError = 0;
					for (size_t j = 0; j < PatternPtsPool[i].size(); j++) {
            if (FeaturePool[i][j].x < 0 || FeaturePool[i][j].y < 0)
                    continue;
						/* per feature of per view */
						double errX = reprojPts[j].x - FeaturePool[i][j].x;
		        double errY = reprojPts[j].y - FeaturePool[i][j].y;
		        double distSq = errX * errX + errY * errY;
		        double dist = sqrt(distSq);
            if(dist > 5 ){
                outlier_cnt++;
                continue;
            }
						// std::cout << "/* errX */" << errX << std::endl;
		        sumError += dist;
						perPoseError += dist;
		        sumSqError += distSq;
		        nTargetPts++;
		        maxError = max(maxError, dist);
					}
					static int ccc = 0;
					// if( flag ){
					// 	 Mat tmp; ImageLists[i][0].copyTo(tmp);
					// 	 drawChessboardCorners(tmp, cv::Size(chessboardHight,chessboardWidth), reprojPts, true);
					// 	//  imwrite("reprojection"+std::to_string(ccc)+".jpg",tmp); waitKey(10);ccc++;
					// 	 imshow("reprojection",tmp); waitKey(1000);ccc++;
					//  }

			std::cout<<"Pose ID: " << i<<"  perPoseError: " << perPoseError/ ( PatternPtsPool[i].size() - outlier_cnt) <<
      " with outlier: " << outlier_cnt << "\n";
	 }
	 double meanError = sumError / nTargetPts;
	 double rmsError = sqrt(sumSqError / nTargetPts);

	 std::cout << std::endl <<"=== Reprojection error report ===" << std::endl;
	 std::cout << "    RMS error = " << rmsError << std::endl;
	 std::cout << "   Mean error = " << meanError << std::endl;
	 std::cout << "    Max error = " << maxError << std::endl << std::endl;
}
