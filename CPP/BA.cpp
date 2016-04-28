#ifndef BA
#define BA
#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/loss_function.h"
#include "AppleTree.h"
#include "float.h"

using namespace std;

pair<double, double>  Proprocessing(std::vector<std::vector<cv::Point3f>>& PatternPtsPool, std::vector<std::vector<cv::Point2f>>& FeaturePool, std::vector<Extrinsic>& extrinsic_list ){
					//Prepocissing to get the min and max value of r_square for regularization
					assert(PatternPtsPool.size() == extrinsic_list.size());
					assert(FeaturePool.size() == PatternPtsPool.size() && FeaturePool[0].size() == PatternPtsPool[0].size());

					double max_r = 0.0;
					double mean_r = 0.0;
					double min_r = DBL_MAX;
					unsigned int all_counter = 0;
					std::vector<double> r_vector;
					for (size_t i = 0; i < PatternPtsPool.size(); i++) {
						//loop every pose
						double pose_R[3] = {extrinsic_list[i].Rot.at<double>(0),
																extrinsic_list[i].Rot.at<double>(1),
															  extrinsic_list[i].Rot.at<double>(2)};
						double pose_T[3] = {extrinsic_list[i].Trans.at<double>(0),
																extrinsic_list[i].Trans.at<double>(1),
																extrinsic_list[i].Trans.at<double>(2)};

						for (size_t j = 0; j < PatternPtsPool[i].size(); j++) {
							/* for every pts */

							if (FeaturePool[i][j].x < 0 || FeaturePool[i][j].y < 0) {
								//jump useless pts
								continue;
							}
							double refPt[3];
							refPt[0] = PatternPtsPool[i][j].x;
							refPt[1] = PatternPtsPool[i][j].y;
							refPt[2] = PatternPtsPool[i][j].z;

							double rotPt[3];
							ceres::AngleAxisRotatePoint(pose_R, refPt, rotPt);

							double p[3];
							p[0] = pose_T[0] + rotPt[0];
							p[1] = pose_T[1] + rotPt[1];
							p[2] = pose_T[2] + rotPt[2];

							double x1 = p[0] / p[2];
							double y1 = p[1] / p[2];

							r_vector.push_back(sqrt(x1*x1 + y1*y1));
							max_r = max(sqrt(x1*x1 + y1*y1), max_r);
							min_r = min(sqrt(x1*x1 + y1*y1), min_r);
							mean_r += sqrt(x1*x1 + y1*y1);
							all_counter += 1;
						}

					}
					std::sort(r_vector.begin(), r_vector.end());
					for (size_t i = 0; i < r_vector.size(); i++) {
						/* code */
						// std::cout << "/* r_vector: */ " << r_vector[i] << " "<<std::endl;
					}
					std::cerr << "/* max min mean */" << max_r << "  |  " << min_r << "  |  " << mean_r / all_counter  << std::endl;
					return make_pair( mean_r/all_counter, min_r);
}


struct DynaimcChessboardReprojectionError{
    DynaimcChessboardReprojectionError( cv::Point2f observedPoints, cv::Point3f chessboardPts, double mean_r_Square, double min_r_Square):
		observedPoints(observedPoints),chessboardPts(chessboardPts), mean_r_Square(mean_r_Square), min_r_Square(min_r_Square){}
    template <typename T>
		//observedPoints & chessboard model are actuall observations

    bool operator()(
				const T* const pose_R, 			 // 3 variables
				const T* const pose_T,			 // 3 variables
				const T* const intrinsic,			 // 2 variables, only focus_X & focus_Y
				const T* const offset_pair,   // 2 variables in many.....
        T* residuals								 // 2 residuals
        ) const {
        // === (1) Find the positions of the chessboard points in chessboard coordinates

        double refPt[3];
        refPt[0] = T(chessboardPts.x);
        refPt[1] = T(chessboardPts.y);
        refPt[2] = T(chessboardPts.z);

        double rotPt[3];

        ceres::AngleAxisRotatePoint(pose_R, refPt, rotPt);

        T* p = new T[3];
        p[0] = pose_T[0] + rotPt[0];
        p[1] = pose_T[1] + rotPt[1];
        p[2] = pose_T[2] + rotPt[2];

        // === (2) Projectpoint to find the reprojection error

        // Finally, apply the projection equation
        double x1 = p[0] / p[2];
        double y1 = p[1] / p[2];

				double r_Square = x1*x1 + y1*y1;

				// T focus_X = intrinsic[0];
				// T focus_Y = intrinsic[1];
				// T pptx = intrinsic[2];
				// T ppty = intrinsic[3];
				// T offset_x = offset_pair[0];
				// T offset_y = offset_pair[1];

        double predicted_x = x1*intrinsic[0] + intrinsic[2] + offset_pair[0];
        double predicted_y = y1*intrinsic[1] + intrinsic[3] + offset_pair[1];
				//need better formaliation
				double lamda = 1.0 ;
				// double lamda = 1.0 / r_Square ;
        if (lamda < (0.0)) {
					DLOG(FATAL) << "lamda < 0\n";
					exit(0);
        }
        // The error is the difference between the predicted and observed position.
        residuals[0] = (predicted_x - observedPoints.x) ;
        residuals[1] = (predicted_y - observedPoints.y) ;
        residuals[2] = offset_pair[0] * sqrt(lamda);
        residuals[3] = offset_pair[1] * sqrt(lamda);
						// std::cout << "/* residuals x *m/" << residuals[2]<< std::endl;
        return true;
			}
			static ceres::CostFunction* Create(const cv::Point2f observedPoints, cv::Point3f chessboardPts, double mean_r_Square, double min_r_Square) {
				 return (new ceres::NumericDiffCostFunction<DynaimcChessboardReprojectionError,ceres::CENTRAL, 4, 3, 3, 4, 2>(
						 new DynaimcChessboardReprojectionError(observedPoints, chessboardPts, mean_r_Square, min_r_Square)));
 }

 cv::Point2f observedPoints;
 cv::Point3f chessboardPts;
 double mean_r_Square, min_r_Square;
};


void AppleJuice::AppleBundleAdjustment() {
	/* code */
	cout << endl << "=== Solving Dynamic Chessboard Bundle Adjustment Problem with CERES ===" << endl;

	//Define Problem
	 ceres::Problem problem;
	 std::vector<double*> R_list(options.N_poses, NULL);
	 std::vector<double*> T_list(options.N_poses, NULL);

	 pair<double,double> min_max_r_Square =  Proprocessing(this->PatternPtsPool,this->FeaturePool, this->extrinsic_list);

				std::cerr << "r_Square range:" << min_max_r_Square.first << " | " << min_max_r_Square.second << std::endl;
				// return;
	 DLOG(WARNING) << endl << "=== preprocessing pts done === " << endl;


	 for (size_t i = 0; i < options.N_poses; i++) {
	 	/* add R & T */
			R_list[i] = new double[3];
			T_list[i] = new double[3];
			double* r_ = R_list[i];
      double* t_ = T_list[i];

			r_[0] = extrinsic_list[i].Rot.at<double>(0);
			r_[1] = extrinsic_list[i].Rot.at<double>(1);
			r_[2] = extrinsic_list[i].Rot.at<double>(2);

			t_[0] = extrinsic_list[i].Trans.at<double>(0);
			t_[1] = extrinsic_list[i].Trans.at<double>(1);
			t_[2] = extrinsic_list[i].Trans.at<double>(2);

			problem.AddParameterBlock(R_list[i], 3);
			problem.AddParameterBlock(T_list[i], 3);

			cv::Mat t;	cv::Rodrigues(extrinsic_list[i].Rot,t);
			DLOG(WARNING) << "init guess Rot & Trans " <<i <<" : "<< t << " | " << extrinsic_list[i].Trans << std::endl;


	 }

   DLOG(WARNING) << endl << "=== injecting cam rt pose done === " << endl;

	 double* focus_x_y = new double[4];
	 focus_x_y[0] = intrinsic.focus_x;
	 focus_x_y[1] = intrinsic.focus_y;
	 focus_x_y[2] = intrinsic.pptx;
	 focus_x_y[3] = intrinsic.ppty;

	 DLOG(WARNING) << "init guess focus_x & focus_y : " << focus_x_y[0] << " | " << focus_x_y[1] << std::endl;

	 problem.AddParameterBlock(focus_x_y, 4);

	 DLOG(WARNING) << endl << "=== injecting camera intrinscs done === " << endl;

	 for (auto it = offset_map.begin(); it != offset_map.end(); it++) {
		 	// it's gonna be huge!!!
			// DLOG(WARNING) << "init guess Offset: " << it->second[0] << " | " << it->second[1] << std::endl;
			problem.AddParameterBlock(it->second, 2);
	 }

	 DLOG(WARNING) << endl << "=== injecting camera offset_map done === " << endl;

	 assert(FeaturePool.size() == PatternPtsPool.size() && FeaturePool[0].size() == PatternPtsPool[0].size());

	 unsigned int awesome_counter = 0;
	 for (size_t i = 0; i < FeaturePool.size(); i++) {
	 	/* loop every pose */
				for (size_t j = 0; j < FeaturePool[0].size(); j++) {
					/* jump the only one observation pts */
					if( FeaturePool[i][j].x < 0 || FeaturePool[i][j].y < 0){
						// std::cerr << "/* jumping */ i: " << i << " j = " << j << std::endl;
									continue;
								}
								auto offset_pair = offset_map.find(FeaturePool[i][j]);
								// auto offset_pair = offset_map.begin();
					ceres::CostFunction* cost_function =
						 DynaimcChessboardReprojectionError::Create(FeaturePool[i][j],
																					PatternPtsPool[i][j],
																					min_max_r_Square.first,
																					min_max_r_Square.second);
																					//  ceres::LossFunction* loss_function(new ceres:: CauchyLoss(1.0));
																					//  ceres::LossFunction* loss_function(new ceres:: HuberLoss(1.0));
													// std::cerr << "/* error message */ i: " << i << " j = " << j << std::endl;
													// 	std::cerr << "/* address */" << R_list[i] << std::endl
													// 															<< T_list[i] << std::endl
													// 															<< focus_x_y << std::endl
													// 															<< offset_pair->second << std::endl;
						 problem.AddResidualBlock(cost_function,
																			NULL /* squared loss */,
																			R_list[i],
																			T_list[i],
																			focus_x_y,
																			offset_pair->second
																		);
						awesome_counter++;
				}
	 }
	 	DLOG(WARNING) << " === injecting all Error terms done, Num: "<< awesome_counter << " ===";
	 	DLOG(WARNING) << " === BA Starts RUN ===";

	  ceres::Solver::Options options;
		options.function_tolerance = 1e-5;
	  options.max_num_iterations = 50;
	  options.linear_solver_type = ceres::DENSE_SCHUR;
	  options.minimizer_progress_to_stdout = true;
	  ceres::Solver::Summary summary;
	  ceres::Solve(options, &problem, &summary);
	  std::cout << summary.FullReport() << "\n";
		// cout<<observations[0]<<endl;

		DLOG(WARNING) << " === BA DONE === " << std::endl << "start returning results..." << endl;

		arma::mat result_map_x = arma::zeros<arma::mat>(this->options.image_width, this->options.image_height);
		arma::mat result_map_y = arma::zeros<arma::mat>(this->options.image_width, this->options.image_height);
		for (auto it = offset_map.begin(); it != offset_map.end(); it++) {
			 result_map_x(it->first.y, it->first.x) = it->second[0];
			 result_map_y(it->first.y, it->first.x) = it->second[1];
		}
		std::cerr  << "results: \n focus_X: "  << focus_x_y[0] << endl
		 					 << "focus_Y: " << focus_x_y[1] << endl
		 					 << "pptx: " << focus_x_y[2] << endl
		 					 << "ppty: " << focus_x_y[3] << endl;

		for (size_t i = 0; i < this->options.N_poses; i++) {
			cv::Mat_<double> t = cv::Mat_<double>(3,1);
			t(0) = R_list[i][0];
			t(1) = R_list[i][1];
			t(2) = R_list[i][2];
			cv::Rodrigues(t,t);
			std::cerr   << "results: \n pose Rot: "  << t << endl
								 << "pose Trans: " << T_list[i][0] <<" " << T_list[i][1] << " " << T_list[i][2] << endl;
		}
		result_map_x.save("result_map_x.mat",arma::raw_ascii);
		result_map_y.save("result_map_y.mat",arma::raw_ascii);
		DLOG(INFO) << "result_map_x & y have saved ";

	  return ;
}




#endif
