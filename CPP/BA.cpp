#ifndef BA
#define BA
#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/loss_function.h"
#include "VisualPath.h"

using namespace std;


void CovarianceWriter(double* mat, string name){
	return;
	cerr<<"Mat: " << name << "\n" <<"[";
	for (size_t i = 0; i < 3; i++) {
		/* code */
		for (size_t j = 0; j < 3; j++){
			cerr<<mat[i*3 + j];
			if(j != 2) cerr<<", ";}
		cerr<<";";
	}
	cerr <<"]\n";
}

struct VPReprojectionError {
		VPReprojectionError(double observed_x, double observed_y, double focus, double pptx, double ppty) : observed_x(observed_x), observed_y(observed_y),
		focus(focus), pptx(pptx), ppty(ppty) {}
    template <typename T>
    bool operator()(
    				  const T* const r, //3+3=6
              const T* const t,
    				  // const T* const distortion, //2
                     const T* const point, //3
                    T* residuals) const {
      // rt[0,1,2] are the angle-axis rotation.
      T p[3];
      ceres::AngleAxisRotatePoint(r, point, p);
      // rt[3,4,5] are the translation.
      p[0] += t[0]; p[1] += t[1]; p[2] += t[2];

      // Compute the center of distortion. The sign change comes from
      // the rt model that Noah Snavely's Bundler assumes, whereby
      // the rt coordinate system has a negative z axis.
      T xp =  p[0] / p[2];
      T yp =  p[1] / p[2];

      T predicted_x = focus  * xp + pptx;
      T predicted_y = focus  * yp + ppty;

      // T predicted_x = focalx * xp;
      // T predicted_y = focaly * yp;

      // The error is the difference between the predicted and observed position.
      residuals[0] = predicted_x - T(observed_x);
      residuals[1] = predicted_y - T(observed_y);
      return true;
    }

     // Factory to hide the construction of the CostFunction object from
     // the client code.
     static ceres::CostFunction* Create(const double observed_x,
                                        const double observed_y,
																				const double focus,
																				const double pptx,
																				const double ppty) {
       return (new ceres::NumericDiffCostFunction<VPReprojectionError,ceres::CENTRAL, 2, 3, 3, 3>(
                   new VPReprojectionError(observed_x, observed_y, focus, pptx, ppty)));

                }

					double observed_x;
					double observed_y;
					double focus;
					double pptx;
					double ppty;
};

struct VPRobotReprojectionError {
		VPRobotReprojectionError(double observed_x, double observed_y) : observed_x(observed_x), observed_y(observed_y) {}
    template <typename T>
    bool operator()(const T* const intrinsics, //4
    				  const T* const r, //3+3=6
              const T* const t,
							const T* const r_local, //3+3=6
							const T* const t_local,
    				  // const T* const distortion, //2
                     const T* const point, //3
                    T* residuals) const {
      // rt[0,1,2] are the angle-axis rotation.
      T c[3];
      ceres::AngleAxisRotatePoint(r, point, c);
      // rt[3,4,5] are the translation.
      c[0] += t[0]; c[1] += t[1]; c[2] += t[2];

			//From TCP to Cam center
			T p[3];
			ceres::AngleAxisRotatePoint(r_local, c, p);
      // rt[3,4,5] are the translation.
      p[0] += t_local[0]; p[1] += t_local[1]; p[2] += t_local[2];
      // Compute the center of distortion. The sign change comes from
      // the rt model that Noah Snavely's Bundler assumes, whereby
      // the rt coordinate system has a negative z axis.
      T xp =  p[0] / p[2];
      T yp =  p[1] / p[2];

      // Apply second and fourth order radial distortion.
      // const T& l1 = distortion[0];
      // const T& l2 = distortion[1];
      // const T& pptx = intrinsics[2];
      // const T& ppty = intrinsics[3];


      // T r2 = xd*xd + yd*yd;
      // T d = T(1.0) + r2  * (l1 + l2  * r2);

      // Compute final projected point position.
      const T& focalx = intrinsics[0];
      const T& focaly = intrinsics[1];
      T predicted_x = focalx  * xp + T(631.50);
      T predicted_y = focaly  * yp + T(507.50);
      // T predicted_x = focalx * xp;
      // T predicted_y = focaly * yp;

      // The error is the difference between the predicted and observed position.
      residuals[0] = predicted_x - T(observed_x);
      residuals[1] = predicted_y - T(observed_y);
      return true;
    }

     // Factory to hide the construction of the CostFunction object from
     // the client code.
     static ceres::CostFunction* Create(const double observed_x,
                                        const double observed_y) {
       return (new ceres::AutoDiffCostFunction<VPRobotReprojectionError, 2, 4, 3, 3, 3, 3, 3>(
                   new VPRobotReprojectionError(observed_x, observed_y)));
                }

					double observed_x;
					double observed_y;
};

struct ChessboardRobotArmReprojectionError{
    ChessboardRobotArmReprojectionError(vector<cv::Point2f> observedPoints, std::vector<cv::Point3f> chessboardModel, double focus_X, double focus_Y, double pptx, double ppty, const double* extrinsicsR,
		     const double* extrinsicsT)
    : observedPoints(observedPoints),chessboardModel(chessboardModel), focus_X(focus_X), focus_Y(focus_Y), pptx(pptx), ppty(ppty), extrinsicsR(extrinsicsR), extrinsicsT(extrinsicsT) {}
    template <typename T>

    bool operator()(
        // const T* const extrinsicsR,
        // const T* const extrinsicsT,
        const T* const targetR,
        const T* const targetT,
				const T* const local_R,
				const T* const local_T,
				const T* const scales,
        T* residuals
        ) const {
        // === (1) Find the positions of the chessboard points in world coordinates

        // Generate the reference points from chessboardModel (1-D)

        // Transform the reference points
        vector<T*> worldCoords;
        for (int iPt = 0; iPt < chessboardModel.size(); iPt++) {
            T refPt[3];
            refPt[0] = T(chessboardModel[iPt].x);
            refPt[1] = T(chessboardModel[iPt].y);
            refPt[2] = T(chessboardModel[iPt].z);

            T rotPt[3];
            ceres::AngleAxisRotatePoint(targetR, refPt, rotPt);

            T* transPt = new T[3];
            transPt[0] = rotPt[0]*scales[0] + targetT[0];
            transPt[1] = rotPt[1]*scales[1] + targetT[1];
            transPt[2] = rotPt[2]*scales[2] + targetT[2];

            worldCoords.push_back(transPt);
        }
        // === (2) Project each point to find the reprojection error
        for (int iPt = 0; iPt < worldCoords.size(); iPt++) {

            T* point = worldCoords[iPt];

            // Rotate from the camera's frame
            T revRot[3];

            for (int i = 0; i < 3; i++) { revRot[i] = T(extrinsicsR[i]); } //base2tool_R

            T p_[3];
            ceres::AngleAxisRotatePoint(revRot, point, p_);

            // Now translate point from world frame to robot frame

						// p_[0] *= scales[0];
						// p_[1] *= scales[1];
						// p_[2] *= scales[2];

						p_[0] += T(extrinsicsT[0]);
						p_[1] += T(extrinsicsT[1]);
						p_[2] += T(extrinsicsT[2]);

						//Transform point from robot frame cord to camera frame
						T p[3];
						ceres::AngleAxisRotatePoint(local_R,p_,p);
						p[0] += local_T[0];
						p[1] += local_T[1];
						p[2] += local_T[2];

            // Finally, apply the projection equation
            T x1 = p[0] / p[2];
            T y1 = p[1] / p[2];

            T predicted_x = x1*focus_X + pptx;
            T predicted_y = y1*focus_Y + ppty;


            // The error is the difference between the predicted and observed position.
            residuals[2*iPt + 0] = predicted_x - T(observedPoints[iPt].x);
            residuals[2*iPt + 1] = predicted_y - T(observedPoints[iPt].y);
						// std::cout << "/* residuals x */" << residuals[2*iPt + 0]<< std::endl;
            delete[] worldCoords[iPt];
        }
        return true;
			}
			static ceres::CostFunction* Create(const vector<cv::Point2f> observedPts,
				 const vector<cv::Point3f> chessboardModel, const double focus_X, const double focus_Y, const double pptx, const double ppty, const double* extrinsicsR,
		 		    const double* extrinsicsT ) {
				 return (new ceres::AutoDiffCostFunction<ChessboardRobotArmReprojectionError, 5*8*2, 3, 3, 3, 3, 3 >(
						 new ChessboardRobotArmReprojectionError(observedPts, chessboardModel, focus_X, focus_Y, pptx, ppty, extrinsicsR, extrinsicsT)));
 }

 vector<cv::Point2f> observedPoints;
 vector<cv::Point3f> chessboardModel;
 double focus_X, focus_Y, pptx, ppty;
 const double* extrinsicsR;
 const double* extrinsicsT;
};

struct ChessboardReprojectionError{
    ChessboardReprojectionError(vector<cv::Point2f> observedPoints, std::vector<cv::Point3f> chessboardModel, double focus_X, double focus_Y, double pptx, double ppty)
    : observedPoints(observedPoints),chessboardModel(chessboardModel), focus_X(focus_X), focus_Y(focus_Y), pptx(pptx), ppty(ppty) {}
    template <typename T>

    bool operator()(
        const T* const extrinsicsR,
        const T* const extrinsicsT,
        const T* const targetR,
        const T* const targetT,
        T* residuals
        ) const {
        // === (1) Find the positions of the chessboard points in world coordinates

        // Generate the reference points from chessboardModel (1-D)

        // Transform the reference points
        vector<T*> worldCoords;
        for (int iPt = 0; iPt < chessboardModel.size(); iPt++) {
            T refPt[3];
            refPt[0] = T(chessboardModel[iPt].x);
            refPt[1] = T(chessboardModel[iPt].y);
            refPt[2] = T(chessboardModel[iPt].z);

            T rotPt[3];
            ceres::AngleAxisRotatePoint(targetR, refPt, rotPt);

            T* transPt = new T[3];
            transPt[0] = rotPt[0] + targetT[0];
            transPt[1] = rotPt[1] + targetT[1];
            transPt[2] = rotPt[2] + targetT[2];

            worldCoords.push_back(transPt);
        }
        // === (2) Project each point to find the reprojection error
        for (int iPt = 0; iPt < worldCoords.size(); iPt++) {

            T* point = worldCoords[iPt];

            // Rotate from the camera's frame
            T revRot[3];

            for (int i = 0; i < 3; i++) { revRot[i] = extrinsicsR[i]; }

            T p[3];
            ceres::AngleAxisRotatePoint(revRot, point, p);

            // Now translate from the camera's frame
            p[0] += extrinsicsT[0];
            p[1] += extrinsicsT[1];
            p[2] += extrinsicsT[2];

            // Finally, apply the projection equation
            T x1 = p[0] / p[2];
            T y1 = p[1] / p[2];

            T predicted_x = x1*focus_X + pptx;
            T predicted_y = y1*focus_Y + ppty;


            // The error is the difference between the predicted and observed position.
            residuals[2*iPt + 0] = predicted_x - T(observedPoints[iPt].x);
            residuals[2*iPt + 1] = predicted_y - T(observedPoints[iPt].y);
						// std::cout << "/* residuals x */" << residuals[2*iPt + 0]<< std::endl;
            delete[] worldCoords[iPt];
        }
        return true;
			}
			static ceres::CostFunction* Create(const vector<cv::Point2f> observedPts,
				 const vector<cv::Point3f> chessboardModel, const double focus_X, const double focus_Y, const double pptx, const double ppty) {

				 return (new ceres::AutoDiffCostFunction<ChessboardReprojectionError, 5*8*2, 3, 3, 3, 3>(
						 new ChessboardReprojectionError(observedPts, chessboardModel, focus_X, focus_Y, pptx, ppty)));
 }

 vector<cv::Point2f> observedPoints;
 vector<cv::Point3f> chessboardModel;
 double focus_X, focus_Y, pptx, ppty;
};


void VisualPath::bundleAdjustment() {
	/* code */
	cout << endl << "=== Solving Bundle Adjustment Problem with CERES" << endl;

	 // Define the cost function

	 // Set up the problem
	 ceres::Problem problem;


   std::vector<double*> r_list(numOfPose,NULL);
   std::vector<double*> t_list(numOfPose,NULL);
  //  double* intrinsics = new double[2];
  //  intrinsics[0] = Camera_Matrix.at<double>(0,0);
  //  intrinsics[1] = Camera_Matrix.at<double>(1,1);
  //  intrinsics[2] = Camera_Matrix.at<double>(0,2);
  //  intrinsics[3] = Camera_Matrix.at<double>(1,2);


   std::vector<double*> guessPts(chessboardHight*chessboardWidth,NULL);

   for (size_t i = 0; i < numOfPose; i++) {
     /* code */
      // 	ceres::LossFunction* loss_function(new ceres:: CauchyLoss(1.0));
      r_list[i] = new double[3];
      t_list[i] = new double[3];
      double* r_ = r_list[i];
      double* t_ = t_list[i];

      //transpose ???????????????
      cv::Mat transpose;
      R_list[i].copyTo(transpose);
      transpose = transpose.t();
      ceres::RotationMatrixToAngleAxis(reinterpret_cast<double*>(transpose.data),r_);

      t_[0] = T_list[i].at<double>(0);
      t_[1] = T_list[i].at<double>(1);
      t_[2] = T_list[i].at<double>(2);

   }

   cerr << endl << "=== rt pose done: " << endl;

   for (size_t j = 0; j < chessboardHight*chessboardWidth; j++) {
     /* code */
     guessPts[j] = new double[3];
     double* guessPts_ = guessPts[j];
     guessPts_[0] = p3d[j].x;
     guessPts_[1] = p3d[j].y;
     guessPts_[2] = p3d[j].z;
   }
  cout << endl << "=== 3dpts  done: " << endl;

   for (size_t i = 0; i < numOfPose; i++) {
      for (size_t j = 0; j < chessboardHight*chessboardWidth; j++) {
        /* num of Observation */
        ceres::CostFunction* cost_function =
           VPReprojectionError::Create(observedControlPts[i][0][j].x,
                                       observedControlPts[i][0][j].y,
																		 	 (Camera_Matrix.at<double>(0,0)+Camera_Matrix.at<double>(1,1))/2,
																			 Camera_Matrix.at<double>(0,2),
																			 Camera_Matrix.at<double>(1,2)
																		 );

			// ceres::LossFunction* loss_function(new ceres:: CauchyLoss(1.0));

       problem.AddResidualBlock(cost_function,
                                NULL /* squared loss */,
                                r_list[i],
                                t_list[i],
                                guessPts[j]);
      }
   }
   std::cout << "/* run BA */" << std::endl;

  ceres::Solver::Options options;
  options.max_num_iterations = 500;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
	// cout<<observations[0]<<endl;

  std::cout << "/* returning BA result... */" << std::endl;

  std::cout << "Camera_Matrix: before BA: " <<  Camera_Matrix << std::endl;


  // Camera_Matrix.at<double>(0,0) = intrinsics[0];
  // Camera_Matrix.at<double>(1,1) = intrinsics[1];
  // Camera_Matrix.at<double>(0,2) = intrinsics[2];
  // Camera_Matrix.at<double>(1,2) = intrinsics[3];

  std::cout << "Camera_Matrix: after BA: " <<  Camera_Matrix << std::endl;
  //
  for (size_t i = 0; i < numOfPose; i++) {
     double* R = new double[9];
     ceres::AngleAxisToRotationMatrix<double>(r_list[i],R);
     cv::Mat R_( 3, 3, CV_64FC1, R );
     //transpose....????????
     R_ = R_.t();
     R_.copyTo(R_list[i]);

     cv::Mat ptr = T_list[i];
     double* p = t_list[i];
     T_list[i].at<double>(0,0) = p[0];
     T_list[i].at<double>(1,0) = p[1];
     T_list[i].at<double>(2,0) = p[2];
  }


  for (size_t i = 0; i < chessboardHight*chessboardWidth; i++) {
    /* code */
    double* guessPts_ = guessPts[i];
     p3d[i].x = guessPts_[0];
     p3d[i].y = guessPts_[1];
     p3d[i].z = guessPts_[2];
  }
  return ;

}

void VisualPath::bundleAdjustmentWithRobotArm(){
		/* code */
		cout << endl << "=== Solving Bundle Adjustment Problem with CERES with Robot Arms" << endl;

		 // Define the cost function

		 // Set up the problem
		 ceres::Problem problem;

	   std::vector<double*> r_list(numOfPose,NULL);
	   std::vector<double*> t_list(numOfPose,NULL);
		 double* local_R_BA = new double[3];
		 double* local_T_BA = new double[3];
	   double* intrinsics = new double[2];
	   intrinsics[0] = Camera_Matrix.at<double>(0,0);
	   intrinsics[1] = Camera_Matrix.at<double>(1,1);
	  //  intrinsics[2] = Camera_Matrix.at<double>(0,2);
	  //  intrinsics[3] = Camera_Matrix.at<double>(1,2);


		 ///////////asign local R & T
		 cv::Mat local_R_Rodrigues;
		 cv::Rodrigues(local_R,local_R_Rodrigues);
		 local_R_BA[0] = local_R_Rodrigues.at<double>(0);
		 local_R_BA[1] = local_R_Rodrigues.at<double>(1);
		 local_R_BA[2] = local_R_Rodrigues.at<double>(2);

		 local_T_BA[0] = local_T.at<double>(0);
		 local_T_BA[1] = local_T.at<double>(1);
		 local_T_BA[2] = local_T.at<double>(2);

	   std::vector<double*> guessPts(chessboardHight*chessboardWidth,NULL);

	   for (size_t i = 0; i < numOfPose; i++) {
	     /* code */
	      r_list[i] = new double[3];
	      t_list[i] = new double[3];
	      double* r_ = r_list[i];
	      double* t_ = t_list[i];

		   cv::Mat R_Rodrigues;
	 		 cv::Rodrigues(R_TCP_list[i],R_Rodrigues);
			 r_[0] = R_Rodrigues.at<double>(0);
			 r_[1] = R_Rodrigues.at<double>(1);
			 r_[2] = R_Rodrigues.at<double>(2);

	      t_[0] = T_TCP_list[i].at<double>(0,0);
	      t_[1] = T_TCP_list[i].at<double>(1,0);
	      t_[2] = T_TCP_list[i].at<double>(2,0);

	   }

	   cerr << endl << "=== rt pose done: " << endl;

	   for (size_t j = 0; j < chessboardHight*chessboardWidth; j++) {
	     /* code */
	     guessPts[j] = new double[3];
	     double* guessPts_ = guessPts[j];
	     guessPts_[0] = p3d[j].x;
	     guessPts_[1] = p3d[j].y;
	     guessPts_[2] = p3d[j].z;
	   }
	  cout << endl << "=== 3dpts  done: " << endl;

	   for (size_t i = 0; i < numOfPose; i++) {
	      for (size_t j = 0; j < chessboardHight*chessboardWidth; j++) {
	        /* num of Observation */
	        ceres::CostFunction* cost_function =
	           VPRobotReprojectionError::Create(observedControlPts[i][0][j].x,
	                                       observedControlPts[i][0][j].y);

	       problem.AddResidualBlock(cost_function,
	                                NULL /* squared loss */,
	                                intrinsics,
	                                r_list[i],
	                                t_list[i],
																	local_R_BA,
																	local_T_BA,
	                                guessPts[j]);
	      }
	   }
	   std::cout << "/* run BA */" << std::endl;

	  ceres::Solver::Options options;
	  options.max_num_iterations = 500;
	  options.linear_solver_type = ceres::DENSE_SCHUR;
	  options.minimizer_progress_to_stdout = true;
	  ceres::Solver::Summary summary;
	  ceres::Solve(options, &problem, &summary);
	  std::cout << summary.FullReport() << "\n";
		// cout<<observations[0]<<endl;

	  std::cout << "/* returning BA result... */" << std::endl;

	   std::cout << "Camera_Matrix: before BA: " <<  Camera_Matrix << std::endl;


	  Camera_Matrix.at<double>(0,0) = intrinsics[0];
	  Camera_Matrix.at<double>(1,1) = intrinsics[1];
	  // Camera_Matrix.at<double>(0,2) = intrinsics[2];
	  // Camera_Matrix.at<double>(1,2) = intrinsics[3];

	   std::cout << "Camera_Matrix: after BA: " <<  Camera_Matrix << std::endl;
	  //
	  for (size_t i = 0; i < numOfPose; i++) {

			 cv::Mat R_( 3, 1, CV_64FC1, r_list[i] );
			 cv::Mat R_Mat;
			 cv::Rodrigues(R_,R_Mat);
	     R_Mat.copyTo(R_TCP_list[i]);

	     double* p = t_list[i];
	     T_TCP_list[i].at<double>(0,0) = p[0];
	     T_TCP_list[i].at<double>(1,0) = p[1];
	     T_TCP_list[i].at<double>(2,0) = p[2];
	  }

	  for (size_t i = 0; i < chessboardHight*chessboardWidth; i++) {
	    /* code */
	    double* guessPts_ = guessPts[i];
	     p3d[i].x = guessPts_[0];
	     p3d[i].y = guessPts_[1];
	     p3d[i].z = guessPts_[2];
	  }

		local_T.at<double>(0,0) = local_T_BA[0];
		local_T.at<double>(1,0) = local_T_BA[1];
		local_T.at<double>(2,0) = local_T_BA[2];

		cv::Mat R_( 3, 1, CV_64FC1, local_R_BA);
		cv::Rodrigues(R_,local_R);

	  return ;

}

void VisualPath::chessboardRobotArmBundleAdjustment() {
	/* code */
	cout << endl << "=== Solving Chessboard-RobotArm Bundle Adjustment Problem with CERES" << endl;
	cout<<"local_R: " << local_R <<"\n";
	cout<<"local_T: " << local_T <<"\n";
	 // Define the cost function

	 // Set up the problem
	 ceres::Problem problem;

   std::vector<double*> r_list(numOfPose,NULL);
   std::vector<double*> t_list(numOfPose,NULL);

	 cv::Mat local_R_Rodrigues;
	 double * local_R_BA = new double[3];
	 double * local_T_BA = new double[3];

	 cv::Rodrigues(chessboard_R,local_R_Rodrigues);
	 local_R_BA[0] = local_R_Rodrigues.at<double>(0);
	 local_R_BA[1] = local_R_Rodrigues.at<double>(1);
	 local_R_BA[2] = local_R_Rodrigues.at<double>(2);

	 local_T_BA[0] = chessboard_T.at<double>(0);
	 local_T_BA[1] = chessboard_T.at<double>(1);
	 local_T_BA[2] = chessboard_T.at<double>(2);
	 cerr << endl << "=== chessboard pose done: " << endl;

	 problem.AddParameterBlock(local_R_BA, 3);
	 problem.AddParameterBlock(local_T_BA, 3);

	 scales = new double[3];
	 scales[0] = scales[1] = scales[2] = 1;

   for (size_t i = 0; i < numOfPose; i++) {
     /* code */
      // 	ceres::LossFunction* loss_function(new ceres:: CauchyLoss(1.0));
      r_list[i] = new double[3];
      t_list[i] = new double[3];
      double* r_ = r_list[i];
      double* t_ = t_list[i];

			cv::Mat transpose;
			cv::Rodrigues(R_TCP_list[i],transpose);
      r_[0] = transpose.at<double>(0);
      r_[1] = transpose.at<double>(1);
      r_[2] = transpose.at<double>(2);

      t_[0] = T_TCP_list[i].at<double>(0);
      t_[1] = T_TCP_list[i].at<double>(1);
      t_[2] = T_TCP_list[i].at<double>(2);

				// problem.AddParameterBlock(r_list[i], 3);
				// problem.AddParameterBlock(t_list[i], 3);
   }

   cerr << endl << "=== cam rt pose done: " << endl;

	 //set camera_robotarm_rotation & translation
	 cv::Mat robot_R_Rodrigues;
	 double * robot_R_BA = new double[3];
	 double * robot_T_BA = new double[3];

	 cv::Rodrigues(local_R,robot_R_Rodrigues);
	 robot_R_BA[0] = robot_R_Rodrigues.at<double>(0);
	 robot_R_BA[1] = robot_R_Rodrigues.at<double>(1);
	 robot_R_BA[2] = robot_R_Rodrigues.at<double>(2);

	 robot_T_BA[0] = local_T.at<double>(0);
	 robot_T_BA[1] = local_T.at<double>(1);
	 robot_T_BA[2] = local_T.at<double>(2);
	 cerr << endl << "=== robot->cam pose done: " << endl;

	 problem.AddParameterBlock(robot_R_BA, 3);
	 problem.AddParameterBlock(robot_T_BA, 3);

	 //do we set the first pose as constant???
	//  problem.SetParameterBlockConstant(r_list[0]);
  //  problem.SetParameterBlockConstant(t_list[0]);

   for (size_t i = 0; i < numOfPose; i++) {
	        /* num of Observation */
	        ceres::CostFunction* cost_function =
	           ChessboardRobotArmReprojectionError::Create(observedControlPts[i][0],
							 														actualControlPts[0],
																			 	 Camera_Matrix.at<double>(0,0),
																				 Camera_Matrix.at<double>(1,1),
																				 Camera_Matrix.at<double>(0,2),
																				 Camera_Matrix.at<double>(1,2),
																				 r_list[i],
																				 t_list[i]
																			 );
			//  ceres::LossFunction* loss_function(new ceres:: CauchyLoss(1.0));
			 ceres::LossFunction* loss_function(new ceres:: HuberLoss(1.0));

	// std::cout << "/* message */" << std::endl;
	       problem.AddResidualBlock(cost_function,
	                                NULL /* squared loss */,
																	local_R_BA,
																	local_T_BA,
																	robot_R_BA,
																	robot_T_BA,
																	scales
	                                );
	   }

	  std::cout << "/* run BA */" << std::endl;

	  ceres::Solver::Options options;
		options.function_tolerance = 1e-9;
	  options.max_num_iterations = 50;
	  options.linear_solver_type = ceres::DENSE_SCHUR;
	  options.minimizer_progress_to_stdout = true;
	  ceres::Solver::Summary summary;
	  ceres::Solve(options, &problem, &summary);
	  std::cout << summary.FullReport() << "\n";
		// cout<<observations[0]<<endl;

	  std::cout << "/* returning BA result... */" << std::endl;

		cv::Mat chessboard_R_ret = cv::Mat::zeros(3,1,CV_64FC1);
		chessboard_R_ret.at<double>(0) = local_R_BA[0];
		chessboard_R_ret.at<double>(1) = local_R_BA[1];
		chessboard_R_ret.at<double>(2) = local_R_BA[2];

		cv::Rodrigues(chessboard_R_ret,chessboard_R);

		chessboard_T.at<double>(0) = local_T_BA[0];
		chessboard_T.at<double>(1) = local_T_BA[1];
		chessboard_T.at<double>(2) = local_T_BA[2];

		cv::Mat robot_R_ret = cv::Mat::zeros(3,1,CV_64FC1);
		robot_R_ret.at<double>(0) = robot_R_BA[0];
		robot_R_ret.at<double>(1) = robot_R_BA[1];
		robot_R_ret.at<double>(2) = robot_R_BA[2];

		cv::Rodrigues(robot_R_ret, local_R);

		local_T.at<double>(0) = robot_T_BA[0];
		local_T.at<double>(1) = robot_T_BA[1];
		local_T.at<double>(2) = robot_T_BA[2];

	  //
	  for (size_t i = 0; i < numOfPose; i++) {
	     double* R = new double[9];
	     ceres::AngleAxisToRotationMatrix<double>(r_list[i],R);
	     cv::Mat R_( 3, 3, CV_64FC1, R );
	     //transpose....????????
	     R_ = R_.t();
	     R_.copyTo(R_TCP_list[i]);

	     cv::Mat ptr = T_list[i];
	     double* p = t_list[i];
	     T_TCP_list[i].at<double>(0,0) = p[0];
	     T_TCP_list[i].at<double>(1,0) = p[1];
	     T_TCP_list[i].at<double>(2,0) = p[2];
	  }

		for (int x = 0; x < 3; x++) {
			cerr<<"scale: "<<scales[x]<<"\n";
		}

	  // return ;
		// Analysis Coverance Matrix
		std::cerr << "covariance start" << std::endl;

		ceres::Covariance::Options optionz;
		ceres::Covariance covariance(optionz);
		vector<pair<const double*, const double*> > covariance_blocks;
		// local_R_BA,
		// local_T_BA,
		// robot_R_BA,
		// robot_T_BA,
		// scales
		covariance_blocks.push_back(make_pair(local_R_BA, local_R_BA));
		covariance_blocks.push_back(make_pair(local_T_BA, local_T_BA));
		covariance_blocks.push_back(make_pair(robot_R_BA, robot_R_BA));
		covariance_blocks.push_back(make_pair(robot_T_BA, robot_T_BA));
		covariance_blocks.push_back(make_pair(scales, scales));

		covariance_blocks.push_back(make_pair(local_R_BA, local_T_BA));
		covariance_blocks.push_back(make_pair(local_R_BA, robot_R_BA));
		covariance_blocks.push_back(make_pair(local_R_BA, robot_T_BA));
		covariance_blocks.push_back(make_pair(local_R_BA, scales));

		covariance_blocks.push_back(make_pair(local_T_BA, robot_R_BA));
		covariance_blocks.push_back(make_pair(local_T_BA, robot_T_BA));
		covariance_blocks.push_back(make_pair(local_T_BA, scales));

		covariance_blocks.push_back(make_pair(robot_R_BA, robot_T_BA));
		covariance_blocks.push_back(make_pair(robot_R_BA, scales));

		covariance_blocks.push_back(make_pair(robot_T_BA, scales));

		covariance.Compute(covariance_blocks, &problem);

		double covariance_local_R_BA2local_R_BA[3 * 3];
		double covariance_local_T_BA2local_T_BA[3 * 3];
		double covariance_robot_R_BA2robot_R_BA[3 * 3];
		double covariance_robot_T_BA2robot_T_BA[3 * 3];
		double covariance_scales2scales[3 * 3];

		double covariance_local_R_BA2local_T_BA[3 * 3];
		double covariance_local_R_BA2robot_R_BA[3 * 3];
		double covariance_local_R_BA2robot_T_BA[3 * 3];
		double covariance_local_R_BA2scales[3 * 3];

		double covariance_local_T_BA2robot_R_BA[3 * 3];
		double covariance_local_T_BA2robot_T_BA[3 * 3];
		double covariance_local_T_BA2scales[3 * 3];

		double covariance_robot_R_BA2robot_T_BA[3 * 3];
		double covariance_robot_R_BA2scales[3 * 3];

		double covariance_robot_T_BA2scales[3 * 3];

		covariance.GetCovarianceBlock(local_R_BA, local_R_BA, covariance_local_R_BA2local_R_BA);
		covariance.GetCovarianceBlock(local_T_BA, local_T_BA, covariance_local_T_BA2local_T_BA);
		covariance.GetCovarianceBlock(robot_R_BA, robot_R_BA, covariance_robot_R_BA2robot_R_BA);
		covariance.GetCovarianceBlock(robot_T_BA, robot_T_BA, covariance_robot_T_BA2robot_T_BA);
		covariance.GetCovarianceBlock(scales, scales, covariance_scales2scales);

		covariance.GetCovarianceBlock(local_R_BA, local_T_BA, covariance_local_R_BA2local_T_BA);
		covariance.GetCovarianceBlock(local_R_BA, robot_R_BA, covariance_local_R_BA2robot_R_BA);
		covariance.GetCovarianceBlock(local_R_BA, robot_T_BA, covariance_local_R_BA2robot_T_BA);
		covariance.GetCovarianceBlock(local_R_BA, scales, covariance_local_R_BA2scales);

		covariance.GetCovarianceBlock(local_T_BA, robot_R_BA, covariance_local_T_BA2robot_R_BA);
		covariance.GetCovarianceBlock(local_T_BA, robot_T_BA, covariance_local_T_BA2robot_T_BA);
		covariance.GetCovarianceBlock(local_T_BA, scales, covariance_local_T_BA2scales);

		covariance.GetCovarianceBlock(robot_R_BA, robot_T_BA, covariance_robot_R_BA2robot_T_BA);
		covariance.GetCovarianceBlock(robot_R_BA, scales, covariance_robot_R_BA2scales);

		covariance.GetCovarianceBlock(robot_T_BA, scales, covariance_robot_T_BA2scales);


		CovarianceWriter(covariance_local_R_BA2local_R_BA,"covariance_local_R_BA2local_R_BA");
		CovarianceWriter(covariance_local_T_BA2local_T_BA,"covariance_local_T_BA2local_T_BA");
		CovarianceWriter(covariance_robot_R_BA2robot_R_BA,"covariance_robot_R_BA2robot_R_BA");
		CovarianceWriter(covariance_robot_T_BA2robot_T_BA,"covariance_robot_T_BA2robot_T_BA");
		CovarianceWriter(covariance_scales2scales,"covariance_scales2scales");

		CovarianceWriter(covariance_local_R_BA2local_T_BA,"covariance_local_R_BA2local_T_BA");
		CovarianceWriter(covariance_local_R_BA2robot_R_BA,"covariance_local_R_BA2robot_R_BA");
		CovarianceWriter(covariance_local_R_BA2robot_T_BA,"covariance_local_R_BA2robot_T_BA");
		CovarianceWriter(covariance_local_R_BA2scales,"covariance_local_R_BA2scales");

		CovarianceWriter(covariance_local_T_BA2robot_R_BA,"covariance_local_T_BA2robot_R_BA");
		CovarianceWriter(covariance_local_T_BA2robot_T_BA,"covariance_local_T_BA2robot_T_BA");
		CovarianceWriter(covariance_local_T_BA2scales,"covariance_local_T_BA2scales");

		CovarianceWriter(covariance_robot_R_BA2robot_T_BA,"covariance_robot_R_BA2robot_T_BA");
		CovarianceWriter(covariance_robot_R_BA2scales,"covariance_robot_R_BA2scales");

		CovarianceWriter(covariance_robot_T_BA2scales,"covariance_robot_T_BA2scales");

		std::cerr << "covariance done" << std::endl;
		return;
}


void VisualPath::chessboardBundleAdjustment() {
	/* code */
	cout << endl << "=== Solving Chessboard Bundle Adjustment Problem with CERES" << endl;

	 // Define the cost function

	 // Set up the problem
	 ceres::Problem problem;

   std::vector<double*> r_list(numOfPose,NULL);
   std::vector<double*> t_list(numOfPose,NULL);

	 cv::Mat local_R_Rodrigues;
	 double * local_R_BA = new double[3];
	 double * local_T_BA = new double[3];

	 cv::Rodrigues(chessboard_R,local_R_Rodrigues);
	 local_R_BA[0] = local_R_Rodrigues.at<double>(0);
	 local_R_BA[1] = local_R_Rodrigues.at<double>(1);
	 local_R_BA[2] = local_R_Rodrigues.at<double>(2);

	 local_T_BA[0] = chessboard_T.at<double>(0);
	 local_T_BA[1] = chessboard_T.at<double>(1);
	 local_T_BA[2] = chessboard_T.at<double>(2);
	 cerr << endl << "=== chessboard pose done: " << endl;

	 problem.AddParameterBlock(local_R_BA, 3);
	 problem.AddParameterBlock(local_T_BA, 3);

   for (size_t i = 0; i < numOfPose; i++) {
     /* code */
      // 	ceres::LossFunction* loss_function(new ceres:: CauchyLoss(1.0));
      r_list[i] = new double[3];
      t_list[i] = new double[3];
      double* r_ = r_list[i];
      double* t_ = t_list[i];

      //transpose ???????????????
      cv::Mat transpose;
      R_list[i].copyTo(transpose);
      transpose = transpose.t();
      ceres::RotationMatrixToAngleAxis(reinterpret_cast<double*>(transpose.data),r_);

      t_[0] = T_list[i].at<double>(0);
      t_[1] = T_list[i].at<double>(1);
      t_[2] = T_list[i].at<double>(2);

				problem.AddParameterBlock(r_list[i], 3);
				problem.AddParameterBlock(t_list[i], 3);
   }

   cerr << endl << "=== rt pose done: " << endl;

	 problem.SetParameterBlockConstant(r_list[0]);
   problem.SetParameterBlockConstant(t_list[0]);

   for (size_t i = 0; i < numOfPose; i++) {
	        /* num of Observation */
	        ceres::CostFunction* cost_function =
	           ChessboardReprojectionError::Create(observedControlPts[i][0],
							 														actualControlPts[0],
																			 	 Camera_Matrix.at<double>(0,0),
																				 Camera_Matrix.at<double>(1,1),
																				 Camera_Matrix.at<double>(0,2),
																				 Camera_Matrix.at<double>(1,2)
																			 );
				// ceres::LossFunction* loss_function(new ceres:: CauchyLoss(1.0));
	// std::cout << "/* message */" << std::endl;
	       problem.AddResidualBlock(cost_function,
	                                NULL /* squared loss */,
	                                r_list[i],
	                                t_list[i],
																	local_R_BA,
																	local_T_BA
	                                );
	   }

	  std::cout << "/* run BA */" << std::endl;

	  ceres::Solver::Options options;
		options.function_tolerance = 1e-9;
	  options.max_num_iterations = 500;
	  options.linear_solver_type = ceres::DENSE_SCHUR;
	  options.minimizer_progress_to_stdout = true;
	  ceres::Solver::Summary summary;
	  ceres::Solve(options, &problem, &summary);
	  std::cout << summary.FullReport() << "\n";
		// cout<<observations[0]<<endl;

	  std::cout << "/* returning BA result... */" << std::endl;

		cv::Mat chessboard_R_ret = cv::Mat::zeros(3,1,CV_64FC1);
		chessboard_R_ret.at<double>(0) = local_R_BA[0];
		chessboard_R_ret.at<double>(1) = local_R_BA[1];
		chessboard_R_ret.at<double>(2) = local_R_BA[2];

		cv::Rodrigues(chessboard_R_ret,chessboard_R);

		chessboard_T.at<double>(0) = local_T_BA[0];
		chessboard_T.at<double>(1) = local_T_BA[1];
		chessboard_T.at<double>(2) = local_T_BA[2];

	  //
	  for (size_t i = 0; i < numOfPose; i++) {
	     double* R = new double[9];
	     ceres::AngleAxisToRotationMatrix<double>(r_list[i],R);
	     cv::Mat R_( 3, 3, CV_64FC1, R );
	     //transpose....????????
	     R_ = R_.t();
	     R_.copyTo(R_list[i]);

	     cv::Mat ptr = T_list[i];
	     double* p = t_list[i];
	     T_list[i].at<double>(0,0) = p[0];
	     T_list[i].at<double>(1,0) = p[1];
	     T_list[i].at<double>(2,0) = p[2];
	  }

	  return ;

}

void VisualPath::generateChessboardAfterBundleAdjustment(int chessboardID){
	p3d.clear();
	std::vector<cv::Point3f> chessboardModel = actualControlPts[chessboardID];
	for (int iPt = 0; iPt < chessboardModel.size(); iPt++) {

			double refPt[3];
			refPt[0] = chessboardModel[iPt].x;
			refPt[1] = chessboardModel[iPt].y;
			refPt[2] = chessboardModel[iPt].z;

			double rotPt[3];
			double * local_R_BA = new double[3];
			cv::Mat local_R_Rodrigues;
			cv::Rodrigues(chessboard_R,local_R_Rodrigues);
			local_R_BA[0] = local_R_Rodrigues.at<double>(0);
			local_R_BA[1] = local_R_Rodrigues.at<double>(1);
			local_R_BA[2] = local_R_Rodrigues.at<double>(2);

			ceres::AngleAxisRotatePoint(local_R_BA, refPt, rotPt);

			cv::Point3f transPt;
			transPt.x = rotPt[0]*scales[0] + chessboard_T.at<double>(0);
			transPt.y = rotPt[1]*scales[1] + chessboard_T.at<double>(1);
			transPt.z = rotPt[2]*scales[2] + chessboard_T.at<double>(2);

			p3d.push_back(transPt);
			delete[] local_R_BA;
	}
}


///////////////////////////////////////////////////camera robot calibration//////////////////////////////////////

struct CameraRobotArmMatchError{
    CameraRobotArmMatchError( const double* R_base2tool, const double* T_base2tool, const double* R_target2cam, const double* T_target2cam)
    : R_base2tool(R_base2tool),T_base2tool(T_base2tool), R_target2cam(R_target2cam), T_target2cam(T_target2cam){}
    template <typename T>

    bool operator()(
        const T* const target2base_R,
        const T* const target2base_T,
				const T* const tool2cam_R,
				const T* const tool2cam_T,
        T* residuals
        ) const {
					T tmp[3];
					T r[3];
					r[0] = T(R_base2tool[0]);
					r[1] = T(R_base2tool[1]);
					r[2] = T(R_base2tool[2]);

					ceres::AngleAxisRotatePoint(r, target2base_T, tmp);
					tmp[0] += T(T_base2tool[0]);
					tmp[1] += T(T_base2tool[1]);
					tmp[2] += T(T_base2tool[2]);

					T pred[3];
					ceres::AngleAxisRotatePoint(tool2cam_R, tmp, pred);
					pred[0] += tool2cam_T[0];
					pred[1] += tool2cam_T[1];
					pred[2] += tool2cam_T[2];


          residuals[0] = T(T_target2cam[0]) - pred[0];
          residuals[1] = T(T_target2cam[1]) - pred[1];
          residuals[2] = T(T_target2cam[2]) - pred[2];

					//Rotation loss


					T target_r[3];
					target_r[0] = T(R_target2cam[0]);
					target_r[1] = T(R_target2cam[1]);
					target_r[2] = T(R_target2cam[2]);

					T xaxis[3];
					xaxis[0] = T(1);
					xaxis[1] = T(0);
					xaxis[2] = T(0);
					T tmp_rot[3];
					T tmp_tmp[3];
					ceres::AngleAxisRotatePoint(target2base_R, xaxis, tmp_tmp);
					ceres::AngleAxisRotatePoint(r, tmp_tmp, tmp_rot);
					T pred_rot[3];
					ceres::AngleAxisRotatePoint(tool2cam_R, tmp_rot, pred_rot);

					T gt[3];
					ceres::AngleAxisRotatePoint(target_r, xaxis, gt);

					residuals[3] = (gt[0] - tmp_rot[0]);
					residuals[4] = (gt[1] - tmp_rot[1]);
					residuals[5] = (gt[2] - tmp_rot[2]);


					T yaxis[3];
					yaxis[0] = T(0);
					yaxis[1] = T(1);
					yaxis[2] = T(0);
					ceres::AngleAxisRotatePoint(target2base_R, yaxis, tmp_tmp);
					ceres::AngleAxisRotatePoint(r, tmp_tmp, tmp_rot);
					ceres::AngleAxisRotatePoint(tool2cam_R, tmp_rot, pred_rot);
					ceres::AngleAxisRotatePoint(target_r, yaxis, gt);

					residuals[6] = (gt[0] - tmp_rot[0]);
					residuals[7] = (gt[1] - tmp_rot[1]);
					residuals[8] = (gt[2] - tmp_rot[2]);

						// std::cout << "/* residuals x */" << residuals[2*iPt + 0]<< std::endl;
        return true;
			}
			static ceres::CostFunction* Create(const double* R_base2tool, const double* T_base2tool, const double* R_target2cam, const double* T_target2cam) {
				 return (new ceres::AutoDiffCostFunction<CameraRobotArmMatchError, 9, 3, 3, 3, 3>(
						 new CameraRobotArmMatchError(R_base2tool, T_base2tool, R_target2cam, T_target2cam)));
 }

 const double* R_base2tool;
 const double* T_base2tool;
 const double* R_target2cam;
 const double* T_target2cam;
};



void VisualPath::calibrate_cam_robot(){
	/* code */
	cout << endl << "=== Solving Camera-RobotArm Bundle Adjustment Problem with CERES" << endl;

	 // Set up the problem
	 ceres::Problem problem;

	 std::vector<double*> r_list(numOfPose,NULL);
	 std::vector<double*> t_list(numOfPose,NULL);

	 std::vector<double*> cam_r_list(numOfPose,NULL);
	 std::vector<double*> cam_t_list(numOfPose,NULL);

	 for (size_t i = 0; i < numOfPose; i++) {
		 /* code */
			// 	ceres::LossFunction* loss_function(new ceres:: CauchyLoss(1.0));
			r_list[i] = new double[3];
			t_list[i] = new double[3];
			double* r_ = r_list[i];
			double* t_ = t_list[i];

			cv::Mat transpose;
			cv::Rodrigues(R_TCP_list[i],transpose);
			r_[0] = transpose.at<double>(0);
			r_[1] = transpose.at<double>(1);
			r_[2] = transpose.at<double>(2);

			t_[0] = T_TCP_list[i].at<double>(0);
			t_[1] = T_TCP_list[i].at<double>(1);
			t_[2] = T_TCP_list[i].at<double>(2);

			//camera

			cam_r_list[i] = new double[3];
			cam_t_list[i] = new double[3];
			r_ = cam_r_list[i];
			t_ = cam_t_list[i];
			cv::Rodrigues(R_list[i],transpose);
			r_[0] = transpose.at<double>(0);
			r_[1] = transpose.at<double>(1);
			r_[2] = transpose.at<double>(2);

			t_[0] = T_list[i].at<double>(0);
			t_[1] = T_list[i].at<double>(1);
			t_[2] = T_list[i].at<double>(2);

	 }

	 cerr << endl << "=== cam & robot rt pose done: " << endl;

	//  CameraRobotArmMatchError( const double* R_base2tool, const double* T_base2tool, const double* R_target2cam, const double* T_target2cam)

	double tool2cam_R[3];
	double tool2cam_T[3] = {0};
	double target2base_R[3];
	double target2base_T[3] = {0};

	cv::Mat initguess_target2base_R = cv::Mat::eye(3,3, CV_64FC1);
	cv::Mat initguess_tool2cam_R = cv::Mat::eye(3,3, CV_64FC1);
	initguess_tool2cam_R.at<double>(1,1) = -1;
	initguess_tool2cam_R.at<double>(2,2) = -1;

	cv::Mat tmp;

	cv::Rodrigues(initguess_target2base_R,tmp);
	target2base_R[0] = tmp.at<double>(0);
	target2base_R[1] = tmp.at<double>(1);
	target2base_R[2] = tmp.at<double>(2);

	cv::Rodrigues(initguess_tool2cam_R,tmp);
	tool2cam_R[0] = tmp.at<double>(0);
	tool2cam_R[1] = tmp.at<double>(1);
	tool2cam_R[2] = tmp.at<double>(2);


	 problem.AddParameterBlock(tool2cam_R, 3);
	 problem.AddParameterBlock(tool2cam_T, 3);
	 problem.AddParameterBlock(target2base_R, 3);
	 problem.AddParameterBlock(target2base_T, 3);

	 //do we set the first pose as constant???
	//  problem.SetParameterBlockConstant(r_list[0]);
	//  problem.SetParameterBlockConstant(t_list[0]);

	 for (size_t i = 0; i < numOfPose; i++) {
					/* num of Observation */
					ceres::CostFunction* cost_function =
						 CameraRobotArmMatchError::Create(r_list[i],
							 																t_list[i],
																							cam_r_list[i],
																							cam_t_list[i]
																			 );
			//  ceres::LossFunction* loss_function(new ceres:: CauchyLoss(1.0));
			 ceres::LossFunction* loss_function(new ceres:: HuberLoss(1.0));

	// std::cout << "/* message */" << std::endl;
				 problem.AddResidualBlock(cost_function,
																	NULL /* squared loss */,
																	target2base_R,
																	target2base_T,
																	tool2cam_R,
																	tool2cam_T
																	);
		 }

		std::cout << "/* run BA */" << std::endl;

		ceres::Solver::Options options;
		options.function_tolerance = 1e-9;
		options.max_num_iterations = 50;
		options.linear_solver_type = ceres::DENSE_SCHUR;
		options.minimizer_progress_to_stdout = true;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		std::cout << summary.FullReport() << "\n";
		// cout<<observations[0]<<endl;

		std::cout << "/* returning BA result... */" << std::endl;

		cv::Mat ret = cv::Mat::zeros(3,1,CV_64FC1);
		ret.at<double>(0) = target2base_R[0];
		ret.at<double>(1) = target2base_R[1];
		ret.at<double>(2) = target2base_R[2];

		cv::Rodrigues(ret,global_R);
		std::cout<<"Global T:\n caliberate rot: "<<global_R<<std::endl;

		for (size_t i = 0; i < 3; i++) {
			std::cout<<"  caliberate T: " << target2base_T[i] <<"\n";
			/* code */
		}

		global_T.at<double>(0) = target2base_T[0];
		global_T.at<double>(1) = target2base_T[1];
		global_T.at<double>(2) = target2base_T[2];


		ret = cv::Mat::zeros(3,1,CV_64FC1);
		ret.at<double>(0) = tool2cam_R[0];
		ret.at<double>(1) = tool2cam_R[1];
		ret.at<double>(2) = tool2cam_R[2];
				cv::Rodrigues(ret,local_R);
				std::cout<<"Local T:\n calibrate rot: "<<local_R<<std::endl;

				for (size_t i = 0; i < 3; i++) {
					std::cout<<"  caliberate T: " << tool2cam_T[i] <<"\n";
					local_T.at<double>(i) = tool2cam_T[i];
						/* code */
				}

		return ;

}



//////////////////////
//Optimize intrinsic and cam to imu transformation
//////////////////////

struct ChessboardRobotArmIntrinsicReprojectionError{
    ChessboardRobotArmIntrinsicReprojectionError(vector<cv::Point2f> observedPoints, std::vector<cv::Point3f> chessboardModel, const double* extrinsicsR,
		     const double* extrinsicsT)
    : observedPoints(observedPoints),chessboardModel(chessboardModel), extrinsicsR(extrinsicsR), extrinsicsT(extrinsicsT) {}
    template <typename T>

    bool operator()(
        // const T* const extrinsicsR,
        // const T* const extrinsicsT,
        const T* const targetR,
        const T* const targetT,
				const T* const local_R,
				const T* const local_T,
				const T* const intrinsic,
				const T* const scales,
        T* residuals
        ) const {
        // === (1) Find the positions of the chessboard points in world coordinates

        // Generate the reference points from chessboardModel (1-D)

        // Transform the reference points
        vector<T*> worldCoords;
        for (int iPt = 0; iPt < chessboardModel.size(); iPt++) {
            T refPt[3];
            refPt[0] = T(chessboardModel[iPt].x);
            refPt[1] = T(chessboardModel[iPt].y);
            refPt[2] = T(chessboardModel[iPt].z);

            T rotPt[3];
            ceres::AngleAxisRotatePoint(targetR, refPt, rotPt);

            T* transPt = new T[3];
            transPt[0] = rotPt[0]*scales[0] + targetT[0];
            transPt[1] = rotPt[1]*scales[1] + targetT[1];
            transPt[2] = rotPt[2]*scales[2] + targetT[2];

            worldCoords.push_back(transPt);
        }
        // === (2) Project each point to find the reprojection error
        for (int iPt = 0; iPt < worldCoords.size(); iPt++) {

            T* point = worldCoords[iPt];

            // Rotate from the camera's frame
            T revRot[3];

            for (int i = 0; i < 3; i++) { revRot[i] = T(extrinsicsR[i]); }

            T p_[3];
            ceres::AngleAxisRotatePoint(revRot, point, p_);

            // Now translate point from world frame to robot frame
            p_[0] += T(extrinsicsT[0]);
            p_[1] += T(extrinsicsT[1]);
            p_[2] += T(extrinsicsT[2]);

						//Transform point from robot frame cord to camera frame
						T p[3];
						ceres::AngleAxisRotatePoint(local_R,p_,p);
						p[0] += local_T[0];
						p[1] += local_T[1];
						p[2] += local_T[2];

            // Finally, apply the projection equation
            T x1 = p[0] / p[2];
            T y1 = p[1] / p[2];

						T focus_X = intrinsic[0];
						T focus_Y = intrinsic[1];
						T pptx = intrinsic[2];
						T ppty = intrinsic[3];

            T predicted_x = x1*intrinsic[0] + intrinsic[2];
            T predicted_y = y1*intrinsic[1] + intrinsic[3];


            // The error is the difference between the predicted and observed position.
            residuals[2*iPt + 0] = predicted_x - T(observedPoints[iPt].x);
            residuals[2*iPt + 1] = predicted_y - T(observedPoints[iPt].y);
						// std::cout << "/* residuals x */" << residuals[2*iPt + 0]<< std::endl;
            delete[] worldCoords[iPt];
        }
        return true;
			}
			static ceres::CostFunction* Create(const vector<cv::Point2f> observedPts,
				 const vector<cv::Point3f> chessboardModel, const double* extrinsicsR,
		 		    const double* extrinsicsT ) {
				 return (new ceres::AutoDiffCostFunction<ChessboardRobotArmIntrinsicReprojectionError, 5*8*2, 3, 3, 3, 3, 4, 3>(
						 new ChessboardRobotArmIntrinsicReprojectionError(observedPts, chessboardModel, extrinsicsR, extrinsicsT)));
 }

 vector<cv::Point2f> observedPoints;
 vector<cv::Point3f> chessboardModel;
 // double focus_X, focus_Y, pptx, ppty;
 const double* extrinsicsR;
 const double* extrinsicsT;
};


void VisualPath::chessboardRobotArmIntrinsicBundleAdjustment() {
	/* code */
	cout << endl << "=== Solving Chessboard-RobotArm Bundle Adjustment Problem with CERES" << endl;
	cout<<"local_R: " << local_R <<"\n";
	cout<<"local_T: " << local_T <<"\n";
	 // Define the cost function

	 // Set up the problem
	 ceres::Problem problem;

   std::vector<double*> r_list(numOfPose,NULL);
   std::vector<double*> t_list(numOfPose,NULL);

	 cv::Mat local_R_Rodrigues;
	 double * local_R_BA = new double[3];
	 double * local_T_BA = new double[3];

	 cv::Rodrigues(chessboard_R,local_R_Rodrigues);
	 local_R_BA[0] = local_R_Rodrigues.at<double>(0);
	 local_R_BA[1] = local_R_Rodrigues.at<double>(1);
	 local_R_BA[2] = local_R_Rodrigues.at<double>(2);

	 local_T_BA[0] = chessboard_T.at<double>(0);
	 local_T_BA[1] = chessboard_T.at<double>(1);
	 local_T_BA[2] = chessboard_T.at<double>(2);
	 cerr << endl << "=== chessboard pose done: " << endl;

	 problem.AddParameterBlock(local_R_BA, 3);
	 problem.AddParameterBlock(local_T_BA, 3);

   for (size_t i = 0; i < numOfPose; i++) {
     /* code */
      // 	ceres::LossFunction* loss_function(new ceres:: CauchyLoss(1.0));
      r_list[i] = new double[3];
      t_list[i] = new double[3];
      double* r_ = r_list[i];
      double* t_ = t_list[i];

			cv::Mat transpose;
			cv::Rodrigues(R_TCP_list[i],transpose);
      r_[0] = transpose.at<double>(0);
      r_[1] = transpose.at<double>(1);
      r_[2] = transpose.at<double>(2);

      t_[0] = T_TCP_list[i].at<double>(0);
      t_[1] = T_TCP_list[i].at<double>(1);
      t_[2] = T_TCP_list[i].at<double>(2);

				// problem.AddParameterBlock(r_list[i], 3);
				// problem.AddParameterBlock(t_list[i], 3);
   }

   cerr << endl << "=== cam rt pose done: " << endl;

	 //set camera_robotarm_rotation & translation
	 cv::Mat robot_R_Rodrigues;
	 double * robot_R_BA = new double[3];
	 double * robot_T_BA = new double[3];

	 cv::Rodrigues(local_R,robot_R_Rodrigues);
	 robot_R_BA[0] = robot_R_Rodrigues.at<double>(0);
	 robot_R_BA[1] = robot_R_Rodrigues.at<double>(1);
	 robot_R_BA[2] = robot_R_Rodrigues.at<double>(2);

	 robot_T_BA[0] = local_T.at<double>(0);
	 robot_T_BA[1] = local_T.at<double>(1);
	 robot_T_BA[2] = local_T.at<double>(2);
	 cerr << endl << "=== robot->cam pose done: " << endl;

	 problem.AddParameterBlock(robot_R_BA, 3);
	 problem.AddParameterBlock(robot_T_BA, 3);

	 //add intrinsic parameters

	 double * intrinsic_parameters = new double[4];
	 intrinsic_parameters[0] = Camera_Matrix.at<double>(0,0);
	 intrinsic_parameters[1] = Camera_Matrix.at<double>(1,1);
	 intrinsic_parameters[2] = Camera_Matrix.at<double>(0,2);
	 intrinsic_parameters[3] = Camera_Matrix.at<double>(1,2);

	 //do we set the first pose as constant???
	//  problem.SetParameterBlockConstant(r_list[0]);
  //  problem.SetParameterBlockConstant(t_list[0]);

   for (size_t i = 0; i < numOfPose; i++) {
	        /* num of Observation */
	        ceres::CostFunction* cost_function =
	           ChessboardRobotArmIntrinsicReprojectionError::Create(observedControlPts[i][0],
							 														actualControlPts[0],
																				 r_list[i],
																				 t_list[i]
																			 );
			//  ceres::LossFunction* loss_function(new ceres:: CauchyLoss(1.0));
			 ceres::LossFunction* loss_function(new ceres:: HuberLoss(1.0));

	// std::cout << "/* message */" << std::endl;
	       problem.AddResidualBlock(cost_function,
	                                NULL /* squared loss */,
																	local_R_BA,
																	local_T_BA,
																	robot_R_BA,
																	robot_T_BA,
																	intrinsic_parameters,
																	scales
	                                );
	   }

	  std::cout << "/* run BA */" << std::endl;

	  ceres::Solver::Options options;
		options.function_tolerance = 1e-9;
	  options.max_num_iterations = 50;
	  options.linear_solver_type = ceres::DENSE_SCHUR;
	  options.minimizer_progress_to_stdout = true;
	  ceres::Solver::Summary summary;
	  ceres::Solve(options, &problem, &summary);
	  std::cout << summary.FullReport() << "\n";
		// cout<<observations[0]<<endl;

	  std::cout << "/* returning BA result... */" << std::endl;

		cv::Mat chessboard_R_ret = cv::Mat::zeros(3,1,CV_64FC1);
		chessboard_R_ret.at<double>(0) = local_R_BA[0];
		chessboard_R_ret.at<double>(1) = local_R_BA[1];
		chessboard_R_ret.at<double>(2) = local_R_BA[2];

		cv::Rodrigues(chessboard_R_ret,chessboard_R);

		chessboard_T.at<double>(0) = local_T_BA[0];
		chessboard_T.at<double>(1) = local_T_BA[1];
		chessboard_T.at<double>(2) = local_T_BA[2];

		cv::Mat robot_R_ret = cv::Mat::zeros(3,1,CV_64FC1);
		robot_R_ret.at<double>(0) = robot_R_BA[0];
		robot_R_ret.at<double>(1) = robot_R_BA[1];
		robot_R_ret.at<double>(2) = robot_R_BA[2];

		cv::Rodrigues(robot_R_ret, local_R);

		local_T.at<double>(0) = robot_T_BA[0];
		local_T.at<double>(1) = robot_T_BA[1];
		local_T.at<double>(2) = robot_T_BA[2];

	  //
	  for (size_t i = 0; i < numOfPose; i++) {
	     double* R = new double[9];
	     ceres::AngleAxisToRotationMatrix<double>(r_list[i],R);
	     cv::Mat R_( 3, 3, CV_64FC1, R );
	     //transpose....????????
	     R_ = R_.t();
	     R_.copyTo(R_TCP_list[i]);

	     cv::Mat ptr = T_list[i];
	     double* p = t_list[i];
	     T_TCP_list[i].at<double>(0,0) = p[0];
	     T_TCP_list[i].at<double>(1,0) = p[1];
	     T_TCP_list[i].at<double>(2,0) = p[2];
	  }

		std::cerr << "orginal intrinsics: " << Camera_Matrix <<"\n";
		//return intrinsic parameters;
		 Camera_Matrix.at<double>(0,0) = intrinsic_parameters[0];
		 Camera_Matrix.at<double>(1,1) = intrinsic_parameters[1];
		 Camera_Matrix.at<double>(0,2) = intrinsic_parameters[2];
		 Camera_Matrix.at<double>(1,2) = intrinsic_parameters[3];

			std::cerr<<" intrinsic: " << Camera_Matrix <<"\n";

			for (size_t i = 0; i < 3; i++) {
				/* code */
				cerr<<"scales: " << scales[i] <<"\n";
			}

	  return ;

}

void VisualPath::computeCameraPoseDifference(){
	cerr<<"\nstart computing difference...\n";
	double degree_max,degree_min,trans_max,trans_min;
	degree_max = degree_min = trans_min = trans_max = 0;
	for (size_t i = 0; i < numOfPose; i++) {
		/* code */
		cv::Mat predict_cam_rot = local_R*R_TCP_list[i]*global_R;
		cv::Mat predict_cam_trans = local_R*(R_TCP_list[i]*global_T + T_TCP_list[i])+local_T;
		cv::Mat tmp_pred, tmp_actual;
		cv::Rodrigues(predict_cam_rot,tmp_pred);
		cv::Rodrigues(R_list[i],tmp_actual);
		cv::Mat r_diff = (tmp_pred - tmp_actual)*180/3.1415;
		for(int i = 0; i < 3; i++)
			if( r_diff.at<double>(i) < 0) r_diff.at<double>(i) += 180;
		cerr<<"Pose "<<i<<"\n";
		cerr<<"rot difference: [*degree]\n"<< r_diff <<"\n";
		double min, max;
		cv::minMaxLoc(r_diff, &min, &max);
		if ( max > degree_max) {
				degree_max = max;
		}else if(min <  degree_min){
			degree_min = min;
		}
		cerr<<"trans difference: [*cm]\n"<< (predict_cam_trans - T_list[i])*100 <<"\n\n";
		cv::minMaxLoc((predict_cam_trans - T_list[i])*100, &min, &max);
		if (max > trans_max) {
			/* code */
			trans_max = max;
		}else if(min < trans_min){
			trans_min = min;
		}
	}
	cerr<<" translation max: "<< trans_max <<"\n translation min: "<<trans_min <<"\n rotation max: " << degree_max <<"\n rotation min: " << degree_min<<"\n";

}







#endif
