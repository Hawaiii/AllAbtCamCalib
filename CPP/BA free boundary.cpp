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
    ChessboardRobotArmReprojectionError(vector<cv::Point2f> observedPoints, std::vector<cv::Point3f> chessboardModel, double focus_X, double focus_Y, double pptx, double ppty)
    : observedPoints(observedPoints),chessboardModel(chessboardModel), focus_X(focus_X), focus_Y(focus_Y), pptx(pptx), ppty(ppty) {}
    template <typename T>

    bool operator()(
        const T* const extrinsicsR,
        const T* const extrinsicsT,
        const T* const targetR,
        const T* const targetT,
				const T* const local_R,
				const T* const local_T,
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

            T p_[3];
            ceres::AngleAxisRotatePoint(revRot, point, p_);

            // Now translate from the camera's frame
            p_[0] += extrinsicsT[0];
            p_[1] += extrinsicsT[1];
            p_[2] += extrinsicsT[2];

						//Transform from camera cord to world frame
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
				 const vector<cv::Point3f> chessboardModel, const double focus_X, const double focus_Y, const double pptx, const double ppty) {

				 return (new ceres::AutoDiffCostFunction<ChessboardRobotArmReprojectionError, 5*8*2, 3, 3, 3, 3, 3, 3>(
						 new ChessboardRobotArmReprojectionError(observedPts, chessboardModel, focus_X, focus_Y, pptx, ppty)));
 }

 vector<cv::Point2f> observedPoints;
 vector<cv::Point3f> chessboardModel;
 double focus_X, focus_Y, pptx, ppty;
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
      R_TCP_list[i].copyTo(transpose);
      transpose = transpose.t();
      ceres::RotationMatrixToAngleAxis(reinterpret_cast<double*>(transpose.data),r_);

      t_[0] = T_TCP_list[i].at<double>(0);
      t_[1] = T_TCP_list[i].at<double>(1);
      t_[2] = T_TCP_list[i].at<double>(2);

				problem.AddParameterBlock(r_list[i], 3);
				problem.AddParameterBlock(t_list[i], 3);
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

	 problem.SetParameterBlockConstant(r_list[0]);
   problem.SetParameterBlockConstant(t_list[0]);

   for (size_t i = 0; i < numOfPose; i++) {
	        /* num of Observation */
	        ceres::CostFunction* cost_function =
	           ChessboardRobotArmReprojectionError::Create(observedControlPts[i][0],
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
																	local_T_BA,
																	robot_R_BA,
																	robot_T_BA
	                                );
	   }

	  std::cout << "/* run BA */" << std::endl;

	  ceres::Solver::Options options;
		options.function_tolerance = 1e-9;
	  options.max_num_iterations = 100;
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

	  return ;

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

      t_[0] = T_list[i].at<double>(0)+0.1;
      t_[1] = T_list[i].at<double>(1)+0.1;
      t_[2] = T_list[i].at<double>(2)+0.1;

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
			transPt.x = rotPt[0] + chessboard_T.at<double>(0);
			transPt.y = rotPt[1] + chessboard_T.at<double>(1);
			transPt.z = rotPt[2] + chessboard_T.at<double>(2);

			p3d.push_back(transPt);
			delete[] local_R_BA;
	}
  std::cerr<<"generate chessboard done!" << std::endl;
}

#endif
