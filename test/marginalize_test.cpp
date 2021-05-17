
//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

//Sophus
#include "sophus/se3.h"

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

//G2O
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

//Std
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>

using namespace std;





//Load the pose of the first camera.
void LoadCameraPose1(cv::Mat & mTrueRotation1,cv::Mat & mTrueRotationVector1, cv::Mat & mTrueTranslation1){
	mTrueRotation1 = (cv::Mat_<double>(3 , 3) << 1.0 , 0.0  , 0.0 ,
											 0.0   , 1.0, 0.0 ,
											 0.0   , 0.0  ,  1.0);


	mTrueTranslation1 = (cv::Mat_<double>(3 , 1) << -1.0 , 0.0  , 0.0);

	cv::Rodrigues(mTrueRotation1, mTrueRotationVector1);

}


//Load the pose of the second camera.
void LoadCameraPose2(cv::Mat & mTrueRotation2,cv::Mat & mTrueRotationVector2, cv::Mat & mTrueTranslation2){
	mTrueRotation2 = (cv::Mat_<double>(3 , 3) << 1.0 , 0.0  , 0.0 ,
											 0.0   , 1.0, 0.0 ,
											 0.0   , 0.0  ,  1.0);


	mTrueTranslation2 = (cv::Mat_<double>(3 , 1) << 2.0 , 0.0  , 0.0);


	cv::Rodrigues(mTrueRotation2, mTrueRotationVector2);
}


Eigen::Vector3d GetCameraPoint3D(Sophus::SE3 mPose, cv::Point3d iPointWorld){
	

	Eigen::Vector3d mPointWorld(iPointWorld.x, iPointWorld.y, iPointWorld.z);
	
	Eigen::Matrix4d mPoseMatrix = mPose.matrix();
	//Secondly convert the world coordinate of the point to camera coordinate.
	Eigen::Vector4d mHomogeneousPointWorld(1 , 1 , 1 , 1);
	mHomogeneousPointWorld.segment(0, 3) = mPointWorld;
	Eigen::Vector4d mHomogeneousPointCamera = mPoseMatrix * mHomogeneousPointWorld;
	//Normalize.
	mHomogeneousPointCamera /= mHomogeneousPointCamera(3);
	return mHomogeneousPointCamera.segment(0, 3);
}


void ComputeHessianMatrix(vector<cv::Point3f> gPoints, Sophus::SE3 mPose1, Sophus::SE3 mPose2,
							cv::Mat mK1, cv::Mat mK2){
	

	double nScale = 1.0;

	//Compute the Hessian matrix.
	int nMapPointSize = gPoints.size();
	int nPoseSize = 1;

	Eigen::MatrixXd mHessian = Eigen::MatrixXd(
		6  + 3 *  nMapPointSize,
		6  + 3 *  nMapPointSize);

	mHessian = mHessian * 0.0;

	//Project on the second frame
	for (int i=0;i<nMapPointSize;i++){
		// 	each jacobian is 2 * (6*posesize+3*pointsize)
		Eigen::MatrixXd mJacobian = Eigen::MatrixXd(2 , 6 + 3 *  nMapPointSize);

		mJacobian = mJacobian * (double)0.0;

		Eigen::MatrixXd mJacobianToPose = Eigen::MatrixXd(2 , 6);
		Eigen::MatrixXd mJacobianToPoint = Eigen::MatrixXd(2 , 3);


		Eigen::Vector3d mPointCamera = GetCameraPoint3D(mPose2, gPoints[i]);
		//Intrinsics.
		double nFx, nFy, nCx, nCy;
		nFx = mK2.at<double>(0 , 0);
		nFy = mK2.at<double>(1 , 1);
		nCx = mK2.at<double>(0 , 2);
		nCx = mK2.at<double>(1 , 2);

		double nX = mPointCamera(0);
		double nY = mPointCamera(1);
		double nZ = mPointCamera(2);

		mJacobianToPose(0 , 0) = nFx/nZ;
		mJacobianToPose(0 , 1) = 0.0;
		mJacobianToPose(0 , 2) = -nFx*nX/(nZ*nZ);
		mJacobianToPose(0 , 3) = -nFx*nX*nY/(nZ*nZ);
		mJacobianToPose(0 , 4) = nFx + (nFx * nX * nX / (nZ * nZ));
		mJacobianToPose(0 , 5) = -nFx * nY / nZ;
		//The second row of the jacobian.
		mJacobianToPose(1 , 0) = 0.0;
		mJacobianToPose(1 , 1) = nFy/nZ;
		mJacobianToPose(1 , 2) = -nFy*nY/(nZ*nZ);
		mJacobianToPose(1 , 3) = -nFy-(nFy*nY*nY/(nZ*nZ));
		mJacobianToPose(1 , 4) = nFy*nX*nY/(nZ*nZ);
		mJacobianToPose(1 , 5) = nFy*nX/nZ;

		mJacobianToPose = -mJacobianToPose;


		Eigen::MatrixXd mJacobianToCameraPoint(2 , 3);
		mJacobianToCameraPoint(0 , 0) = nFx/nZ;
		mJacobianToCameraPoint(0 , 1) = 0.0;
		mJacobianToCameraPoint(0 , 2) = -nFx*nX/(nZ*nZ);
		
		mJacobianToCameraPoint(1 , 0) = 0.0;
		mJacobianToCameraPoint(1 , 1) = nFy/nZ;
		mJacobianToCameraPoint(1 , 2) = -nFy*nY/(nZ*nZ);

		//Get the rotation matrix.
		Eigen::Matrix3d mRotation = mPose2.rotation_matrix();
		
		//Get the jacobian to camera point.
		mJacobianToPoint = -mJacobianToCameraPoint *  mRotation;

		mJacobian.block(0 , 0 , 2 , 6) = mJacobianToPose;
		mJacobian.block(0 , 6+i*3, 2 , 3) = mJacobianToPoint;

		Eigen::MatrixXd mLocalHessian = mJacobian.transpose() * mJacobian;

		mHessian = mHessian + mLocalHessian;
	}

	//Project on the first frame
	for (int i=0;i<nMapPointSize;i++){
		// 	each jacobian is 2 * (6*posesize+3*pointsize)
		Eigen::MatrixXd mJacobian = Eigen::MatrixXd(2 , 6 + 3 *  nMapPointSize);

		mJacobian = mJacobian * (double)0.0;

		Eigen::MatrixXd mJacobianToPoint = Eigen::MatrixXd(2 , 3);


		Eigen::Vector3d mPointCamera = GetCameraPoint3D(mPose1, gPoints[i]);
		//Intrinsics.
		double nFx, nFy, nCx, nCy;
		nFx = mK1.at<double>(0 , 0);
		nFy = mK1.at<double>(1 , 1);
		nCx = mK1.at<double>(0 , 2);
		nCx = mK1.at<double>(1 , 2);

		double nX = mPointCamera(0);
		double nY = mPointCamera(1);
		double nZ = mPointCamera(2);

	

		Eigen::MatrixXd mJacobianToCameraPoint(2 , 3);
		mJacobianToCameraPoint(0 , 0) = nFx/nZ;
		mJacobianToCameraPoint(0 , 1) = 0.0;
		mJacobianToCameraPoint(0 , 2) = -nFx*nX/(nZ*nZ);
		
		mJacobianToCameraPoint(1 , 0) = 0.0;
		mJacobianToCameraPoint(1 , 1) = nFy/nZ;
		mJacobianToCameraPoint(1 , 2) = -nFy*nY/(nZ*nZ);

		//Get the rotation matrix.
		Eigen::Matrix3d mRotation = mPose1.rotation_matrix();
		
		//Get the jacobian to camera point.
		mJacobianToPoint = -mJacobianToCameraPoint *  mRotation;

		mJacobian.block(0 , 6+i*3, 2 , 3) = mJacobianToPoint;
		
		Eigen::MatrixXd mLocalHessian = mJacobian.transpose() * mJacobian;

		mHessian = mHessian + mLocalHessian;
		// mHessian = mHessian + mJacobian.transpose() * mJacobian;
	}



	mHessian =  mHessian/1000000;


	// cout << "Hessian determinant is: " << mHessian.determinant() << endl;

	//Compute the inverse of ll
	Eigen::MatrixXd mHessianPP = mHessian.block(0 , 0 , 6 , 6);
	Eigen::MatrixXd mHessianPL = mHessian.block(0 , 6 , 6 , nMapPointSize * 3);
	Eigen::MatrixXd mHessianLL = mHessian.block(6 , 6 , nMapPointSize * 3 , nMapPointSize * 3);

		  


	Eigen::MatrixXd mHessianLLInverse = Eigen::MatrixXd(nMapPointSize * 3, nMapPointSize * 3);
	for (int i=0;i<nMapPointSize;i++){
		Eigen::MatrixXd mLocalHessianLL = mHessianLL.block(i*3 , i*3, 3, 3);
		// cout << "Local hessian1 is: " << endl << mLocalHessianLL << endl;
		// cout << "Det is: " << mLocalHessianLL.determinant() << endl
		Eigen::MatrixXd mLocalHessianLLInverse = mLocalHessianLL.inverse();
		// cout << "Local Hessian is: " << endl << mLocalHessianLLInverse << endl;
		mHessianLLInverse.block(i*3, i*3 , 3 , 3) = mLocalHessianLLInverse;
	}
	Eigen::MatrixXd mAdditionalInfo = mHessianPL * mHessianLLInverse * mHessianPL.transpose();
	cout << "Hessian PP is: " << endl << mHessianPP << endl; 
	cout << "Additional Info is: " << endl << mAdditionalInfo << endl; 

	Eigen::MatrixXd mMarginalizedHessian = mHessianPP - mAdditionalInfo;



	// cout << "Hessian is: " << endl << mHessian << endl;
	cout << "mMarginalizedHessian is: " << endl << mMarginalizedHessian << endl;
	cout << "Det is: " << mMarginalizedHessian.determinant() << endl;

	Eigen::EigenSolver<Eigen::MatrixXd> iSolver(mMarginalizedHessian);
	

	cout << "Eigen values: " << endl << iSolver.eigenvalues() << endl;

}








int main(){

//Firstly generate 1000 points
	
	vector<cv::Point3f> gPoints;

	for (int i=-5; i<5; i++){
		for (int k=-5; k<5;k++){
			for (int j=1; j< 4; j++){
				double nX = (double)(i);
				double nY = (double)(k);
				double nZ = (double)(j)*(3.3);
				cv::Point3f iPoint(nX, nY, nZ);
				gPoints.push_back(iPoint);
			}
		}
	}




	//Define 2 cameras
	cv::Mat mK1 = (cv::Mat_<double>(3 , 3) << 400.0 , 0.0  , 0.0 ,
											 0.0   , 400.0, 0.0 ,
											 0.0   , 0.0  ,  1.0);
	cv::Mat mD1 = (cv::Mat_<double>(4 , 1) << 0.0 , 0.0 , 0.0 , 0.0);


	cv::Mat mK2 = (cv::Mat_<double>(3 , 3) << 400.0 , 0.0  , 0.0 ,
											 0.0   , 400.0, 0.0 ,
											 0.0   , 0.0  ,  1.0);
	cv::Mat mD2 = (cv::Mat_<double>(4 , 1) << 0.0 , 0.0 , 0.0 , 0.0);



	//Define the pose of 2 cameras.
	cv::Mat mRotation1, mTranslation1, mRotationVector1;
	cv::Mat mRotation2, mTranslation2, mRotationVector2;

	LoadCameraPose1(mRotation1, mRotationVector1, mTranslation1);
	LoadCameraPose2(mRotation2, mRotationVector2, mTranslation2);

	vector<cv::Point2f> gImagingPoints1, gImagingPoints2;
	cv::projectPoints(gPoints, mRotationVector1, mTranslation1, mK1, mD1, gImagingPoints1);
	cv::projectPoints(gPoints, mRotationVector2, mTranslation2, mK2, mD2, gImagingPoints2);
	
	Eigen::Matrix3d mRotation1GT, mRotation2GT;
	Eigen::Vector3d mTranslation1GT, mTranslation2GT;
	cv::cv2eigen(mRotation1, mRotation1GT);
	cv::cv2eigen(mRotation2, mRotation2GT);

	cv::cv2eigen(mTranslation1, mTranslation1GT);
	cv::cv2eigen(mTranslation2, mTranslation2GT);


	mTranslation2.at<double>(0 , 0) = mTranslation2.at<double>(0 , 0) + 1.2;

	Eigen::Matrix3d mRotation1Eigen, mRotation2Eigen;
	Eigen::Vector3d mTranslation1Eigen, mTranslation2Eigen;
	cv::cv2eigen(mRotation1, mRotation1Eigen);
	cv::cv2eigen(mRotation2, mRotation2Eigen);

	cv::cv2eigen(mTranslation1, mTranslation1Eigen);
	cv::cv2eigen(mTranslation2, mTranslation2Eigen);

	
	Sophus::SE3 mPose1GT = Sophus::SE3(mRotation1GT, mTranslation1GT);
	Sophus::SE3 mPose2GT = Sophus::SE3(mRotation2GT, mTranslation2GT);

	Sophus::SE3 mPose1 = Sophus::SE3(mRotation1Eigen, mTranslation1Eigen);
	Sophus::SE3 mPose2 = Sophus::SE3(mRotation2Eigen, mTranslation2Eigen);



	vector<Sophus::SE3> gGroundTruthPoses = {mPose1GT, mPose2GT};

	vector<Sophus::SE3> gPoses = {mPose1, mPose2};


	ComputeHessianMatrix(gPoints, mPose1, mPose2, mK1, mK2);


	return 0;
}


