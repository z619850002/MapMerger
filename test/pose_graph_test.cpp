
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

#include <iostream>

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


	mTrueTranslation2 = (cv::Mat_<double>(3 , 1) << 0.0 , 0.0  , 0.0);


	cv::Rodrigues(mTrueRotation2, mTrueRotationVector2);
}




//Load the pose of the second camera.
void LoadCameraPose3(cv::Mat & mTrueRotation3,cv::Mat & mTrueRotationVector3, cv::Mat & mTrueTranslation3){
	mTrueRotation3 = (cv::Mat_<double>(3 , 3) << 1.0 , 0.0  , 0.0 ,
											 0.0   , 1.0, 0.0 ,
											 0.0   , 0.0  ,  1.0);


	mTrueTranslation3 = (cv::Mat_<double>(3 , 1) << 2.0 , 0.0  , 0.0);


	cv::Rodrigues(mTrueRotation3, mTrueRotationVector3);
}




double CalculateAbsoluteError(	vector<Sophus::SE3> gGroundTruth, 
								vector<Sophus::SE3> gPoses){
	double nError = 0.0;

	for (int i=0;i<gGroundTruth.size();i++){
		Sophus::SE3 mError = gPoses[i] * gGroundTruth[i].inverse();
		Eigen::Matrix<double, 6, 1> mErrorVector = mError.log();
		double nLocalError = 0.0;
		for (int j=0;j<6;j++){
			nLocalError += mErrorVector(j,0) * mErrorVector(j,0);
		}
		nLocalError = sqrt(nLocalError);
		nError += nLocalError;
	}

	return nError;
}


double ComputeReprojectionError(Sophus::SE3 mPose, cv::Mat mK, cv::Mat mD, 
								vector<cv::Point3f> gPoints3D,
								vector<cv::Point2f> gImagingPoints){
	double nError = 0.0;
	vector<cv::Point2f> gPoints2D;
	gPoints2D.reserve(gImagingPoints.size());

	cv::Mat mRotation, mTranslation;
	cv::eigen2cv(mPose.rotation_matrix(), mRotation);
	cv::eigen2cv(mPose.translation(), mTranslation);

	cv::Mat mRotationVec;
	cv::Rodrigues(mRotation, mRotationVec);


	cv::projectPoints(gPoints3D, mRotationVec, mTranslation, mK, mD, gPoints2D);
	
	for (int i=0;i<gImagingPoints.size();i++){
		cv::Point2f iImagingPoint = gImagingPoints[i];
		cv::Point2f iPoint2D = gPoints2D[i];
		float nLocalError = 0.0;
		nLocalError += (iImagingPoint.x - iPoint2D.x) * (iImagingPoint.x - iPoint2D.x);
		nLocalError += (iImagingPoint.y - iPoint2D.y) * (iImagingPoint.y - iPoint2D.y);
		nLocalError = sqrt(nLocalError);
		nError += nLocalError;
	}

	return nError;
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


Eigen::MatrixXd ComputeHessianMatrix(vector<cv::Point3f> gPoints, Sophus::SE3 mPose1, Sophus::SE3 mPose2,
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

	return mMarginalizedHessian;

}










int main(){

	



	//Construct the optimizer.
    //Construct the optimizer.
    //Now we use Levenberg solver. GN,LM,Dogleg is also avaliable.
    // typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> DirectBlock;  // 求解的向量是6＊1的
    typedef g2o::BlockSolverX DirectBlock;
    std::unique_ptr<DirectBlock::LinearSolverType> pLinearSolver (new g2o::LinearSolverEigen< DirectBlock::PoseMatrixType > ());
    // std::unique_ptr<DirectBlock::LinearSolverType> pLinearSolver (new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ());
    std::unique_ptr<DirectBlock> pSolverPtr (new DirectBlock ( std::move(pLinearSolver) ));
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    g2o::OptimizationAlgorithmLevenberg* pSolver = new g2o::OptimizationAlgorithmLevenberg ( std::move(pSolverPtr) ); // L-M
    // g2o::OptimizationAlgorithmGaussNewton* pSolver = new g2o::OptimizationAlgorithmGaussNewton ( std::move(pSolverPtr) ); // L-M
	//TODO: These configuration should be adjusted in the future.    
	g2o::SparseOptimizer iOptimizer;
    iOptimizer.setAlgorithm ( pSolver );
    iOptimizer.setVerbose( true );


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


	cv::Mat mK3 = (cv::Mat_<double>(3 , 3) << 400.0 , 0.0  , 0.0 ,
											 0.0   , 400.0, 0.0 ,
											 0.0   , 0.0  ,  1.0);
	cv::Mat mD3 = (cv::Mat_<double>(4 , 1) << 0.0 , 0.0 , 0.0 , 0.0);





	//Define the pose of 2 cameras.
	cv::Mat mRotation1, mTranslation1, mRotationVector1;
	cv::Mat mRotation2, mTranslation2, mRotationVector2;
	cv::Mat mRotation3, mTranslation3, mRotationVector3;

	LoadCameraPose1(mRotation1, mRotationVector1, mTranslation1);
	LoadCameraPose2(mRotation2, mRotationVector2, mTranslation2);
	LoadCameraPose3(mRotation3, mRotationVector3, mTranslation3);

	vector<cv::Point2f> gImagingPoints1, gImagingPoints2, gImagingPoints3;
	cv::projectPoints(gPoints, mRotationVector1, mTranslation1, mK1, mD1, gImagingPoints1);
	cv::projectPoints(gPoints, mRotationVector2, mTranslation2, mK2, mD2, gImagingPoints2);
	cv::projectPoints(gPoints, mRotationVector3, mTranslation3, mK3, mD3, gImagingPoints3);

	Eigen::Matrix3d mRotation1GT, mRotation2GT, mRotation3GT;
	Eigen::Vector3d mTranslation1GT, mTranslation2GT, mTranslation3GT;
	cv::cv2eigen(mRotation1, mRotation1GT);
	cv::cv2eigen(mRotation2, mRotation2GT);
	cv::cv2eigen(mRotation3, mRotation3GT);

	cv::cv2eigen(mTranslation1, mTranslation1GT);
	cv::cv2eigen(mTranslation2, mTranslation2GT);
	cv::cv2eigen(mTranslation3, mTranslation3GT);


	mTranslation2.at<double>(0 , 0) = mTranslation2.at<double>(0 , 0) + 1.2;

	Eigen::Matrix3d mRotation1Eigen, mRotation2Eigen, mRotation3Eigen;
	Eigen::Vector3d mTranslation1Eigen, mTranslation2Eigen, mTranslation3Eigen;
	cv::cv2eigen(mRotation1, mRotation1Eigen);
	cv::cv2eigen(mRotation2, mRotation2Eigen);
	cv::cv2eigen(mRotation3, mRotation3Eigen);

	cv::cv2eigen(mTranslation1, mTranslation1Eigen);
	cv::cv2eigen(mTranslation2, mTranslation2Eigen);
	cv::cv2eigen(mTranslation3, mTranslation3Eigen);

	
	Sophus::SE3 mPose1GT = Sophus::SE3(mRotation1GT, mTranslation1GT);
	Sophus::SE3 mPose2GT = Sophus::SE3(mRotation2GT, mTranslation2GT);
	Sophus::SE3 mPose3GT = Sophus::SE3(mRotation3GT, mTranslation3GT);

	Sophus::SE3 mPose1 = Sophus::SE3(mRotation1Eigen, mTranslation1Eigen);
	Sophus::SE3 mPose2 = Sophus::SE3(mRotation2Eigen, mTranslation2Eigen);
	Sophus::SE3 mPose3 = Sophus::SE3(mRotation3Eigen, mTranslation3Eigen);



	vector<Sophus::SE3> gGroundTruthPoses = {mPose1GT, mPose2GT, mPose3GT};

	vector<Sophus::SE3> gPoses = {mPose1, mPose2, mPose3};






    int nVertexId = 3;
    int nVertexSize = 3;

    vector<g2o::VertexSE3Expmap *> gAllVertices;

    for (int i=0;i<nVertexSize;i++){
    	Sophus::SE3 mPose = gPoses[i];



	    g2o::VertexSE3Expmap * pPoseVertex = new g2o::VertexSE3Expmap();
	    pPoseVertex->setId(nVertexId);
	    if (i==0){
	    	pPoseVertex->setFixed(true);
	    }

	    nVertexId++;
	    pPoseVertex->setEstimate( g2o::SE3Quat((mPose).rotation_matrix(),(mPose ).translation()));

	    iOptimizer.addVertex(pPoseVertex);

	    gAllVertices.push_back(pPoseVertex);
    }

    vector<g2o::EdgeSE3Expmap *> gEdges;


	const Eigen::Matrix<double,6,6> mInformation = Eigen::Matrix<double,6,6>::Identity();


    int nEdgeIndex = 200;

    vector<vector<cv::Point2f>> gAllImagingPoints = {gImagingPoints1, gImagingPoints2, gImagingPoints3};




    for (int i=0;i<nVertexSize;i++){
    	for (int j=i+1;j<nVertexSize;j++){



    		Sophus::SE3 mPose1 = gPoses[i];
    		Sophus::SE3 mPose2 = gPoses[j];

    		double nError1 = ComputeReprojectionError(mPose1, mK1, mD1, gPoints, gAllImagingPoints[i]);
    		double nError2 = ComputeReprojectionError(mPose2, mK1, mD1, gPoints, gAllImagingPoints[j]);
    		cout << "i,j are: " << i << " " << j << endl;
    		cout << "Error 1,2 are: " << nError1 << " " << nError2 << endl;

    		// double nRatio = 1.0/ ((nError1 + 1.0) * (nError2 + 1.0));

    		// from->estimate().inverse() * to->estimate()
    		Sophus::SE3 mEstimate = ( mPose2 * mPose1.inverse());

    		g2o::EdgeSE3Expmap * pPoseGraphEdge = new g2o::EdgeSE3Expmap();
		    pPoseGraphEdge->setMeasurement(g2o::SE3Quat(
		        mEstimate.rotation_matrix(),
		        mEstimate.translation()));

		    pPoseGraphEdge->setId( nEdgeIndex++ );
            //Linked with the position of 3d points.
            pPoseGraphEdge->setVertex(0 , gAllVertices[i]);
            pPoseGraphEdge->setVertex(1 , gAllVertices[j]);


			Eigen::MatrixXd mInformationMarginalized = ComputeHessianMatrix(gPoints, mPose1, mPose2, mK1, mK1);            

            // Eigen::Matrix<double, 6, 6> mInformation = Eigen::Matrix<double, 6, 6>::Identity()*2;

            // pPoseGraphEdge->setInformation(mInformation);
			cout << "Information marginalized det" << endl << mInformationMarginalized.determinant() << endl;
            pPoseGraphEdge->setInformation(mInformationMarginalized);
            // cout << "First: " << endl << pPoseGraphEdge->information() << endl;
            gEdges.push_back(pPoseGraphEdge);
    		iOptimizer.addEdge(pPoseGraphEdge);
    	}
    }
  
    int nVertexIndex = nEdgeIndex ++;

    int nCameraParameterId = nVertexIndex;
    nVertexIndex++;
    g2o::CameraParameters * pCameraParameter = new g2o::CameraParameters(
        mK1.at<double>(0 , 0), 
        Eigen::Vector2d(mK1.at<double>(0 , 2) , mK1.at<double>(1 , 2)), 
        0);
    pCameraParameter->setId(nCameraParameterId);
    //Add parameters to the optimizer.
    iOptimizer.addParameter(pCameraParameter);

    for (int i=0;i<gPoints.size();i++){
    	cv::Point3d iPoint = gPoints[i];
        nVertexIndex++;
        g2o::VertexSBAPointXYZ * pPointVertex = new g2o::VertexSBAPointXYZ();
        pPointVertex->setId(nVertexIndex);
        // dMapPointsAndVertices[pMapPoint] = pPointVertex;
        pPointVertex->setEstimate(Eigen::Vector3d(iPoint.x,
                                            iPoint.y,
                                            iPoint.z));
        // pPointVertex->setMarginalized(true);
        // pPointVertex->setFixed(true);
        iOptimizer.addVertex(pPointVertex);

        nVertexIndex++;
        cv::Point2f iObservation = gImagingPoints2[i];    
        g2o::EdgeProjectXYZ2UV * pEdge = new g2o::EdgeProjectXYZ2UV();
        pEdge->setId( nVertexIndex );
        //Linked with the position of 3d points.
        pEdge->setVertex(0 , pPointVertex);
        pEdge->setVertex(1 , gAllVertices[1]);
        //Set measurement and camera parameters.
        pEdge->setMeasurement(Eigen::Vector2d(iObservation.x, iObservation.y));
        pEdge->setParameterId(0 , nCameraParameterId);
        pEdge->setInformation(Eigen::Matrix2d::Identity());
        iOptimizer.addEdge(pEdge);
    }

  



    iOptimizer.initializeOptimization();


    iOptimizer.optimize(100);


    double nInitialError = CalculateAbsoluteError(gGroundTruthPoses, gPoses);


    for (int i=0;i<nVertexSize;i++){
    	Eigen::Isometry3d mUpdatedIsometry =  Eigen::Isometry3d(gAllVertices[i]->estimate());
    	Eigen::Matrix3d mRotation = mUpdatedIsometry.matrix().block(0 , 0 , 3 , 3);
    	Eigen::Matrix<double, 3, 1> mTranslation = mUpdatedIsometry.matrix().block(0 , 3 , 3 , 1);
        Sophus::SE3 mUpdatedPose =  Sophus::SE3(mRotation, mTranslation);
        gPoses[i] = mUpdatedPose;
    }

    double nCurrentError = CalculateAbsoluteError(gGroundTruthPoses, gPoses);
    cout << "Initial error is: " << nInitialError << endl;
    cout << "Current error is: " << nCurrentError << endl;

	return 0;
}