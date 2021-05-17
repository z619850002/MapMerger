
//Sophus
#include "sophus/se3.h"

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

//Std
#include <iostream>
#include <string>
#include <vector>

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

#include "../include/plotter/pose_plotter.h"
#include "../include/optimizer/smooth_optimizer.h"
#include "../include/optimizer/smooth_euler_optimizer.h"
using namespace std;


inline Sophus::SE3 TrajectoryInterpolate(
	Sophus::SE3 iStartPose, 
	Sophus::SE3 iEndPose, 
	double nRatio){
	Sophus::SE3 iRelativePose = iStartPose.inverse() * iEndPose;
	Eigen::Matrix<double,6,1> mRelativePose = iRelativePose.log();

	Eigen::Matrix<double,6,1> mInterpolation = mRelativePose * nRatio;

	return iStartPose * Sophus::SE3::exp(mInterpolation);
}


inline Sophus::SE3 TrajectoryInterpolateMulti(
	vector<Sophus::SE3> gAllPoses){
	double nRatio = 1.0/gAllPoses.size();
	Sophus::SE3 iStartPose = gAllPoses[0];
	for (int i=1;i<gAllPoses.size();i++){
		Sophus::SE3 iEndPose = gAllPoses[i];
		Sophus::SE3 iRelativePose = iStartPose.inverse() * iEndPose;
		Eigen::Matrix<double,6,1> mRelativePose = iRelativePose.log();

		Eigen::Matrix<double,6,1> mInterpolation = mRelativePose * nRatio;
		iStartPose *= Sophus::SE3::exp(mInterpolation);
	}

	return iStartPose;
}




vector<Sophus::SE3> SimulateTrajectory(Sophus::SE3 iStartPose, Sophus::SE3 iEndPose, int nNumber){
	//Random number generator
	cv::RNG iRNG(12345);
	double nSigma = 0.0000000000001;

	vector<Sophus::SE3> gPoses;
	gPoses.reserve(nNumber+2);
	//Sampled all poses
	gPoses.push_back(iStartPose);
	for (int i=1;i<nNumber;i++){
		double nRatio = 1.0/(double)nNumber * (double)i;
		Sophus::SE3 iSampledPose = TrajectoryInterpolate(
			iStartPose, 
			iEndPose, 
			nRatio);
		
		//Add noise to the smooth trajectory.
		double nNoiseX = iRNG.gaussian(nSigma);
		double nNoiseY = iRNG.gaussian(nSigma);
		double nNoiseZ = iRNG.gaussian(nSigma);
		double nNoiseR1 = iRNG.gaussian(nSigma);
		double nNoiseR2 = iRNG.gaussian(nSigma);
		double nNoiseR3 = iRNG.gaussian(nSigma);

		Eigen::Matrix<double, 6 , 1> mNoiseVec(6);
		mNoiseVec << nNoiseX, nNoiseY, nNoiseZ, nNoiseR1, nNoiseR2, nNoiseR3;

		iStartPose = Sophus::SE3::exp(iStartPose.log() + mNoiseVec);

		gPoses.push_back(iSampledPose);


	}

	
	return gPoses;
}



void initializePose(Sophus::SE3& T_FG,Sophus::SE3& T_LG,
					Sophus::SE3& T_BG,Sophus::SE3& T_RG)
{
	double nScale = 1;


	// Initialize T_FG
	Eigen::Matrix3d R_FG = Eigen::Matrix3d();
	R_FG<< 9.99277118e-01,  3.82390286e-04, -3.80143958e-02,
		  -2.30748265e-02, -7.88582447e-01, -6.14495953e-01,
	      -3.02124625e-02,  6.14928921e-01, -7.88003572e-01;




	Eigen::Vector3d t_FG;
	t_FG<< 6.75437418e-01,  2.50896883e+01,  3.17779305e+00;
	T_FG = Sophus::SE3(R_FG,t_FG/nScale);



	
	// Initialize T_LG
	Eigen::Matrix3d R_LG = Eigen::Matrix3d();
	R_LG<<-1.21898860e-02,  9.99924056e-01, -1.81349393e-03,
		   8.02363600e-01,  8.69913885e-03, -5.96772133e-01,
	      -5.96711036e-01, -8.72966581e-03, -8.02408707e-01;
	Eigen::Vector3d t_LG;
	t_LG<< 1.36392943e+00,  1.60942881e+01,  1.04105913e+01;
	T_LG = Sophus::SE3(R_LG,t_LG/nScale);
	
	// Initialize T_BG
	Eigen::Matrix3d R_BG = Eigen::Matrix3d();
	R_BG<<-9.99615699e-01,  1.56439861e-02, -2.28849354e-02,
		   2.59906371e-02,  8.16008735e-01, -5.77454960e-01,
	       9.64060983e-03, -5.77827838e-01, -8.16101739e-01;
	Eigen::Vector3d t_BG;
	t_BG<< 1.09266953e+00,  2.46308124e+01,  6.60957845e+00;
	T_BG = Sophus::SE3(R_BG,t_BG/nScale);

	// Initialize T_RG
	Eigen::Matrix3d R_RG = Eigen::Matrix3d();
	R_RG<< 4.57647596e-03, -9.99989102e-01,  9.22798184e-04,
		  -6.26343448e-01, -3.58584197e-03, -7.79538984e-01,
	       7.79533797e-01,  2.98955282e-03, -6.26353033e-01;
	Eigen::Vector3d t_RG;
	t_RG<<-1.66115120e-01,  1.76226207e+01, 6.08338205e+00;
	T_RG = Sophus::SE3(R_RG,t_RG/nScale);
	
	return;
}

vector<Sophus::SE3> SampleGTPose(){
	Sophus::SE3 mTF, mTL, mTB, mTR;
	initializePose(mTF, mTL, mTB, mTR);

	vector<Sophus::SE3> gPoses;

	gPoses = SimulateTrajectory(mTF, mTL, 16);

	vector<Sophus::SE3> gPoses2, gPoses3, gPoses4;
	gPoses2 = SimulateTrajectory(mTL, mTB, 16);
	gPoses3 = SimulateTrajectory(mTB, mTR, 16);
	gPoses4 = SimulateTrajectory(mTR, mTF, 16);
	gPoses.insert(gPoses.end(), gPoses2.begin(), gPoses2.end());
	gPoses.insert(gPoses.end(), gPoses3.begin(), gPoses3.end());
	gPoses.insert(gPoses.end(), gPoses4.begin(), gPoses4.end());
	return gPoses;
}

// vector<Sophus::SE3> BlurPose(vector<Sophus::SE3> & gOriginalPoses){
// 	vector<Sophus::SE3> gResultPoses;

// 	cv::RNG iRNG(12345);
// 	double nSigma = 0.007;

// 	for (int i=0;i<gOriginalPoses.size();i++){
// 		Sophus::SE3 mPose = gOriginalPoses[i];
// 		//Add noise to the smooth trajectory.
// 		double nNoiseX = iRNG.gaussian(nSigma);
// 		double nNoiseY = iRNG.gaussian(nSigma);
// 		double nNoiseZ = iRNG.gaussian(nSigma);
// 		double nNoiseR1 = iRNG.gaussian(nSigma);
// 		double nNoiseR2 = iRNG.gaussian(nSigma);
// 		double nNoiseR3 = iRNG.gaussian(nSigma);

// 		Eigen::Matrix<double, 6 , 1> mNoiseVec(6);
// 		mNoiseVec << nNoiseX, nNoiseY, nNoiseZ, nNoiseR1, nNoiseR2, nNoiseR3;

// 		mPose = Sophus::SE3::exp(mPose.log() + mNoiseVec);
// 		gResultPoses.push_back(mPose);
// 	}
// 	return gResultPoses;
// }


Sophus::SE3 GetRelativeTransform(	
	vector<Sophus::SE3>  gInitialPoses,
	vector<Sophus::SE3>  gOptimizedPoses){
	
	Sophus::SE3 mRelativePose;
	mRelativePose = gOptimizedPoses[0] * gInitialPoses[0].inverse();

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
    iOptimizer.setVerbose( false );


	const Eigen::Matrix<double,6,6> mInformation = Eigen::Matrix<double,6,6>::Identity();

	int nVertexId = 10;
	g2o::VertexSE3Expmap * pRefVertex = new g2o::VertexSE3Expmap();
	pRefVertex->setFixed(true);
	pRefVertex->setId(nVertexId++);
	pRefVertex->setEstimate( g2o::SE3Quat(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0.0 , 0.0 , 0.0)));
	iOptimizer.addVertex(pRefVertex);
	
	g2o::VertexSE3Expmap * pCurrVertex = new g2o::VertexSE3Expmap();
	pCurrVertex->setId(nVertexId++);
	pCurrVertex->setEstimate( g2o::SE3Quat(
		mRelativePose.rotation_matrix(),
		mRelativePose.translation()));
	iOptimizer.addVertex(pCurrVertex);


    //Add edges.
    int nEdgeIndex = nVertexId+10;
    for (int i=0;i<gInitialPoses.size();i++){
		Sophus::SE3 mPose1 = gInitialPoses[i];
		Sophus::SE3 mPose2 = gOptimizedPoses[i];
		Sophus::SE3 mEstimate = ( mPose2 * mPose1.inverse());

		g2o::EdgeSE3Expmap * pPoseGraphEdge = new g2o::EdgeSE3Expmap();
	    pPoseGraphEdge->setMeasurement(g2o::SE3Quat(
	        mEstimate.rotation_matrix(),
	        mEstimate.translation()));

	    pPoseGraphEdge->setId( nEdgeIndex++ );
        //Linked with the position of 3d points.
        pPoseGraphEdge->setVertex(0 , pRefVertex);
        pPoseGraphEdge->setVertex(1 , pCurrVertex);

		pPoseGraphEdge->setInformation(mInformation);
        // cout << "First: " << endl << pPoseGraphEdge->information() << endl;
		iOptimizer.addEdge(pPoseGraphEdge);
    }

	//Begin to optimize.
    iOptimizer.initializeOptimization();
    iOptimizer.optimize(100);

    //Load pose.
	Eigen::Isometry3d mUpdatedIsometry =  Eigen::Isometry3d(pCurrVertex->estimate());
	Eigen::Matrix3d mRotation = mUpdatedIsometry.matrix().block(0 , 0 , 3 , 3);
	Eigen::Matrix<double, 3, 1> mTranslation = mUpdatedIsometry.matrix().block(0 , 3 , 3 , 1);
    Sophus::SE3 mUpdatedPose =  Sophus::SE3(mRotation, mTranslation);


	return mUpdatedPose;
}


vector<Sophus::SE3> Optimize(	vector<Sophus::SE3>  gInitialPoses,
								vector<Sophus::SE3>  gRelativePoses){
	vector<Sophus::SE3> gFinalPoses;



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
    iOptimizer.setVerbose( false );


	const Eigen::Matrix<double,6,6> mInformation = Eigen::Matrix<double,6,6>::Identity();


    //Add vertices.
    vector<g2o::VertexSE3Expmap *> gAllVertices;
    int nVertexId = 10;
    int nVertexSize = gInitialPoses.size();
    for (int i=0;i<nVertexSize;i++){
    	Sophus::SE3 mPose = gInitialPoses[i];

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

    //Add edges.
    int nEdgeIndex = nVertexId+10;
    for (int i=3;i<gAllVertices.size();i++){

    	for (int j=i-3;j<i;j++){

    		Sophus::SE3 mPose1 = gInitialPoses[i];
    		Sophus::SE3 mPose2 = gInitialPoses[j];
    		Sophus::SE3 mEstimate = ( mPose2 * mPose1.inverse());

    		g2o::EdgeSE3Expmap * pPoseGraphEdge = new g2o::EdgeSE3Expmap();
		    pPoseGraphEdge->setMeasurement(g2o::SE3Quat(
		        mEstimate.rotation_matrix(),
		        mEstimate.translation()));

		    pPoseGraphEdge->setId( nEdgeIndex++ );
            //Linked with the position of 3d points.
            pPoseGraphEdge->setVertex(0 , gAllVertices[i]);
            pPoseGraphEdge->setVertex(1 , gAllVertices[j]);

			pPoseGraphEdge->setInformation(mInformation);
            // cout << "First: " << endl << pPoseGraphEdge->information() << endl;
    		iOptimizer.addEdge(pPoseGraphEdge);

    	}
    }

    //Add the loop edge.
	g2o::EdgeSE3Expmap * pPoseGraphEdge = new g2o::EdgeSE3Expmap();
	Sophus::SE3 mRelativePose = gRelativePoses[gRelativePoses.size()-1];
    pPoseGraphEdge->setMeasurement(g2o::SE3Quat(
        mRelativePose.rotation_matrix(),
        mRelativePose.translation()));

    pPoseGraphEdge->setId( nEdgeIndex++ );
    //Linked with the position of 3d points.
    pPoseGraphEdge->setVertex(0 , gAllVertices[gAllVertices.size()-1]);
    pPoseGraphEdge->setVertex(1 , gAllVertices[0]);

	pPoseGraphEdge->setInformation(mInformation*3);
	iOptimizer.addEdge(pPoseGraphEdge);

	//Begin to optimize.
    iOptimizer.initializeOptimization();
    iOptimizer.optimize(100);

    //Load pose.

    for (int i=0;i<nVertexSize;i++){
    	Eigen::Isometry3d mUpdatedIsometry =  Eigen::Isometry3d(gAllVertices[i]->estimate());
    	Eigen::Matrix3d mRotation = mUpdatedIsometry.matrix().block(0 , 0 , 3 , 3);
    	Eigen::Matrix<double, 3, 1> mTranslation = mUpdatedIsometry.matrix().block(0 , 3 , 3 , 1);
        Sophus::SE3 mUpdatedPose =  Sophus::SE3(mRotation, mTranslation);
        gFinalPoses.push_back(mUpdatedPose);
    }

	return gFinalPoses;
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
	nError /= gGroundTruth.size();

	return nError;
}


vector<Sophus::SE3> SimpleOptimize(
		vector<Sophus::SE3> gInitialPoses,
		vector<Sophus::SE3> gRelativePoses){
	vector<Sophus::SE3> gFinalPoses;
	gFinalPoses.reserve(gInitialPoses.size());
	for (int i=0;i<gInitialPoses.size();i++){
		gFinalPoses.push_back(gInitialPoses[i]);
	}

	for (int nIter = 0;nIter < 10;nIter++){
		for (int i=gFinalPoses.size()-1;i>=1;i--){
			int nMatchSize = 3;
			if (gFinalPoses.size()-1-i<3){
				gFinalPoses.size()-1-i;
			}
			//Find correspondence frames.
			if (i==gFinalPoses.size()-1){
				nMatchSize +=1;
			}
			vector<Sophus::SE3> gPoses;

			for (int nShift = -1;nShift <=4;nShift++){
				int nIndex= i+nShift;
				if (nShift==0){
					continue;
				}
				if (nIndex > gFinalPoses.size()-1 || nIndex <=0){
					break;
				}
				Sophus::SE3 mRelativePose = gInitialPoses[i] * gInitialPoses[nIndex].inverse();
				Sophus::SE3 mEstimatePose1 = mRelativePose * gFinalPoses[nIndex];
				mEstimatePose1 = TrajectoryInterpolate(mEstimatePose1, gFinalPoses[i], 0.5);
				gPoses.push_back(mEstimatePose1);
			}
			if (i==gFinalPoses.size()-1){
				Sophus::SE3 mEstimatePose1 = gRelativePoses[gRelativePoses.size()-1].inverse() *gFinalPoses[0];
				mEstimatePose1 = TrajectoryInterpolate(mEstimatePose1, gFinalPoses[i], 0.5);
				gPoses.push_back(mEstimatePose1);
				gPoses.push_back(mEstimatePose1);
				gPoses.push_back(mEstimatePose1);
			}
			if (gPoses.size()==0){
				continue;
			}
			Sophus::SE3 mUpdatedPose = TrajectoryInterpolateMulti(gPoses);
			gFinalPoses[i] = mUpdatedPose;
		}
	}
	return gFinalPoses;
}




// vector<Vector6d> SimpleOptimizeClass(
// 		vector<Ve> gInitialPoses,
// 		vector<Sophus::SE3> gRelativePoses){
// 	vector<Sophus::SE3> gFinalPoses;
// 	int nNodeSize = gInitialPoses.size();
// 	SmoothOptimizer * pOptimizer = new SmoothOptimizer(nNodeSize);
// 	//Bind Poses.
// 	for (int i=0;i<nNodeSize;i++){
// 		pOptimizer->AddKeyFramePose(gInitialPoses[i]);
// 	}

// 	int nConnectSize = 4;
// 	for (int i=nNodeSize-1;i>=1;i--){
// 		for (int nShift = 1;nShift<nConnectSize;nShift++){
// 			int nConnectIndex = i+nShift;
// 			if (nConnectIndex >= nNodeSize){
// 				continue;
// 			}
// 			Sophus::SE3 mPose_rc = gInitialPoses[i] * gInitialPoses[nConnectIndex].inverse();
// 			pOptimizer->AddConnections(i, nConnectIndex, mPose_rc);
// 		}
// 	}

// 	pOptimizer->AddConnections(nNodeSize-1, 0, gRelativePoses[gRelativePoses.size()-1].inverse());
// 	pOptimizer->Optimize();

// 	gFinalPoses = pOptimizer->LoadPoses();

// 	return gFinalPoses;
// }







Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
}



Vector6d ConvertSophusToPose(Sophus::SE3 mPoseSophus){
	Vector6d mResult;
	Eigen::Vector3d mEuler = R2ypr(mPoseSophus.rotation_matrix());
	mResult(0 , 0) = mEuler(0 , 0);
	mResult(1 , 0) = mEuler(1 , 0);
	mResult(2 , 0) = mEuler(2 , 0);

	mResult(3 , 0) = mPoseSophus.translation()(0 , 0);
	mResult(4 , 0) = mPoseSophus.translation()(1 , 0);
	mResult(5 , 0) = mPoseSophus.translation()(2 , 0);
	return mResult;
}




Sophus::SE3 ConvertPoseToSophus(Vector6d mPose){
	Eigen::Matrix3d mRotation_wc;
	YawPitchRollToRotationMatrix(mPose(0 , 0), mPose(1 , 0), mPose(2 , 0), mRotation_wc);
	Eigen::Vector3d mTranslation_wc(
			mPose(3 , 0),
			mPose(4 , 0),
			mPose(5 , 0)); 
	Sophus::SE3 mPoseSophus(mRotation_wc, mTranslation_wc);
	return mPoseSophus;
}



vector<Vector6d> BlurPose(vector<Vector6d> & gOriginalPoses){
	vector<Vector6d> gResultPoses;

	cv::RNG iRNG(12345);
	double nSigma = 0.1;

	for (int i=0;i<gOriginalPoses.size();i++){
		Vector6d mPose = gOriginalPoses[i];
		//Add noise to the smooth trajectory.
		double nNoiseX = iRNG.gaussian(nSigma);
		double nNoiseY = iRNG.gaussian(nSigma);
		double nNoiseZ = iRNG.gaussian(nSigma);
		double nNoiseR1 = iRNG.gaussian(nSigma*10);
		double nNoiseR2 = iRNG.gaussian(nSigma*10);
		double nNoiseR3 = iRNG.gaussian(nSigma*10);

		Vector6d mFinalPose;
		mFinalPose << 	mPose(0 , 0) + nNoiseR1  , mPose(1 , 0), mPose(2 , 0),
						mPose(3 , 0) + nNoiseX, mPose(4 , 0)+nNoiseY , mPose(5 , 0) + nNoiseZ;

		gResultPoses.push_back(mFinalPose);
	}
	return gResultPoses;
}




int main(){
	
	vector<Sophus::SE3> gPosesSophus_wc = SampleGTPose();
	//Convert to wc.
	for (int i=0;i<gPosesSophus_wc.size();i++){
		gPosesSophus_wc[i] = gPosesSophus_wc[i].inverse();
	}


	// Sophus::SE3 mLoopPose = gPosesSophus_wc[0].inverse() * gPosesSophus_wc[gPosesSophus_wc.size()-1];

	//Convert to Euler and translation.
	vector<Vector6d> gInitialPoses;
	for (int i=0;i<gPosesSophus_wc.size();i++){
		gInitialPoses.push_back(ConvertSophusToPose(gPosesSophus_wc[i]));
	}

	//Disturb on yaw and translations.
	vector<Vector6d> gRelativePoses;
	for (int i=0;i<gPosesSophus_wc.size();i++){
		if (i == gPosesSophus_wc.size()-1){
			gRelativePoses.push_back(gInitialPoses[0]-gInitialPoses[i]);
		}else{
			gRelativePoses.push_back(gInitialPoses[i+1]-gInitialPoses[i]);
			// gInitialPoses.push_back(ConvertSophusToPose(gPosesSophus_wc[i]));
		}
	}

	gRelativePoses = BlurPose(gRelativePoses);

	vector<Vector6d> gDisturbedPoses;
	gDisturbedPoses.push_back(gInitialPoses[0]);
	for (int i=0;i<gInitialPoses.size()-1;i++){
		gDisturbedPoses.push_back(gDisturbedPoses[i]+gRelativePoses[i]);
	}




	//Begin to optimize.
	int nNodeSize = gInitialPoses.size();
	SmoothEulerOptimizer * pOptimizer = new SmoothEulerOptimizer(nNodeSize);
	//Bind Poses.
	for (int i=0;i<nNodeSize;i++){
		pOptimizer->AddKeyFramePose(gDisturbedPoses[i]);
	}


	int nConnectSize = 4;
	for (int i=nNodeSize-1;i>=1;i--){
		for (int nShift = -2;nShift<nConnectSize;nShift++){
			int nConnectIndex = i+nShift;
			if (nConnectIndex >= nNodeSize || nConnectIndex <1){
				continue;
			}
			//wp.inv * wo
			Sophus::SE3 mPoseSophus_wp = gPosesSophus_wc[nConnectIndex];
			Sophus::SE3 mPoseSophus_wo = gPosesSophus_wc[i];
			double nYaw_po = ConvertSophusToPose(mPoseSophus_wp)(0 , 0) - ConvertSophusToPose(mPoseSophus_wo)(0 , 0);
			double nYaw2 = gInitialPoses[nConnectIndex](0 , 0) - gInitialPoses[i](0 , 0);
			nYaw_po = nYaw_po;
			Sophus::SE3 mPoseSophus_po = gPosesSophus_wc[nConnectIndex].inverse() * gPosesSophus_wc[i];
			Eigen::Vector4d mPose_po;
			Vector6d mPose6_po = ConvertSophusToPose(mPoseSophus_po);
			mPose_po(0 , 0)  = nYaw_po;
			mPose_po(1 , 0)  = mPose6_po(3 , 0);
			mPose_po(2 , 0)  = mPose6_po(4 , 0);
			mPose_po(3 , 0)  = mPose6_po(5 , 0);

			pOptimizer->AddConnections(i, nConnectIndex, mPose_po);
		}
	}

	Sophus::SE3 mPoseSophus_wp = gPosesSophus_wc[0];
	Sophus::SE3 mPoseSophus_wo = gPosesSophus_wc[nNodeSize-1];
	double nYaw_po = ConvertSophusToPose(mPoseSophus_wp)(0 , 0) - ConvertSophusToPose(mPoseSophus_wo)(0 , 0);
	nYaw_po = nYaw_po;
	Sophus::SE3 mPoseSophus_po = gPosesSophus_wc[0].inverse() * gPosesSophus_wc[nNodeSize-1];
	Eigen::Vector4d mLoopPose_po;
	Vector6d mPose6_po = ConvertSophusToPose(mPoseSophus_po);
	mLoopPose_po(0 , 0)  = nYaw_po;
	mLoopPose_po(1 , 0)  = mPose6_po(3 , 0);
	mLoopPose_po(2 , 0)  = mPose6_po(4 , 0);
	mLoopPose_po(3 , 0)  = mPose6_po(5 , 0);
	pOptimizer->AddConnections(nNodeSize-1, 0, mLoopPose_po);
	

	pOptimizer->Optimize();


	vector<Vector6d> gFinalPoses = pOptimizer->LoadPoses();
	vector<Sophus::SE3> gFinalSophusPoses;
	for (int i=0;i<gFinalPoses.size();i++){
		gFinalSophusPoses.push_back(ConvertPoseToSophus(gFinalPoses[i]).inverse());
	}


	PosePlotter * pPlotter = new PosePlotter();
	for (auto mPose : gFinalSophusPoses){
		pPlotter->AddKeyFrame(mPose);	
	}
	pPlotter->Run();


	return 0;
}